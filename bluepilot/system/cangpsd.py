#!/usr/bin/env python3
"""Publish the Ford CAN GPS solution as this device's GPS location.

The comma device's ublox GPS can never get a fix in this Mach-E: the
IR-reflective windshield blocks the antenna and there's no uncoated zone to
mount in. The car itself broadcasts a full GPS solution on CAN bus 0 at
~1 Hz (APIMGPS_Data_Nav_1/2/3_FD1, DBC ford_lincoln_base_pt.dbc). This daemon
decodes it and republishes it so system/timed.py, selfdrived.py,
speed_limit_resolver.py, mapd, and athenad all keep working unmodified
(locationd in this tree is camera+IMU only and never reads GPS). The device's
own GPS daemon is gated off elsewhere so msgq's one-publisher-per-topic rule
doesn't collide.

Which topic that is depends on the device, and we do not get to choose: consumers pick
between gpsLocationExternal and gpsLocation on UbloxAvailable (common/gps.py, and
locationd.cc in trees whose locationd reads GPS), so the topic is resolved through the same
get_gps_location_service() the consumers use. gpsLocationExternal where ublox_available()
is true, gpsLocation where it is not. Publishing the other one reaches nobody, which is
what a comma 3X user saw when the toggle appeared to do nothing.

Do not shortcut that to a device model. ublox_available() is a probe, not a lookup: it
tests for /dev/ttyHS0 and for the /persist/comma/use-quectel-gps override, so a board with
both receivers can be flipped either way and one model does not imply one answer. Every
comma 3X is Quectel; the comma 4 this daemon was developed and driven on (model string
"comma mici") probes ublox -- /dev/ttyHS0 present, no override, UbloxAvailable 1, measured
on the device. That is one device, not a claim about every comma 4.

The daemon runs in one of two modes. As the selected GPS source it publishes, as it always
has. Otherwise it runs as an observer: it decodes the same CAN messages but sends nothing,
and instead watches whether the device receiver is producing fixes -- see cangps_fallback,
which owns the decision of when to take the topic over. Decoding CAN costs nothing extra
while the device daemon owns the topic, since the one-publisher rule constrains sending,
not reading.

Decode is split into pure functions (decode_utc, decode_position,
decode_quality) that take plain dicts of already-scaled DBC signal values, so
they can be unit-tested and replayed offline against a recorded rlog without
any cereal sockets -- see comma-gps-time/scripts/decode_can_gps.py.
"""
import datetime
import enum
import math
import sys
import time
from dataclasses import dataclass
from typing import NoReturn

import capnp
from cereal import car, log
import cereal.messaging as messaging

from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC

from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.bluepilot.system.cangps_fallback import (
  MIN_SPEED,
  SOURCE_CAN,
  FallbackArbiter,
  json_state,
  load_state,
  save_state,
  vehicle_key,
)

# The topic to publish on when nobody says otherwise. main() asks
# get_gps_location_service() instead; this is only the default for direct callers -- the
# unit tests and the offline replay scripts, which have no Params to consult.
GPS_SERVICE_DEFAULT = 'gpsLocationExternal'

# APIMGPS_Data_Nav_1_FD1: lat/lon degrees+minutes and hemisphere enums
GPS_ADDR_POS = 0x462
# APIMGPS_Data_Nav_2_FD1: UTC date/time, PDOP, fault flag
GPS_ADDR_TIME = 0x463
# APIMGPS_Data_Nav_3_FD1: altitude, speed, heading, sat count, hdop/vdop
GPS_ADDR_QUALITY = 0x464

# The one place a GPS message is declared. Both the CANParser and decode_gps_frames()'s
# filter are built from this, because they must not disagree: the filter runs first, so a
# message the parser wants but the filter omits never reaches it and decodes as permanently
# absent -- no error, no log, just a signal that is always stale.
#
# Kept as one tuple rather than two lists specifically so that porting this daemon to
# another brand cannot reintroduce that gap. Everything else here is Ford-shaped (the DBC
# and bus lookup in parser_config, the signal names in the decode functions, the sentinel
# encodings), so a port rewrites plenty -- but those failures are loud, and this one would
# not be.
GPS_MESSAGES = (
  ("APIMGPS_Data_Nav_1_FD1", GPS_ADDR_POS),
  ("APIMGPS_Data_Nav_2_FD1", GPS_ADDR_TIME),
  ("APIMGPS_Data_Nav_3_FD1", GPS_ADDR_QUALITY),
)
GPS_ADDRS = frozenset(address for _, address in GPS_MESSAGES)

# capnp refuses to walk a large message without this. pandad's helper sets the same thing;
# the `can` topic on a CAN FD car is well past the default limit.
CAPNP_NO_TRAVERSAL_LIMIT = 2**64 - 1

# both 0x462 and 0x463 are ~1 Hz; beyond this we can no longer trust the held fix
MAX_FIX_AGE = 3.0

# Publishing is phase-locked to the CAN burst rather than free-running: send as soon as
# a new 0x463 lands, then a held repeat if the topic would otherwise go quiet.
#
# Why not just pass 1 Hz straight through: SubMaster.alive is
# (now - last_recv) < 10 / declared_freq, and services.py declares this topic at 10 Hz,
# giving a 1.0 s window. Measured 0x463 spacing on this car is min 0.974 / median 0.991 /
# max 1.030 s -- 34% of intervals exceed 1.0 s, which would leave `alive` false ~1% of the
# time. One consumer cares: hardwared.py builds the server STATUS_PACKET with
# `location if sm.alive[...] else None`, so a passthrough would occasionally report no
# position at all.
#
# Why not a free-running cadence: a fresh fix would then wait up to a full publish period
# before anyone saw it, on top of the offset the car itself introduces (0.4-0.9 s, see
# decode_utc). Latency is a systematic along-track position error, which matters more for
# map matching than the duplicate count does, and unlike the car's offset this part is
# ours to remove.
KEEPALIVE_INTERVAL = 0.5

# how often to look for the live CarParams while running off the previous route's copy
CP_CHECK_INTERVAL = 1.0

# Only the CAN FD Ford platforms broadcast 0x463; the classic-CAN ones (Escape MK4,
# Explorer MK6, Focus MK4, Maverick MK1, Bronco Sport MK1) send 0x462 and mostly 0x464 but
# never the time message. This split holds across every public segment measured so far,
# both a first spot-check and a later, broader run across more platforms -- see
# bluepilot/system/CANGPS_PLATFORMS.md for the measurement detail and counts.
#
# Position is worth publishing on its own -- mapd, speed limits, athenad and the UI all
# want lat/lon and none of them read unixTimestampMillis (timed.py is its only real
# consumer). So a car with no 0x463 gets everything except the clock, which is the one
# thing it could never have had.
#
# "No 0x463 on this platform" is only distinguishable from "the first 0x463 hasn't landed
# yet" by waiting. Both messages are ~1 Hz, so several seconds of position frames with no
# time frame settles it. Until the grace expires we publish nothing, so a car that does
# have 0x463 never emits a position-only sample on the way up.
NO_TIME_GRACE = 5.0

# Stand-ins for "the car never told us", used when 0x464 is absent or reports a sentinel.
# 5.8 is the DBC's maximum hdop/vdop and 5.0 the nominal metres of error per dop unit, so
# 30 m is a hair past the worst accuracy the signal is capable of expressing. Bearing gets
# 180 deg (any direction) and speed 50 m/s (any road speed) -- both saturated rather than
# invented, since there is no honest middle value.
UNKNOWN_ACCURACY = 30.0
UNKNOWN_SPEED_ACCURACY = 50.0
UNKNOWN_BEARING_ACCURACY = 180.0

# Accuracy floor for a dead-reckoned (GPS_Actual_vs_Infer_pos) position. The APIM's DR is
# good: across two SR-99 tunnel transits (356 frames / 5m55s and 137 frames / 2m16s) the
# position jump on regaining a real fix was 24 m and 19 m, against a normal 1 Hz
# sample-to-sample step of p50 7-10 m and p95 26 m -- i.e. no discontinuity resolvable at
# this sample rate, and every jump explainable as the car simply moving. So the error is
# bounded by tens of metres, not measured, and 30 m sits just past that bound. Same value
# and same reasoning as UNKNOWN_ACCURACY ("no better than anything the signal could have
# expressed"), kept separate because it answers a different question and may move alone.
#
# Deliberately a constant, not a ramp. Real DR error grows with time since the last true
# fix, so this understates a long outage -- but the longest measured here is six minutes
# and showed no resolvable error, so there is nothing to fit a growth rate to. A ramp
# would be an invented number wearing a formula.
INFERRED_ACCURACY_FLOOR = 30.0


def should_publish(fresh_fix: bool, now: float, last_publish: float) -> bool:
  """Publish on a fresh 0x463, else once the topic is about to go stale.

  With CAN at ~1 Hz and a 0.5 s keepalive the repeat fires every cycle, so this is
  ~2 Hz in practice -- one fresh sample and one held repeat per second. The point is
  not the conditional (it is effectively unconditional at this source rate) but the
  phase: fresh data goes out on arrival instead of waiting for the next tick.
  """
  if fresh_fix:
    return True
  return (now - last_publish) >= KEEPALIVE_INTERVAL


def decode_utc(vl: dict) -> datetime.datetime | None:
  """Decode the UTC timestamp out of 0x463, or None if the sample isn't usable.

  Plain range checks subsume every DBC fault sentinel: year raw 31 -> 2041,
  month raw 15 -> 16, day raw 31 -> 32, hour raw 30/31, second raw 62/63 all
  land outside the legal range below, so there's no need to special-case them.
  This path is confirmed correct against a real rlog.

  The label is whole seconds and reads behind true UTC -- measured against NTP at
  -0.400 s on one drive, -0.905 and -0.790 on two others. Stable to +-50 ms within
  a drive, half a second apart between boots, so it is the phase of the APIM's ~1 Hz
  emission timer relative to UTC, re-rolled at each power-up, not a transport delay.
  No constant can correct it, and the phase cannot be recovered from the bus: that
  needs an external reference, and the only one is NTP, which is present exactly when
  this daemon is not needed. Deliberately uncorrected. timed.py acts on a 10 s
  threshold, so it never matters there; it costs 12-27 m of along-track position lag
  at highway speed, which reaches map matching only.
  """
  if vl["Gps_B_Falt"] != 0:
    return None
  pdop = vl["GPS_Pdop"]
  if not (0 < pdop <= 5.0):
    return None
  year, month, day = int(vl["GpsUtcYr_No_Actl"]), int(vl["GpsUtcMnth_No_Actl"]), int(vl["GpsUtcDay_No_Actl"])
  hour, minute, second = int(vl["GPS_UTC_hours"]), int(vl["GPS_UTC_minutes"]), int(vl["GPS_UTC_seconds"])
  if not (2024 <= year <= 2040 and 1 <= month <= 12 and 1 <= day <= 31 and hour <= 23 and minute <= 59 and second <= 59):
    return None
  try:
    return datetime.datetime(year, month, day, hour, minute, second, tzinfo=datetime.UTC)
  except ValueError:
    # e.g. Feb 30 -- a legal-range but calendar-invalid date
    return None


def decode_position(vl: dict) -> tuple[float, float, int, int] | None:
  """Decode lat/lon out of 0x462, or None if out of range or the (0, 0) sentinel.

  The DBC gives GPS_Latitude_Degrees/GPS_Longitude_Degrees offsets of -89/-179,
  so CANParser already returns signed degrees and the GpsHsphLattSth_D_Actl/
  GpsHsphLongEast_D_Actl hemisphere enums are redundant. Verified against
  recorded rlogs (routes 0x101, 0x106, 0xf1, all in Seattle): signed degrees
  alone give the correct position, longitude negative for west. Cross-checked
  by differentiating position -- the implied speed and heading (17.2 m/s at
  41.9 deg) match what the car independently reports in 0x464 (17.0-17.4 m/s
  at 41.5 deg), so the decode is right in both magnitude and sign.

  Both hemisphere enums read a constant 2 on every sample, north+west, so they
  are not a south/east boolean pair. Left unused; still returned so the offline
  decoder can surface them if another vehicle disagrees.

  The degrees sentinels (raw 254/255 on GPS_Latitude_Degrees, decoded 165/166) are caught
  for free by the |lat| > 90 check below. The minutes sentinels are not: GPS_Latitude/
  Longitude_Minutes raw 62 Unknown/63 Fault (scale 1) and _Min_dec raw 16382 Unknown/16383
  Invalid (scale 0.0001) all land inside a plausible minutes range, worth up to 1.077 deg
  (~120 km) of silent error if passed through. Checked explicitly, mirroring decode_quality.
  """
  def is_sentinel(sig: str, scale: float, *sentinels: int) -> bool:
    return round(vl[sig] / scale) in sentinels

  if (is_sentinel("GPS_Latitude_Minutes", 1, 62, 63) or is_sentinel("GPS_Longitude_Minutes", 1, 62, 63)
      or is_sentinel("GPS_Latitude_Min_dec", 0.0001, 16382, 16383)
      or is_sentinel("GPS_Longitude_Min_dec", 0.0001, 16382, 16383)):
    return None

  lat_deg = vl["GPS_Latitude_Degrees"]
  lat_min = vl["GPS_Latitude_Minutes"] + vl["GPS_Latitude_Min_dec"]
  lon_deg = vl["GPS_Longitude_Degrees"]
  lon_min = vl["GPS_Longitude_Minutes"] + vl["GPS_Longitude_Min_dec"]

  lat_sign = -1.0 if lat_deg < 0 else 1.0
  lon_sign = -1.0 if lon_deg < 0 else 1.0
  lat = lat_sign * (abs(lat_deg) + lat_min / 60.0)
  lon = lon_sign * (abs(lon_deg) + lon_min / 60.0)

  hemi_lat_raw = int(vl["GpsHsphLattSth_D_Actl"])
  hemi_lon_raw = int(vl["GpsHsphLongEast_D_Actl"])

  if abs(lat) > 90.0 or abs(lon) > 180.0:
    return None
  if lat == 0.0 and lon == 0.0:
    # never-had-a-fix sentinel, not a real position off the coast of Africa
    return None
  return lat, lon, hemi_lat_raw, hemi_lon_raw


# Everything 0x464 carries, with nothing known about any of it. Used when the car does not
# send 0x464 at all (Focus MK4 sends 0x462 and nothing else) and before the first frame.
QUALITY_UNKNOWN: dict = {"altitude": None, "speed": None, "bearing_deg": None,
                         "hdop": None, "vdop": None, "sat_count": 0}


def decode_inferred(vl: dict) -> bool:
  """Is the position in 0x462 dead-reckoned rather than measured?

  GPS_Actual_vs_Infer_pos is a plain 1-bit enum in 0x463 (0 Actual_Postition,
  1 Inferred_Position) with no sentinel, so there is nothing to range-check.

  This is not a hasFix input. The APIM dead-reckons well -- measured over two SR-99
  tunnel transits it tracked ~3.1 km of enclosed roadway -- and a tunnel or garage is
  exactly where this daemon matters most, since the GPS clock stays valid while inferred
  and that is the one thing the device cannot get anywhere else. Dropping the fix here
  would throw away a usable position to no end. What the flag changes is the *claimed
  accuracy*; see build_gps_msg.
  """
  return vl["GPS_Actual_vs_Infer_pos"] != 0


def decode_quality(vl: dict) -> dict:
  """Decode the secondary fix-quality fields out of 0x464, None for anything unusable.

  Not part of the hasFix gate (only 0x462/0x463 freshness and validity are), but used to
  fill in the rest of the published message. Every field here has Unknown/Invalid/Fault
  sentinels in the DBC, and a sentinel passed through the scale factor becomes a plausible
  reading -- GPS_Speed 255 would publish as 114 m/s. None of them fire on the Mach-E (722
  frames across routes 0x10b and 0x10c: only GPS_Sat_num_in_view, constantly), so this is
  guarding a latent case rather than an observed one.
  """
  def valid(sig: str, scale: float, offset: float, *sentinels: int) -> float | None:
    raw = round((vl[sig] - offset) / scale)
    return None if raw in sentinels else vl[sig]

  # GPS_Sat_num_in_view is a 5-bit field the DBC bounds at [0|29], so 30/31 are sentinels.
  # Every local route sampled reports a constant 31, but that is not a property of the car:
  # across four public Mach-E segments the count is real about a quarter of the time, and
  # two segments from one vehicle on identical firmware disagree. It is intermittent, not
  # absent. Report 0 (unknown) rather than a fabricated 31 satellites.
  sat_count = int(vl["GPS_Sat_num_in_view"])
  altitude = valid("GPS_MSL_altitude", 10, -20460, 4094, 4095)
  speed = valid("GPS_Speed", 1, 0, 254, 255)
  return {
    # GPS_MSL_altitude is geoid height (mean sea level); GpsLocationData.altitude
    # is documented as WGS84 ellipsoid height. The two differ by ~35 m in Seattle.
    # Accepted uncorrected -- we have no geoid model on the device.
    "altitude": None if altitude is None else altitude * 0.3048,
    "speed": None if speed is None else speed * 0.44704,
    "bearing_deg": valid("GPS_Heading", 0.01, 0, 65534, 65535),
    "hdop": valid("GPS_Hdop", 0.2, 0, 30, 31),
    "vdop": valid("GPS_Vdop", 0.2, 0, 30, 31),
    "sat_count": sat_count if sat_count <= 29 else 0,
  }


def build_gps_msg(lat: float, lon: float, altitude: float | None, speed: float | None,
                   bearing_deg: float | None, unix_timestamp_millis: int, hdop: float | None,
                   vdop: float | None, sat_count: int,
                   has_fix: bool, inferred: bool = False,
                   service: str = GPS_SERVICE_DEFAULT) -> capnp.lib.capnp._DynamicStructBuilder:
  """Build a GPS location message from already-decoded plain values.

  `service` is the topic to shape the message for -- gpsLocationExternal or gpsLocation.
  Both carry the same GpsLocationData, so this only picks which union field to fill. It
  defaults to gpsLocationExternal so callers and tests that predate the gpsLocation path,
  and the offline replay scripts, are unchanged.

  A None means the car never told us. GpsLocationData has no way to say "unknown" for a
  value, only an accuracy alongside it, so the value goes out as 0 and the matching
  accuracy is widened to cover the whole plausible range. That is the difference between
  publishing 0 m/s because the car is stopped and publishing 0 m/s because we have no idea
  -- a consumer that reads accuracy can tell them apart, and one that ignores accuracy is
  no worse off than it would be with a fabricated number.
  """
  msg = messaging.new_message(service, valid=True)
  gps = getattr(msg, service)
  gps.latitude = lat
  gps.longitude = lon
  gps.altitude = altitude if altitude is not None else 0.0
  gps.speed = speed if speed is not None else 0.0
  gps.bearingDeg = bearing_deg if bearing_deg is not None else 0.0
  gps.unixTimestampMillis = unix_timestamp_millis
  gps.source = log.GpsLocationData.SensorSource.car
  bearing_rad = math.radians(gps.bearingDeg)
  gps.vNED = [gps.speed * math.cos(bearing_rad), gps.speed * math.sin(bearing_rad), 0.0]
  # Kept strictly positive: nothing in this tree filters on accuracy today (locationd
  # does not read GPS here), but upstream's locationd discards a sample whose accuracies
  # are not all > 0, so a nominal floor keeps the message usable by any such consumer
  # rather than leaving it at raw (possibly zero, e.g. before the first 0x464) hdop/vdop.
  # The DBC caps hdop/vdop at 5.8, so UNKNOWN_ACCURACY is one step past the worst the car
  # could have reported -- "no better than anything expressible", not an invented number.
  horizontal = max(1.0, hdop * 5.0) if hdop is not None else UNKNOWN_ACCURACY
  # verticalAccuracy has to widen on an unknown altitude too, not just an unknown vdop --
  # otherwise a missing GPS_MSL_altitude (published as 0.0 above) pairs with a valid vdop
  # to publish a confident sea-level reading. Speed/bearing below already couple this way.
  vertical = max(1.0, vdop * 5.0) if (vdop is not None and altitude is not None) else UNKNOWN_ACCURACY
  # A dead-reckoned position is not as good as its dop values claim, and the dop values do
  # not know that: PDOP correlates with the inferred flag but does not track it, because on
  # a short outage the APIM's error has not accumulated yet and it stays confident. So the
  # faster the tunnel, the more confident the wrong position -- exactly backwards. Floor
  # both accuracies rather than trusting the reported dop; see INFERRED_ACCURACY_FLOOR.
  if inferred:
    horizontal = max(horizontal, INFERRED_ACCURACY_FLOOR)
    vertical = max(vertical, INFERRED_ACCURACY_FLOOR)
  gps.horizontalAccuracy = horizontal
  gps.verticalAccuracy = vertical
  gps.speedAccuracy = 0.5 if speed is not None else UNKNOWN_SPEED_ACCURACY
  gps.bearingAccuracyDeg = 5.0 if bearing_deg is not None else UNKNOWN_BEARING_ACCURACY
  gps.hasFix = has_fix
  gps.satelliteCount = sat_count
  return msg


class TimeSource(enum.Enum):
  """Whether this platform gives us a GPS clock. Decided from what has arrived on the bus."""
  UNKNOWN = enum.auto()  # no 0x463 yet, and NO_TIME_GRACE has not expired: keep waiting
  CAN = enum.auto()      # 0x463 has been seen at least once: this platform has a clock
  NONE = enum.auto()     # 0x462 for a full grace period with no 0x463: position-only platform


@dataclass
class FixTracker:
  """The freshness state behind hasFix and unixTimestampMillis.

  main() feeds this what it sees on the bus -- a position frame, a quality frame, a time
  frame with or without a decodable UTC -- stamped with time.monotonic(), and asks it the
  questions the published message needs. Everything is monotonic seconds; nothing here
  reads a clock, so the whole thing is testable by advancing `now` by hand.

  Two rules that are easy to get wrong live here, both settled against real Mach-E logs:

  - A 0x463 frame that fails decode_utc (transient Gps_B_Falt or PDOP sentinel, ~3 in 240
    frames) still proves the platform *has* a time message, so it counts for time_source
    and for the publish phase-lock. It must not count as a fresh fix: that feeds
    MAX_FIX_AGE, and letting a faulted frame advance it would let a sustained fault hold
    hasFix true forever while every sample is being rejected.
  - "No 0x463 on this platform" is only distinguishable from "the first 0x463 hasn't
    landed yet" by waiting. Until NO_TIME_GRACE expires we report no fix at all, so a car
    that does have 0x463 never emits a position-only sample on the way up.
  """
  last_seen_pos: float = 0.0
  last_seen_time: float = 0.0  # last *decodable* 0x463, see observe_time
  last_seen_quality: float = 0.0
  first_seen_pos: float = 0.0
  ever_saw_time: bool = False
  # the last decoded 0x463 and the monotonic reading taken at that moment; the published
  # timestamp is advanced from these between CAN frames, never from time.time(): timed.py
  # steps the wall clock off of our own fix, so reading it back would be circular
  last_utc: datetime.datetime | None = None
  last_utc_monotonic: float = 0.0
  # frozen, not cleared, when 0x463 goes stale -- consumers keep getting the last value
  publish_millis: int = 0

  def observe_position(self, now: float) -> None:
    self.last_seen_pos = now
    if self.first_seen_pos == 0.0:
      self.first_seen_pos = now

  def observe_quality(self, now: float) -> None:
    self.last_seen_quality = now

  def observe_time(self, now: float, utc: datetime.datetime | None) -> None:
    """A 0x463 frame arrived; `utc` is decode_utc's verdict on it."""
    self.ever_saw_time = True
    if utc is not None:
      self.last_seen_time = now
      self.last_utc = utc
      self.last_utc_monotonic = now

  def reset(self) -> None:
    """Forget everything seen so far -- it came off the wrong bus (car changed)."""
    self.last_seen_pos = self.last_seen_time = self.last_seen_quality = self.first_seen_pos = 0.0
    self.ever_saw_time = False
    self.last_utc = None

  def time_source(self, now: float) -> TimeSource:
    if self.ever_saw_time:
      return TimeSource.CAN
    if self.first_seen_pos > 0.0 and (now - self.first_seen_pos) >= NO_TIME_GRACE:
      return TimeSource.NONE
    return TimeSource.UNKNOWN

  def pos_fresh(self, now: float) -> bool:
    return (now - self.last_seen_pos) < MAX_FIX_AGE

  def time_fresh(self, now: float) -> bool:
    return (now - self.last_seen_time) < MAX_FIX_AGE

  def quality_fresh(self, now: float) -> bool:
    return (now - self.last_seen_quality) < MAX_FIX_AGE

  def fresh_fix(self, now: float, fresh_time: bool, fresh_pos: bool) -> bool:
    """Did this cycle bring something worth publishing immediately?

    A 0x463 always is. On a position-only platform 0x462 is the only cadence there is,
    so publishing phase-locks to it instead of falling back to the bare keepalive.
    """
    return fresh_time or (fresh_pos and self.time_source(now) == TimeSource.NONE)

  def has_fix(self, now: float, position_valid: bool) -> bool:
    if self.time_source(now) == TimeSource.NONE:
      # position-only platform: a fix means a fresh, valid position and nothing more
      return position_valid and self.pos_fresh(now)
    return self.last_utc is not None and position_valid and self.time_fresh(now) and self.pos_fresh(now)

  def timestamp_millis(self, now: float, wall_millis: int) -> int:
    """The unixTimestampMillis to publish this cycle.

    `wall_millis` is only consulted on a position-only platform, where no GPS time exists
    and the field has no "unknown" value. There is no circularity in using the wall clock
    there the way there would be on a car with real GPS time: timed.py only ever steps the
    clock off a fix *we* published, and on such a platform we never publish a GPS-derived
    time, so there is nothing for it to have fed back to us. The caller passes 0 when the
    clock is still at the AGNOS flash date, which unconditionally fails timed.py's
    min_date() check rather than relying on the diff-vs-flash-date arithmetic happening to
    come out too small.
    """
    if self.time_fresh(now) and self.last_utc is not None:
      # hold the last decoded fix, advancing the clock by elapsed monotonic time
      self.publish_millis = int((self.last_utc.timestamp() + (now - self.last_utc_monotonic)) * 1000)
    elif self.time_source(now) == TimeSource.NONE:
      self.publish_millis = wall_millis
    # else: 0x463 has gone stale -- freeze, but keep publishing so SubMaster.alive holds
    return self.publish_millis


# car.CarParams is a capnp struct module rather than a type, so it cannot form a `| None`
# union; a deserialized CarParams is a _DynamicStructReader, the read side of the builder
# type build_gps_msg returns.
def load_car_params(params: Params, key: str, block: bool = False) -> capnp.lib.capnp._DynamicStructReader | None:
  """Read a CarParams param, or None if it is absent, corrupt, or not a car we can decode."""
  cp_bytes = params.get(key, block=block)
  if cp_bytes is None:
    return None
  try:
    CP = messaging.log_from_bytes(cp_bytes, car.CarParams)
  except Exception:
    cloudlog.exception(f"cangpsd: failed to deserialize {key}")
    return None
  if CP.carFingerprint not in DBC:
    cloudlog.warning(f"cangpsd: no DBC for {CP.carFingerprint} in {key}")
    return None
  return CP


def parser_config(CP: car.CarParams) -> tuple[str, int]:
  return DBC[CP.carFingerprint][Bus.pt], CanBus(CP).main


def make_parser(dbc_name: str, can_bus: int) -> CANParser:
  return CANParser(dbc_name, [(name, 1) for name, _ in GPS_MESSAGES], can_bus)


# Cached capnp schema field accessors, resolved on the first frame we see. Same trick as
# pandad's can_capnp_to_list -- looking fields up by name per frame is the slow path.
_can_fields: tuple | None = None


def decode_gps_frames(can_strs: list[bytes]) -> list[tuple[int, list[tuple]]]:
  """can_capnp_to_list, but only for the three addresses we parse.

  Worth diverging from the shared helper here because of the ratio involved: this car puts
  ~8100 CAN frames/s on the topic and the GPS messages are 3 of them. The shared helper
  materialises every frame -- three field reads, a bytes copy of dat (up to 64 B on CAN FD)
  and a tuple -- and CANParser, which is pure Python in this tree, then walks the whole list
  again to discard all but ours. Reading the address first and only building a tuple on a
  hit measured 4.34% -> 2.10% of a core on device, about 2 points off cangpsd's ~5.8%.

  The filter set comes from GPS_MESSAGES, the same declaration the parser is built from, so
  the two cannot drift apart.

  Safe for cangpsd specifically, and not in general: pre-filtering makes the parser see an
  empty bus on nearly every cycle, so last_nonempty_nanos and everything built on it
  (can_valid, bus_timeout) become meaningless. cangpsd never reads them -- it tracks
  staleness itself through FixTracker and MAX_FIX_AGE. A consumer that relies on the
  parser's own validity machinery must keep using the shared helper.

  Bus filtering is deliberately left to CANParser.update: an address on the wrong bus costs
  one extra tuple per cycle, and duplicating the bus check here is one more thing to keep in
  step with the parser.
  """
  global _can_fields
  result = []
  for s in can_strs:
    with log.Event.from_bytes(s, traversal_limit_in_words=CAPNP_NO_TRAVERSAL_LIMIT) as event:
      frames = event.can
      if _can_fields is None and len(frames) > 0:
        schema_fields = frames[0].schema.fields
        _can_fields = (schema_fields['address'], schema_fields['dat'], schema_fields['src'])

      kept: list[tuple] = []
      if _can_fields is not None:
        addr_f, dat_f, src_f = _can_fields
        for f in frames:
          address = f._get_by_field(addr_f)
          if address in GPS_ADDRS:
            kept.append((address, f._get_by_field(dat_f), f._get_by_field(src_f)))
      result.append((event.logMonoTime, kept))
  return result


def make_pub_master(service: str = GPS_SERVICE_DEFAULT, attempts: int = 5, delay: float = 1.0) -> messaging.PubMaster:
  """Open the GPS pub socket, retrying with backoff.

  When the selected source changes while onroad, manager's ensure_running stops the
  outgoing publisher in the same pass that starts us, and msgq only allows one publisher
  per topic -- so the socket can briefly still be held by that process. Retry instead of
  crash-looping through manager.
  """
  last_exc: Exception | None = None
  for attempt in range(attempts):
    try:
      return messaging.PubMaster([service])
    except Exception as e:
      last_exc = e
      cloudlog.warning(f"cangpsd: failed to open {service} pub socket (attempt {attempt + 1}/{attempts}): {e}")
      time.sleep(delay)
  assert last_exc is not None
  raise last_exc


# Cap the per-cycle timestep fed to the arbiter. A scheduling hiccup or a debugger pause
# should not count as driving time toward a threshold measured in minutes.
MAX_ARBITER_DT = 0.5


def select_mode(params: Params, CP) -> tuple[bool, FallbackArbiter | None]:
  """Decide whether to publish this run, and set up the arbiter if it is ours to decide.

  The manual toggle wins outright and skips measurement entirely: a user who has ticked it
  has already diagnosed the problem the arbiter exists to discover, and second-guessing
  them by handing the topic back after twenty quiet minutes would be obnoxious.
  """
  if params.get_bool("FordPrefUseVehicleGps"):
    cloudlog.info("cangpsd: publishing (FordPrefUseVehicleGps is set)")
    return True, None

  state = load_state(params, vehicle_key(CP))
  arbiter = FallbackArbiter(state)
  publishing = state.source == SOURCE_CAN
  mode = 'publishing' if publishing else 'observing'
  cloudlog.info(f"cangpsd: {mode}, fallback state {json_state(state)}")
  return publishing, arbiter


def main() -> NoReturn:
  params = Params()

  # Start decoding from the previous drive's CarParams rather than blocking on the live one.
  # card only publishes CarParams once it has fingerprinted the car, about six seconds into a
  # boot, and until something publishes a fix timed.py cannot set the wall clock -- which is
  # why cold-boot routes open stamped with the AGNOS flash date. The car is already sending
  # 0x462/0x463 by then, so all that waiting buys us is a later clock. The live CP is
  # reconciled in the loop below and the parser rebuilt if the car actually changed.
  CP = load_car_params(params, "CarParams")
  live_cp_pending = CP is None
  if CP is None:
    CP = load_car_params(params, "CarParamsPersistent")
  if CP is not None:
    cloudlog.info(f"cangpsd starting from {'live' if not live_cp_pending else 'previous route'} CarParams ({CP.carFingerprint})")
  else:
    cloudlog.info("cangpsd is waiting for CarParams")
    CP = load_car_params(params, "CarParams", block=True)
    if CP is None:
      raise RuntimeError("cangpsd: CarParams names no car we have a DBC for")
    cloudlog.info("cangpsd got CarParams")
    live_cp_pending = False

  dbc_name, can_bus = parser_config(CP)
  cp = make_parser(dbc_name, can_bus)

  can_sock = messaging.sub_sock('can', timeout=20)

  publishing, arbiter = select_mode(params, CP)
  # Resolve the topic the same way every consumer does, so it cannot drift from them:
  # gpsLocationExternal where the device has a working ublox, gpsLocation otherwise. This is
  # also the topic the device's own GPS daemon owns, which is what makes it the one to
  # observe when we are not publishing.
  gps_service = get_gps_location_service(params)
  cloudlog.info(f"cangpsd {'publishing on' if publishing else 'observing'} {gps_service}")
  # Only open the pub socket when we are the selected source. In observer mode the device
  # daemon holds it, and taking it would be the collision this arrangement exists to avoid.
  pm = make_pub_master(gps_service) if publishing else None
  # carState for "is the car actually moving" (parked with no fix is not evidence), and the
  # GPS topic to see whether the source that currently owns it is working. In publishing
  # mode that source is us, so we read our own has_fix directly instead.
  observed = ['carState'] if publishing else ['carState', gps_service]
  sm = messaging.SubMaster(observed) if arbiter is not None else None

  now = time.monotonic()
  fix = FixTracker()
  had_fix = False
  ever_had_fix = False
  next_cp_check = now
  last_arbiter_now = now

  rk = Ratekeeper(10, print_delay_threshold=None)
  last_publish = now - KEEPALIVE_INTERVAL
  while True:
    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    now = time.monotonic()

    # Reconcile the early start against the live CarParams once card publishes it. All Ford
    # fingerprints share ford_lincoln_base_pt, so only the bus offset can realistically move
    # -- but decoding the right addresses off the wrong bus would fail silently, so rebuild
    # rather than assume. A brand change is manager's problem: the gate drops us instead.
    if live_cp_pending and now >= next_cp_check:
      next_cp_check = now + CP_CHECK_INTERVAL
      live_CP = load_car_params(params, "CarParams")
      if live_CP is not None:
        live_cp_pending = False
        # We chose publish-or-observe from the previous route's CarParams, and so did
        # manager's ubloxd gate. If the device has moved to a different car since, the two
        # will now disagree the moment manager re-reads the live CP -- it would start
        # ubloxd while we are still publishing, putting two publishers on the topic. Bail
        # out and let manager restart us against the right vehicle's decision.
        live_key = vehicle_key(live_CP)
        if arbiter is not None and live_key and live_key != arbiter.state.vin:
          cloudlog.warning("cangpsd: different vehicle than last route, restarting to re-decide")
          sys.exit(0)
        if parser_config(live_CP) != (dbc_name, can_bus):
          dbc_name, can_bus = parser_config(live_CP)
          cloudlog.warning(f"cangpsd: car changed since last route, re-parsing {live_CP.carFingerprint} on bus {can_bus}")
          cp = make_parser(dbc_name, can_bus)
          fix.reset()

    fresh_pos = fresh_time = False
    if can_strs:
      updated = cp.update(decode_gps_frames(can_strs))
      if GPS_ADDR_POS in updated:
        fresh_pos = True
        fix.observe_position(now)
      if GPS_ADDR_QUALITY in updated:
        fix.observe_quality(now)
      if GPS_ADDR_TIME in updated:
        fresh_time = True
        fix.observe_time(now, decode_utc(cp.vl[GPS_ADDR_TIME]))

    if should_publish(fix.fresh_fix(now, fresh_time, fresh_pos), now, last_publish):
      last_publish = now
      position = decode_position(cp.vl[GPS_ADDR_POS])
      has_fix = fix.has_fix(now, position is not None)
      wall_millis = int(time.time() * 1000) if system_time_valid() else 0  # noqa: TID251
      publish_millis = fix.timestamp_millis(now, wall_millis)

      if has_fix and not had_fix:
        cloudlog.warning("cangpsd: GPS fix reacquired" if ever_had_fix else "cangpsd: first valid GPS fix acquired")
      elif had_fix and not has_fix:
        cloudlog.warning("cangpsd: GPS fix lost")
      had_fix = has_fix
      ever_had_fix = ever_had_fix or has_fix

      # 0x463's dead-reckoning flag. Needed in *both* modes, which is why it is computed
      # here rather than in the publish branch below: publishing uses it to widen the
      # accuracies, and observing uses it to tell the arbiter whether the car's fix is real.
      # The flag only means anything while 0x463 is fresh. A position-only platform never
      # has one, so those cars keep their current accuracy; on a 0x463 platform a stale time
      # frame has already cleared has_fix via MAX_FIX_AGE.
      inferred = fix.time_fresh(now) and decode_inferred(cp.vl[GPS_ADDR_TIME])

      # Stay silent until the first real decode. The keepalive above exists to hold
      # SubMaster.alive true for consumers once we are a working GPS; before that there is
      # nothing to hold up, and a zeroed sample (lat/lon 0, publish_millis 0 -> 1970) is
      # indistinguishable on the wire from a receiver that is present and failing.
      if pm is not None and ever_had_fix:
        lat, lon = (position[0], position[1]) if position is not None else (0.0, 0.0)
        # Covers both "this car has no 0x464" (Focus MK4) and "0x464 has gone quiet":
        # stale altitude/speed/bearing are worse than admitting we do not know.
        quality = decode_quality(cp.vl[GPS_ADDR_QUALITY]) if fix.quality_fresh(now) else QUALITY_UNKNOWN
        msg = build_gps_msg(lat, lon, quality["altitude"], quality["speed"], quality["bearing_deg"],
                             publish_millis, quality["hdop"], quality["vdop"], quality["sat_count"], has_fix,
                             inferred, service=gps_service)
        pm.send(gps_service, msg)

      if arbiter is not None:
        # The publish cadence is the arbiter's tick. KEEPALIVE_INTERVAL makes this branch
        # run at least every 0.5 s even with no CAN traffic at all, which is exactly the
        # case that has to keep accumulating.
        dt = min(now - last_arbiter_now, MAX_ARBITER_DT)
        last_arbiter_now = now
        sm.update(0)
        moving = sm.alive['carState'] and sm['carState'].vEgo > MIN_SPEED
        # Who currently owns the topic decides where "is it working" comes from. As the
        # publisher that is our own fix; as an observer it is whatever the device's GPS
        # daemon is sending, and a topic that has gone silent entirely counts as no fix.
        published_fix = has_fix if publishing else (sm.alive[gps_service] and
                                                    sm[gps_service].hasFix)
        # Dead-reckoned fixes are excluded deliberately: the APIM asserts a fix while
        # inferring, so passing raw has_fix would let a tunnel -- the one thing likely to
        # run the device receiver out to NO_FIX_TIMEOUT -- also satisfy the guard meant to
        # catch it. See CAN_FIX_MIN_S.
        if arbiter.update(dt, moving, published_fix, has_fix and not inferred):
          save_state(params, arbiter.state)
          # Exit rather than swap the pub socket in place. manager re-reads the ubloxd gate
          # against the state just written and starts us again a moment later; going out
          # through process death is the only way to be sure the outgoing publisher has
          # released the GPS topic before the incoming one asks for it.
          cloudlog.warning("cangpsd: GPS source changed, restarting to hand over the topic")
          sys.exit(0)
        elif arbiter.take_dirty():
          save_state(params, arbiter.state)

    rk.keep_time()


if __name__ == "__main__":
  main()
