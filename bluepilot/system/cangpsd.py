#!/usr/bin/env python3
"""Publish gpsLocationExternal from the Ford CAN GPS solution.

The comma device's ublox GPS can never get a fix in this Mach-E: the
IR-reflective windshield blocks the antenna and there's no uncoated zone to
mount in. The car itself broadcasts a full GPS solution on CAN bus 0 at
~1 Hz (APIMGPS_Data_Nav_1/2/3_FD1, DBC ford_lincoln_base_pt.dbc). This daemon
decodes it and republishes gpsLocationExternal so system/timed.py,
selfdrived.py, speed_limit_resolver.py, mapd, and athenad all keep working
unmodified (locationd in this tree is camera+IMU only and never reads GPS).
ubloxd is gated off elsewhere so msgq's one-publisher-per-topic rule doesn't
collide.

Decode is split into pure functions (decode_utc, decode_position,
decode_quality) that take plain dicts of already-scaled DBC signal values, so
they can be unit-tested and replayed offline against a recorded rlog without
any cereal sockets -- see comma-gps-time/scripts/decode_can_gps.py.
"""
import datetime
import enum
import math
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

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.selfdrive.pandad import can_capnp_to_list

# APIMGPS_Data_Nav_1_FD1: lat/lon degrees+minutes and hemisphere enums
GPS_ADDR_POS = 0x462
# APIMGPS_Data_Nav_2_FD1: UTC date/time, PDOP, fault flag
GPS_ADDR_TIME = 0x463
# APIMGPS_Data_Nav_3_FD1: altitude, speed, heading, sat count, hdop/vdop
GPS_ADDR_QUALITY = 0x464

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
                   has_fix: bool) -> capnp.lib.capnp._DynamicStructBuilder:
  """Build a gpsLocationExternal message from already-decoded plain values.

  A None means the car never told us. GpsLocationData has no way to say "unknown" for a
  value, only an accuracy alongside it, so the value goes out as 0 and the matching
  accuracy is widened to cover the whole plausible range. That is the difference between
  publishing 0 m/s because the car is stopped and publishing 0 m/s because we have no idea
  -- a consumer that reads accuracy can tell them apart, and one that ignores accuracy is
  no worse off than it would be with a fabricated number.
  """
  msg = messaging.new_message('gpsLocationExternal', valid=True)
  gps = msg.gpsLocationExternal
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
  gps.horizontalAccuracy = max(1.0, hdop * 5.0) if hdop is not None else UNKNOWN_ACCURACY
  # verticalAccuracy has to widen on an unknown altitude too, not just an unknown vdop --
  # otherwise a missing GPS_MSL_altitude (published as 0.0 above) pairs with a valid vdop
  # to publish a confident sea-level reading. Speed/bearing below already couple this way.
  gps.verticalAccuracy = max(1.0, vdop * 5.0) if (vdop is not None and altitude is not None) else UNKNOWN_ACCURACY
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
  return CANParser(dbc_name, [
    ("APIMGPS_Data_Nav_1_FD1", 1),
    ("APIMGPS_Data_Nav_2_FD1", 1),
    ("APIMGPS_Data_Nav_3_FD1", 1),
  ], can_bus)


def make_pub_master(attempts: int = 5, delay: float = 1.0) -> messaging.PubMaster:
  """Open the gpsLocationExternal pub socket, retrying with backoff.

  If UbloxAvailable was just flipped off while onroad, manager's
  ensure_running stops ubloxd in the same pass that starts us, and msgq only
  allows one publisher per topic -- so the socket can briefly still be held
  by the outgoing process. Retry instead of crash-looping through manager.
  """
  last_exc: Exception | None = None
  for attempt in range(attempts):
    try:
      return messaging.PubMaster(['gpsLocationExternal'])
    except Exception as e:
      last_exc = e
      cloudlog.warning(f"cangpsd: failed to open gpsLocationExternal pub socket (attempt {attempt + 1}/{attempts}): {e}")
      time.sleep(delay)
  assert last_exc is not None
  raise last_exc


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
  pm = make_pub_master()

  now = time.monotonic()
  fix = FixTracker()
  had_fix = False
  ever_had_fix = False
  next_cp_check = now

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
        if parser_config(live_CP) != (dbc_name, can_bus):
          dbc_name, can_bus = parser_config(live_CP)
          cloudlog.warning(f"cangpsd: car changed since last route, re-parsing {live_CP.carFingerprint} on bus {can_bus}")
          cp = make_parser(dbc_name, can_bus)
          fix.reset()

    fresh_pos = fresh_time = False
    if can_strs:
      updated = cp.update(can_capnp_to_list(can_strs))
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
        cloudlog.warning("cangpsd: first valid GPS fix acquired")
      elif had_fix and not has_fix:
        cloudlog.warning("cangpsd: GPS fix lost")
      had_fix = has_fix
      ever_had_fix = ever_had_fix or has_fix

      # Stay silent until the first real decode. The keepalive above exists to hold
      # SubMaster.alive true for consumers once we are a working GPS; before that there is
      # nothing to hold up, and a zeroed sample (lat/lon 0, publish_millis 0 -> 1970) is
      # indistinguishable on the wire from a receiver that is present and failing.
      if ever_had_fix:
        lat, lon = (position[0], position[1]) if position is not None else (0.0, 0.0)
        # Covers both "this car has no 0x464" (Focus MK4) and "0x464 has gone quiet":
        # stale altitude/speed/bearing are worse than admitting we do not know.
        quality = decode_quality(cp.vl[GPS_ADDR_QUALITY]) if fix.quality_fresh(now) else QUALITY_UNKNOWN
        msg = build_gps_msg(lat, lon, quality["altitude"], quality["speed"], quality["bearing_deg"],
                             publish_millis, quality["hdop"], quality["vdop"], quality["sat_count"], has_fix)
        pm.send('gpsLocationExternal', msg)

    rk.keep_time()


if __name__ == "__main__":
  main()
