#!/usr/bin/env python3
"""Publish gpsLocationExternal from the Ford CAN GPS solution.

The comma device's ublox GPS can never get a fix in this Mach-E: the
IR-reflective windshield blocks the antenna and there's no uncoated zone to
mount in. The car itself broadcasts a full GPS solution on CAN bus 0 at
~1 Hz (APIMGPS_Data_Nav_1/2/3_FD1, DBC ford_lincoln_base_pt.dbc). This daemon
decodes it and republishes gpsLocationExternal so system/timed.py,
locationd_llk, speed_limit_resolver.py, mapd, and athenad all keep working
unmodified. ubloxd is gated off elsewhere so msgq's one-publisher-per-topic
rule doesn't collide.

Decode is split into pure functions (decode_utc, decode_position,
decode_quality) that take plain dicts of already-scaled DBC signal values, so
they can be unit-tested and replayed offline against a recorded rlog without
any cereal sockets -- see comma-gps-time/scripts/decode_can_gps.py.
"""
import datetime
import math
import time
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
  """
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


def decode_quality(vl: dict) -> dict:
  """Decode the secondary fix-quality fields out of 0x464.

  Not part of the hasFix gate (only 0x462/0x463 freshness and validity are),
  but used to fill in the rest of the published message.
  """
  sat_count = int(vl["GPS_Sat_num_in_view"])
  return {
    # GPS_MSL_altitude is geoid height (mean sea level); GpsLocationData.altitude
    # is documented as WGS84 ellipsoid height. The two differ by ~35 m in Seattle.
    # Accepted uncorrected -- we have no geoid model on the device.
    "altitude": vl["GPS_MSL_altitude"] * 0.3048,
    "speed": vl["GPS_Speed"] * 0.44704,
    "bearing_deg": vl["GPS_Heading"],
    "hdop": vl["GPS_Hdop"],
    "vdop": vl["GPS_Vdop"],
    # GPS_Sat_num_in_view is a 5-bit field the DBC bounds at [0|29], so 30/31 are
    # sentinels. Verified against routes 0x101, 0x106 and 0xf1: this car reports a
    # constant 31 even while stationary with PDOP 0.6, i.e. it never populates the
    # count at all. Report 0 (unknown) rather than a fabricated 31 satellites.
    "sat_count": sat_count if sat_count <= 29 else 0,
  }


def build_gps_msg(lat: float, lon: float, altitude: float, speed: float, bearing_deg: float,
                   unix_timestamp_millis: int, hdop: float, vdop: float, sat_count: int,
                   has_fix: bool) -> capnp.lib.capnp._DynamicStructBuilder:
  """Build a gpsLocationExternal message from already-decoded plain values."""
  msg = messaging.new_message('gpsLocationExternal', valid=True)
  gps = msg.gpsLocationExternal
  gps.latitude = lat
  gps.longitude = lon
  gps.altitude = altitude
  gps.speed = speed
  gps.bearingDeg = bearing_deg
  gps.unixTimestampMillis = unix_timestamp_millis
  gps.source = log.GpsLocationData.SensorSource.car
  bearing_rad = math.radians(bearing_deg)
  gps.vNED = [speed * math.cos(bearing_rad), speed * math.sin(bearing_rad), 0.0]
  # locationd.cc discards the message outright unless all three accuracies are
  # positive, so these are clamped to a nominal floor rather than left at raw
  # (possibly zero, e.g. before the first 0x464 frame) hdop/vdop.
  gps.horizontalAccuracy = max(1.0, hdop * 5.0)  # 5 m nominal UERE per dop unit
  gps.verticalAccuracy = max(1.0, vdop * 5.0)
  gps.speedAccuracy = 0.5
  gps.bearingAccuracyDeg = 5.0
  gps.hasFix = has_fix
  gps.satelliteCount = sat_count
  return msg


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
  cloudlog.info("cangpsd is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("cangpsd got CarParams")

  dbc_name = DBC[CP.carFingerprint][Bus.pt]
  can_bus = CanBus(CP).main
  cp = CANParser(dbc_name, [
    ("APIMGPS_Data_Nav_1_FD1", 1),
    ("APIMGPS_Data_Nav_2_FD1", 1),
    ("APIMGPS_Data_Nav_3_FD1", 1),
  ], can_bus)

  can_sock = messaging.sub_sock('can', timeout=20)
  pm = make_pub_master()

  now = time.monotonic()
  last_seen_pos = 0.0
  last_seen_time = 0.0
  # timestamp of the last decoded 0x463 sample and the monotonic clock reading taken
  # at that moment -- unixTimestampMillis is advanced from these between CAN frames
  # using time.monotonic(), never time.time(): timed.py steps the wall clock off of
  # our own published fix, so reading time.time() back here would be circular.
  last_utc: datetime.datetime | None = None
  last_utc_monotonic = now
  publish_millis = 0
  had_fix = False

  rk = Ratekeeper(10, print_delay_threshold=None)
  last_publish = now - KEEPALIVE_INTERVAL
  while True:
    fresh_fix = False
    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    now = time.monotonic()
    if can_strs:
      updated = cp.update(can_capnp_to_list(can_strs))
      if GPS_ADDR_POS in updated:
        last_seen_pos = now
      if GPS_ADDR_TIME in updated:
        last_seen_time = now
        fresh_fix = True
        utc = decode_utc(cp.vl[GPS_ADDR_TIME])
        if utc is not None:
          last_utc = utc
          last_utc_monotonic = now

    if should_publish(fresh_fix, now, last_publish):
      last_publish = now
      time_fresh = (now - last_seen_time) < MAX_FIX_AGE
      pos_fresh = (now - last_seen_pos) < MAX_FIX_AGE
      position = decode_position(cp.vl[GPS_ADDR_POS])
      has_fix = last_utc is not None and position is not None and time_fresh and pos_fresh

      if time_fresh and last_utc is not None:
        # hold the last decoded fix, advancing the clock by elapsed monotonic time
        publish_millis = int((last_utc.timestamp() + (now - last_utc_monotonic)) * 1000)
      # else: 0x463 has gone stale -- freeze publish_millis and report no fix,
      # but keep publishing so SubMaster.alive stays true for consumers

      if has_fix and not had_fix:
        cloudlog.warning("cangpsd: first valid GPS fix acquired")
      elif had_fix and not has_fix:
        cloudlog.warning("cangpsd: GPS fix lost")
      had_fix = has_fix

      lat, lon = (position[0], position[1]) if position is not None else (0.0, 0.0)
      quality = decode_quality(cp.vl[GPS_ADDR_QUALITY])
      msg = build_gps_msg(lat, lon, quality["altitude"], quality["speed"], quality["bearing_deg"],
                           publish_millis, quality["hdop"], quality["vdop"], quality["sat_count"], has_fix)
      pm.send('gpsLocationExternal', msg)

    rk.keep_time()


if __name__ == "__main__":
  main()
