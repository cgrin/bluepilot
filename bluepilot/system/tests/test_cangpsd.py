"""Unit tests for the Ford CAN GPS decode.

cangpsd's decode is deliberately split into pure functions over plain dicts of already
scaled DBC signal values, so everything below runs without a panda, a car, or a msgq
socket. Values are written as the scaled floats CANParser hands back, not raw bits -- a
raw 31 in GPS_Hdop arrives here as 6.2, and getting that mapping wrong is exactly the
class of bug these tests exist to catch.

Platform coverage for which of these messages each Ford actually sends is measured in
bluepilot/system/CANGPS_PLATFORMS.md.
"""
import datetime

import pytest

from openpilot.bluepilot.system.cangpsd import (
  KEEPALIVE_INTERVAL,
  QUALITY_UNKNOWN,
  UNKNOWN_ACCURACY,
  UNKNOWN_BEARING_ACCURACY,
  UNKNOWN_SPEED_ACCURACY,
  build_gps_msg,
  decode_position,
  decode_quality,
  decode_utc,
  should_publish,
)

# A good 0x463 sample: 2026-08-27 00:33:45 UTC, PDOP 1.0, no fault.
GOOD_TIME = {
  "Gps_B_Falt": 0, "GPS_Pdop": 1.0, "GpsUtcYr_No_Actl": 2026, "GpsUtcMnth_No_Actl": 8,
  "GpsUtcDay_No_Actl": 27, "GPS_UTC_hours": 0, "GPS_UTC_minutes": 33, "GPS_UTC_seconds": 45,
}

# A good 0x462 sample: Seattle, 47.629057 N 122.342 W. Degrees are already signed by the
# DBC offsets (-89 lat, -179 lon), which is why the hemisphere enums go unused.
GOOD_POS = {
  "GPS_Latitude_Degrees": 47, "GPS_Latitude_Minutes": 37, "GPS_Latitude_Min_dec": 0.7434,
  "GPS_Longitude_Degrees": -122, "GPS_Longitude_Minutes": 20, "GPS_Longitude_Min_dec": 0.52,
  "GpsHsphLattSth_D_Actl": 2, "GpsHsphLongEast_D_Actl": 2,
}

# A good 0x464 sample. Scaled units: altitude feet, speed MPH, heading degrees.
GOOD_QUALITY = {
  "GPS_MSL_altitude": 100.0, "GPS_Speed": 40.0, "GPS_Heading": 267.76,
  "GPS_Hdop": 0.6, "GPS_Vdop": 0.8, "GPS_Sat_num_in_view": 12,
}


def with_(base: dict, **over) -> dict:
  return {**base, **over}


class TestDecodeUtc:
  def test_good_sample(self):
    assert decode_utc(GOOD_TIME) == datetime.datetime(2026, 8, 27, 0, 33, 45, tzinfo=datetime.UTC)

  def test_fault_flag_rejects(self):
    assert decode_utc(with_(GOOD_TIME, Gps_B_Falt=1)) is None

  @pytest.mark.parametrize("pdop", [0.0, 5.2, 6.0, 6.2])
  def test_pdop_out_of_band_rejects(self, pdop):
    # 0 is "no solution"; 6.0/6.2 are raw 30/31, the Unknown/Invalid sentinels; 5.2 is a
    # real but too-weak geometry. The 5.0 ceiling is cangpsd's own quality bar.
    assert decode_utc(with_(GOOD_TIME, GPS_Pdop=pdop)) is None

  def test_pdop_at_limit_accepted(self):
    assert decode_utc(with_(GOOD_TIME, GPS_Pdop=5.0)) is not None

  @pytest.mark.parametrize("field,value", [
    ("GpsUtcYr_No_Actl", 2041),    # raw 31 Fault -> 2010 + 31
    ("GpsUtcMnth_No_Actl", 16),    # raw 15 Fault -> 1 + 15
    ("GPS_UTC_hours", 30),         # Unknown
    ("GPS_UTC_hours", 31),         # Invalid
    ("GPS_UTC_minutes", 62),       # Unknown
    ("GPS_UTC_minutes", 63),       # Fault
    ("GPS_UTC_seconds", 62),       # Unknown
    ("GPS_UTC_seconds", 63),       # Fault
  ])
  def test_sentinels_rejected_by_range_check(self, field, value):
    # No sentinel is special-cased; each one lands outside the legal calendar range.
    assert decode_utc(with_(GOOD_TIME, **{field: value})) is None

  def test_year_below_floor_rejected(self):
    assert decode_utc(with_(GOOD_TIME, GpsUtcYr_No_Actl=2023)) is None

  def test_calendar_invalid_date_rejected(self):
    # In range field by field, impossible as a date -- the ValueError path.
    assert decode_utc(with_(GOOD_TIME, GpsUtcMnth_No_Actl=2, GpsUtcDay_No_Actl=30)) is None

  def test_result_is_utc_aware(self):
    # timed.py compares this against the wall clock; a naive datetime would be read as
    # local time and silently shift the clock by the timezone offset.
    assert decode_utc(GOOD_TIME).tzinfo is datetime.UTC


class TestDecodePosition:
  def test_degrees_plus_decimal_minutes(self):
    lat, lon, _, _ = decode_position(GOOD_POS)
    assert lat == pytest.approx(47 + (37 + 0.7434) / 60.0, abs=1e-9)
    assert lon == pytest.approx(-(122 + (20 + 0.52) / 60.0), abs=1e-9)

  def test_west_longitude_is_negative(self):
    _, lon, _, _ = decode_position(GOOD_POS)
    assert lon < 0

  def test_southern_latitude_keeps_minutes_positive(self):
    # Sign lives on the degrees field; minutes are always a positive magnitude, so a
    # naive deg + min/60 would pull a southern latitude toward the equator.
    lat, _, _, _ = decode_position(with_(GOOD_POS, GPS_Latitude_Degrees=-33,
                                         GPS_Latitude_Minutes=52, GPS_Latitude_Min_dec=0.0))
    assert lat == pytest.approx(-(33 + 52 / 60.0), abs=1e-9)

  def test_eastern_longitude(self):
    _, lon, _, _ = decode_position(with_(GOOD_POS, GPS_Longitude_Degrees=120,
                                          GPS_Longitude_Minutes=40, GPS_Longitude_Min_dec=0.0))
    assert lon == pytest.approx(120 + 40 / 60.0, abs=1e-9)

  def test_null_island_is_a_sentinel_not_a_position(self):
    assert decode_position(with_(GOOD_POS, GPS_Latitude_Degrees=0, GPS_Latitude_Minutes=0,
                                 GPS_Latitude_Min_dec=0.0, GPS_Longitude_Degrees=0,
                                 GPS_Longitude_Minutes=0, GPS_Longitude_Min_dec=0.0)) is None

  @pytest.mark.parametrize("over", [
    {"GPS_Latitude_Degrees": 95},
    {"GPS_Longitude_Degrees": 200},
  ])
  def test_out_of_range_rejected(self, over):
    assert decode_position(with_(GOOD_POS, **over)) is None

  @pytest.mark.parametrize("field,value", [
    ("GPS_Latitude_Minutes", 62),      # Unknown
    ("GPS_Latitude_Minutes", 63),      # Fault
    ("GPS_Longitude_Minutes", 62),     # Unknown
    ("GPS_Longitude_Minutes", 63),     # Fault
    ("GPS_Latitude_Min_dec", 1.6382),  # raw 16382, Unknown
    ("GPS_Latitude_Min_dec", 1.6383),  # raw 16383, Invalid
    ("GPS_Longitude_Min_dec", 1.6382),  # raw 16382, Unknown
    ("GPS_Longitude_Min_dec", 1.6383),  # raw 16383, Invalid
  ])
  def test_minutes_sentinel_rejected(self, field, value):
    # Unlike the degrees sentinels, these land inside a plausible minutes range once
    # scaled -- e.g. GPS_Latitude_Minutes 63 reads as a legal 63 minutes, not caught by
    # the |lat| > 90 check. Worth up to ~120 km of silent error if not rejected explicitly.
    assert decode_position(with_(GOOD_POS, **{field: value})) is None

  def test_hemisphere_enums_passed_through_unused(self):
    # Returned so an offline decoder can surface them if another vehicle disagrees, but
    # they must not influence the sign -- this car reports 2/2 (north+west) always.
    lat_a, lon_a, h_lat, h_lon = decode_position(GOOD_POS)
    lat_b, lon_b, _, _ = decode_position(with_(GOOD_POS, GpsHsphLattSth_D_Actl=1,
                                                GpsHsphLongEast_D_Actl=1))
    assert (h_lat, h_lon) == (2, 2)
    assert (lat_a, lon_a) == (lat_b, lon_b)


class TestDecodeQuality:
  def test_unit_conversions(self):
    q = decode_quality(GOOD_QUALITY)
    assert q["altitude"] == pytest.approx(100.0 * 0.3048, abs=1e-9)   # feet -> m
    assert q["speed"] == pytest.approx(40.0 * 0.44704, abs=1e-9)      # MPH -> m/s
    assert q["bearing_deg"] == pytest.approx(267.76)
    assert q["sat_count"] == 12

  @pytest.mark.parametrize("field,value,raw", [
    ("GPS_MSL_altitude", 20480.0, 4094),   # Unknown: raw * 10 - 20460
    ("GPS_MSL_altitude", 20490.0, 4095),   # Fault
    ("GPS_Speed", 254.0, 254),             # Unknown
    ("GPS_Speed", 255.0, 255),             # Invalid
    ("GPS_Heading", 655.34, 65534),        # Unknown: raw * 0.01
    ("GPS_Heading", 655.35, 65535),        # Fault
    ("GPS_Hdop", 6.0, 30),                 # Unknown: raw * 0.2
    ("GPS_Hdop", 6.2, 31),                 # Invalid
    ("GPS_Vdop", 6.0, 30),
    ("GPS_Vdop", 6.2, 31),
  ])
  def test_sentinel_becomes_none(self, field, value, raw):
    # Passed through the scale factor a sentinel looks like a real reading: GPS_Speed 255
    # would publish as 114 m/s.
    key = {"GPS_MSL_altitude": "altitude", "GPS_Speed": "speed", "GPS_Heading": "bearing_deg",
           "GPS_Hdop": "hdop", "GPS_Vdop": "vdop"}[field]
    assert decode_quality(with_(GOOD_QUALITY, **{field: value}))[key] is None

  @pytest.mark.parametrize("raw", [30, 31])
  def test_satellite_sentinel_reports_zero_not_a_count(self, raw):
    # Intermittent on CAN FD platforms; 0 means unknown rather than 31 satellites.
    assert decode_quality(with_(GOOD_QUALITY, GPS_Sat_num_in_view=raw))["sat_count"] == 0

  def test_satellite_count_at_upper_bound_kept(self):
    assert decode_quality(with_(GOOD_QUALITY, GPS_Sat_num_in_view=29))["sat_count"] == 29

  def test_quality_unknown_matches_shape(self):
    # QUALITY_UNKNOWN stands in when 0x464 is absent, so it has to be substitutable.
    assert QUALITY_UNKNOWN.keys() == decode_quality(GOOD_QUALITY).keys()


class TestBuildGpsMsg:
  def build(self, **over):
    args = dict(lat=47.6, lon=-122.3, altitude=30.0, speed=18.0, bearing_deg=90.0,
                unix_timestamp_millis=1787790825000, hdop=0.6, vdop=0.8, sat_count=12,
                has_fix=True)
    args.update(over)
    return build_gps_msg(**args).gpsLocationExternal

  def test_source_is_car(self):
    assert str(self.build().source) == "car"

  def test_accuracies_scale_from_dop(self):
    gps = self.build()
    assert gps.horizontalAccuracy == pytest.approx(0.6 * 5.0)
    assert gps.verticalAccuracy == pytest.approx(0.8 * 5.0)

  def test_accuracy_floor_keeps_locationd_from_discarding(self):
    # locationd.cc drops the message unless all three accuracies are positive.
    gps = self.build(hdop=0.0, vdop=0.0)
    assert gps.horizontalAccuracy > 0 and gps.verticalAccuracy > 0 and gps.speedAccuracy > 0

  def test_vned_from_speed_and_bearing(self):
    gps = self.build(speed=10.0, bearing_deg=0.0)
    assert gps.vNED[0] == pytest.approx(10.0)   # due north
    assert gps.vNED[1] == pytest.approx(0.0, abs=1e-9)

  def test_unknown_dop_widens_accuracy_instead_of_claiming_one_metre(self):
    # The bug this guards: max(1.0, None-as-0 * 5.0) published 1 m accuracy for a car
    # that never sent 0x464 at all.
    gps = self.build(hdop=None, vdop=None)
    assert gps.horizontalAccuracy == pytest.approx(UNKNOWN_ACCURACY)
    assert gps.verticalAccuracy == pytest.approx(UNKNOWN_ACCURACY)

  def test_unknown_speed_and_bearing_saturate_their_accuracies(self):
    gps = self.build(speed=None, bearing_deg=None)
    assert gps.speed == 0.0 and gps.bearingDeg == 0.0
    assert gps.speedAccuracy == pytest.approx(UNKNOWN_SPEED_ACCURACY)
    assert gps.bearingAccuracyDeg == pytest.approx(UNKNOWN_BEARING_ACCURACY)

  def test_unknown_accuracy_exceeds_anything_the_dbc_can_express(self):
    # 5.8 is the DBC's hdop ceiling, so a real reading can never reach UNKNOWN_ACCURACY
    # and a consumer can tell "bad geometry" from "no data".
    assert UNKNOWN_ACCURACY > 5.8 * 5.0

  def test_unknown_altitude_is_zero(self):
    assert self.build(altitude=None).altitude == 0.0

  def test_unknown_altitude_widens_vertical_accuracy(self):
    # The bug this guards: a missing GPS_MSL_altitude published as altitude=0.0 above, but
    # a valid vdop still scaled verticalAccuracy to a tight value -- a confident sea-level
    # reading manufactured out of "we don't know". altitude=None must widen it regardless
    # of vdop, the same way speed=None/bearing_deg=None widen their own accuracies.
    gps = self.build(altitude=None, vdop=0.8)
    assert gps.verticalAccuracy == pytest.approx(UNKNOWN_ACCURACY)

  def test_whole_unknown_quality_dict_builds(self):
    gps = build_gps_msg(47.6, -122.3, **{k: QUALITY_UNKNOWN[k] for k in
                                          ("altitude", "speed", "bearing_deg")},
                        unix_timestamp_millis=0, hdop=QUALITY_UNKNOWN["hdop"],
                        vdop=QUALITY_UNKNOWN["vdop"], sat_count=QUALITY_UNKNOWN["sat_count"],
                        has_fix=True)
    assert gps.gpsLocationExternal.horizontalAccuracy == pytest.approx(UNKNOWN_ACCURACY)


class TestShouldPublish:
  def test_fresh_fix_publishes_immediately(self):
    # Phase-lock: a new 0x463 goes out on arrival rather than waiting for the next tick.
    assert should_publish(True, now=100.0, last_publish=99.99)

  def test_keepalive_fires_when_topic_would_go_stale(self):
    assert should_publish(False, now=100.0, last_publish=100.0 - KEEPALIVE_INTERVAL)

  def test_quiet_between_keepalives(self):
    assert not should_publish(False, now=100.0, last_publish=100.0 - KEEPALIVE_INTERVAL / 2)

  def test_keepalive_beats_submaster_alive_window(self):
    # services.py declares gpsLocationExternal at 10 Hz and SubMaster.alive is
    # (now - last_recv) < 10/freq, so the topic must not go quiet for a whole second.
    assert KEEPALIVE_INTERVAL < 1.0
