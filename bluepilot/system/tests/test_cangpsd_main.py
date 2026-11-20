"""Unit tests for cangpsd's main() loop.

Everything else in this daemon is a pure function and tested as one. The loop was not, and
that gap has cost real drives: the auto-detect path shipped twice with a defect that no
decode test could have caught, because the defect was in what the loop *fed* the arbiter,
not in the arbiter. An offline harness that reimplements the publish loop cannot catch that
class of bug either -- a divergence between the harness and the daemon is exactly the thing
being missed. So these tests run the real `main()`.

What is faked is only the outside world: sockets, Params, the clock, and Ratekeeper. The
CANParser, the decode functions, the FixTracker and the arbiter are all the shipped ones,
and the CAN frames are encoded through the same DBC the daemon parses, so a signal renamed
out from under us fails here rather than silently decoding as absent.
"""
import datetime

import pytest

import cereal.messaging as messaging
from cereal import car
from opendbc.can.packer import CANPacker

from openpilot.bluepilot.system import cangpsd
from openpilot.bluepilot.system.cangps_fallback import (
  FALLBACK_PARAM,
  NO_FIX_TIMEOUT,
  SOURCE_CAN,
  SOURCE_DEVICE,
)

DBC_NAME = "ford_lincoln_base_pt"
VIN = "3FMTK3SU6PMA63566"
OTHER_VIN = "1FTFW1E80MFA00000"

# Somewhere in Seattle, in the degrees+minutes encoding 0x462 uses. The DBC gives the
# degrees signals offsets of -89/-179, so CANParser returns *signed* degrees and the
# hemisphere enums are redundant -- west is a negative degrees value, not enum 2. The enums
# are still set to the honest value so a future decode that consults them sees real data.
LAT_DEG, LAT_MIN, LAT_MIN_DEC = 47, 36, 0.5
LON_DEG, LON_MIN, LON_MIN_DEC = -122, 19, 0.5
HEMI_NORTH, HEMI_WEST = 2, 2
UTC = datetime.datetime(2026, 9, 2, 17, 45, 30)


class StopLoop(Exception):
  """Raised from the fake Ratekeeper to end main()'s `while True` at a known cycle."""


class FakeParams:
  def __init__(self, initial=None):
    self.d = dict(initial or {})
    self.puts = []

  def get(self, key, block=False):
    return self.d.get(key)

  def get_bool(self, key):
    return bool(self.d.get(key, False))

  def put(self, key, value, block=False):
    self.d[key] = value
    self.puts.append((key, value))

  def put_bool(self, key, value, block=False):
    self.put(key, value, block)


class FakePubMaster:
  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append((service, msg))


class FakeSubMaster:
  def __init__(self, services, harness):
    self.services = list(services)
    self.harness = harness
    self.alive = dict.fromkeys(self.services, True)

  def update(self, timeout=0):
    pass

  def __getitem__(self, service):
    return self.harness.device_gps


class DeviceGps:
  def __init__(self, has_fix=False):
    self.hasFix = has_fix


class Harness:
  """Drives main() for a bounded number of cycles against encoded CAN frames."""

  def __init__(self, monkeypatch, *, params=None, max_cycles=40,
               gps_hz=1.0, speed_kph=50.0, send_speed=True, inferred=False):
    self.packer = CANPacker(DBC_NAME)
    self.clock = 1000.0
    self.cycles = 0
    self.max_cycles = max_cycles
    self.gps_hz = gps_hz
    self.gps_until = None      # monotonic time after which the GPS burst stops
    self.speed_kph = speed_kph
    self.send_speed = send_speed
    self.inferred = inferred
    self.device_gps = DeviceGps(has_fix=False)
    self.pub = None
    self.submasters = []
    self.pubmaster_services = []
    self._last_gps = -1e9
    self.params = params if params is not None else self.default_params()
    self._install(monkeypatch)

  # ---- fixtures -----------------------------------------------------------------
  @staticmethod
  def car_params(vin=VIN):
    CP = car.CarParams.new_message()
    CP.carFingerprint = "FORD_MUSTANG_MACH_E_MK1"
    CP.carVin = vin
    CP.safetyConfigs = [car.CarParams.SafetyConfig.new_message()]
    return CP.to_bytes()

  @classmethod
  def default_params(cls, **over):
    p = {
      "CarParams": cls.car_params(),
      "UbloxAvailable": True,
      "FordPrefUseVehicleGps": False,
      "FordPrefAutoVehicleGps": True,
    }
    p.update(over)
    return FakeParams(p)

  # ---- CAN encoding -------------------------------------------------------------
  def _gps_frames(self):
    pos = self.packer.make_can_msg("APIMGPS_Data_Nav_1_FD1", 0, {
      "GPS_Latitude_Degrees": LAT_DEG, "GPS_Latitude_Minutes": LAT_MIN,
      "GPS_Latitude_Min_dec": LAT_MIN_DEC, "GpsHsphLattSth_D_Actl": HEMI_NORTH,
      "GPS_Longitude_Degrees": LON_DEG, "GPS_Longitude_Minutes": LON_MIN,
      "GPS_Longitude_Min_dec": LON_MIN_DEC, "GpsHsphLongEast_D_Actl": HEMI_WEST,
    })
    tim = self.packer.make_can_msg("APIMGPS_Data_Nav_2_FD1", 0, {
      "GpsUtcYr_No_Actl": UTC.year, "GpsUtcMnth_No_Actl": UTC.month,
      "GpsUtcDay_No_Actl": UTC.day, "GPS_UTC_hours": UTC.hour,
      "GPS_UTC_minutes": UTC.minute, "GPS_UTC_seconds": UTC.second,
      "GPS_Pdop": 1.0, "Gps_B_Falt": 0,
      "GPS_Actual_vs_Infer_pos": 1 if self.inferred else 0,
    })
    return [pos, tim]

  def _speed_frame(self):
    return self.packer.make_can_msg("BrakeSysFeatures", 0, {"Veh_V_ActlBrk": self.speed_kph})

  def next_frames(self):
    frames = []
    if self.send_speed:
      frames.append(self._speed_frame())
    gps_on = self.gps_hz and (self.gps_until is None or self.clock <= self.gps_until)
    if gps_on and (self.clock - self._last_gps) >= 1.0 / self.gps_hz:
      self._last_gps = self.clock
      frames += self._gps_frames()
    if not frames:
      return []
    msg = messaging.new_message('can', len(frames))
    for i, (addr, dat, bus) in enumerate(frames):
      msg.can[i].address = addr
      msg.can[i].dat = dat
      msg.can[i].src = bus
    return [msg.to_bytes()]

  # ---- patching -----------------------------------------------------------------
  def _install(self, mp):
    harness = self

    class FakeMessaging:
      def __getattr__(self, name):
        return getattr(messaging, name)

      def sub_sock(self, *a, **k):
        return "can_sock"

      def drain_sock_raw(self, sock, wait_for_one=False):
        return harness.next_frames()

      def SubMaster(self, services, *a, **k):
        sm = FakeSubMaster(services, harness)
        harness.submasters.append(sm)
        return sm

      def PubMaster(self, services, *a, **k):
        harness.pubmaster_services.append(list(services))
        harness.pub = FakePubMaster()
        return harness.pub

    class FakeClock:
      def monotonic(self):
        return harness.clock

      def time(self):
        return 1788000000.0

    class FakeRatekeeper:
      def __init__(self, rate, print_delay_threshold=None):
        self.dt = 1.0 / rate

      def keep_time(self):
        harness.clock += self.dt
        harness.cycles += 1
        if harness.cycles >= harness.max_cycles:
          raise StopLoop

    mp.setattr(cangpsd, "messaging", FakeMessaging())
    mp.setattr(cangpsd, "time", FakeClock())
    mp.setattr(cangpsd, "Ratekeeper", FakeRatekeeper)
    mp.setattr(cangpsd, "set_core_affinity", lambda cores: None)
    mp.setattr(cangpsd, "system_time_valid", lambda: True)
    mp.setattr(cangpsd, "Params", lambda: harness.params)

  def run(self):
    """Run main() to the cycle budget. Returns the SystemExit if it exited, else None."""
    try:
      cangpsd.main()
    except StopLoop:
      return None
    except SystemExit as e:
      return e
    raise AssertionError("main() returned, which it never should")

  @property
  def state(self):
    return self.params.d.get(FALLBACK_PARAM) or {}


@pytest.fixture
def mp(monkeypatch):
  return monkeypatch


class TestPublishingMode:
  def test_publishes_a_decoded_fix(self, mp):
    params = Harness.default_params(FordPrefUseVehicleGps=True)
    h = Harness(mp, params=params, max_cycles=40)
    h.run()

    assert h.pubmaster_services == [["gpsLocationExternal"]]
    assert h.pub is not None and len(h.pub.sent) > 0
    service, msg = h.pub.sent[-1]
    assert service == "gpsLocationExternal"
    gps = msg.gpsLocationExternal
    assert gps.hasFix
    assert gps.latitude == pytest.approx(47.608333, abs=1e-5)
    assert gps.longitude == pytest.approx(-122.325, abs=1e-5)
    assert gps.source == 'car'
    published = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000., datetime.UTC)
    assert abs((published.replace(tzinfo=None) - UTC).total_seconds()) < 2.0

  def test_manual_toggle_builds_no_submaster_at_all(self, mp):
    # The manual path skips measurement, so it must not add a msgq reader to any topic.
    params = Harness.default_params(FordPrefUseVehicleGps=True)
    h = Harness(mp, params=params, max_cycles=20)
    h.run()
    assert h.submasters == []

  def test_keepalive_republishes_when_the_bus_goes_quiet(self, mp):
    # Consumers hold SubMaster.alive off our publish cadence, so once we are a working GPS
    # a quiet bus must not let the topic go stale. Two seconds of GPS, then silence.
    params = Harness.default_params(FordPrefUseVehicleGps=True)
    h = Harness(mp, params=params, max_cycles=80)
    h.gps_until = h.clock + 2.0
    h.run()

    quiet_start = h.gps_until
    after = [m for _, m in h.pub.sent if m.logMonoTime is not None]
    assert len(after) >= 2
    # Publishing must continue past the last decoded frame, at roughly the keepalive rate.
    sent_while_quiet = sum(1 for _, m in h.pub.sent
                           if m.gpsLocationExternal.unixTimestampMillis > 0)
    assert sent_while_quiet >= 2
    # The held fix eventually goes stale; until MAX_FIX_AGE it is still reported as a fix.
    assert h.pub.sent[-1][1].gpsLocationExternal.source == 'car'
    assert quiet_start < h.clock


class TestObserverMode:
  def test_observer_publishes_nothing_and_opens_no_pub_socket(self, mp):
    h = Harness(mp, max_cycles=40)
    h.run()
    assert h.pubmaster_services == []
    assert h.pub is None

  def test_observer_never_subscribes_to_carstate(self, mp):
    """Regression guard: msgq allows NUM_READERS=15 per topic and carState already has 15.

    A 16th reader does not fail politely -- msgq evicts every subscriber, they all
    re-register, and the eviction storm takes engagement down with it. cangpsd must take
    vehicle speed off CAN instead. See SPEED_MESSAGE in cangpsd.py.
    """
    h = Harness(mp, max_cycles=20)
    h.run()
    assert len(h.submasters) == 1
    assert h.submasters[0].services == ["gpsLocationExternal"]
    assert "carState" not in h.submasters[0].services


class TestArbiterInputs:
  def test_accumulates_while_moving_with_no_device_fix(self, mp):
    # 10 Hz loop, so 1200 cycles is 120 s of simulated driving -- the accumulator persists
    # every 60 s, so anything shorter would assert against a param that was never written.
    h = Harness(mp, max_cycles=1200, speed_kph=50.0)
    h.run()
    assert h.state.get("no_fix_s", 0) > 60.0
    # Every accumulated second had a real car fix and no device fix, so the differential
    # evidence must track the window exactly.
    assert h.state["can_only_s"] == pytest.approx(h.state["no_fix_s"])
    assert h.state["source"] == SOURCE_DEVICE
    assert h.state["vin"] == VIN

  def test_stale_speed_stops_accumulation(self, mp):
    # A car that stops sending BrakeSysFeatures must read as not-moving, not freeze at the
    # last speed seen and keep accumulating while parked.
    h = Harness(mp, max_cycles=200, send_speed=False)
    h.run()
    assert h.state.get("no_fix_s", 0.0) == 0.0

  def test_parked_car_does_not_accumulate(self, mp):
    h = Harness(mp, max_cycles=200, speed_kph=0.0)
    h.run()
    assert h.state.get("no_fix_s", 0.0) == 0.0

  def test_device_fix_zeroes_the_window(self, mp):
    h = Harness(mp, max_cycles=200)
    h.device_gps.hasFix = True
    h.run()
    assert h.state.get("no_fix_s", 0.0) == 0.0

  def test_dead_reckoned_car_fix_is_not_evidence(self, mp):
    # The APIM asserts a fix while inferring, so a tunnel would otherwise satisfy the very
    # guard meant to catch it.
    h = Harness(mp, max_cycles=200, inferred=True)
    h.run()
    assert h.state.get("can_only_s", 0.0) == 0.0


class TestSwitching:
  def test_crossing_the_threshold_persists_can_and_exits(self, mp):
    # 10 Hz loop, so the threshold needs NO_FIX_TIMEOUT * 10 cycles, plus enough slack to
    # cover the seconds before the first fix is acquired (nothing accumulates until then).
    h = Harness(mp, max_cycles=int(NO_FIX_TIMEOUT * 10) + 1500, speed_kph=50.0)
    exc = h.run()
    assert exc is not None, "main() should exit so manager can restart it as publisher"
    assert exc.code == 0
    assert h.state["source"] == SOURCE_CAN
    assert h.state["flips"] == 1

  def test_a_decision_about_another_vehicle_is_not_inherited(self, mp):
    # Moving the device to a different car must re-test rather than carry the old verdict:
    # a CAN-GPS decision for another VIN must not put us straight into publishing mode.
    params = Harness.default_params()
    params.d[FALLBACK_PARAM] = {"vin": OTHER_VIN, "source": SOURCE_CAN,
                                "no_fix_s": 999.0, "can_only_s": 999.0, "flips": 1}
    h = Harness(mp, params=params, max_cycles=40)
    h.run()
    assert h.pubmaster_services == [], "inherited another car's decision and started publishing"
    assert h.submasters and h.submasters[0].services == ["gpsLocationExternal"]
