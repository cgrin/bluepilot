"""Unit tests for the automatic GPS source selection.

The arbiter is deliberately a pure state machine over (dt, moving, published_fix, can_fix),
with no sockets or params, so a twenty-minute threshold can be exercised in a few hundred
microseconds. The cases that matter are the ones where it must *not* switch: a receiver
that is merely in a tunnel, a car parked for a week, and a Ford whose own GPS is no better
than the one we would be leaving.
"""
import pytest

from openpilot.bluepilot.system.cangps_fallback import (
  MAX_FLIPS,
  NO_FIX_TIMEOUT,
  SOURCE_CAN,
  SOURCE_DEVICE,
  FallbackArbiter,
  FallbackState,
  load_state,
  vehicle_key,
)

DT = 0.5


def drive(arbiter, seconds, moving=True, published_fix=False, can_fix=True):
  """Run the arbiter for `seconds`, returning the number of source switches."""
  switches = 0
  for _ in range(int(seconds / DT)):
    switches += arbiter.update(DT, moving, published_fix, can_fix)
  return switches


class TestArbiter:
  def test_switches_to_can_after_the_threshold(self):
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    assert drive(a, NO_FIX_TIMEOUT - 60) == 0
    assert a.state.source == SOURCE_DEVICE
    # exactly to the threshold, so the switch is the last step -- past it the helper would
    # keep driving and start accumulating against the new source, as the daemon would only
    # do after restarting
    assert drive(a, 60) == 1
    assert a.state.source == SOURCE_CAN
    assert a.state.no_fix_s == 0.0
    assert a.state.flips == 1

  def test_one_fix_resets_the_accumulator(self):
    # A tunnel is not a blocked windshield. Anything short of "never works" has to leave
    # the driver on their own receiver.
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    drive(a, NO_FIX_TIMEOUT - 60)
    drive(a, 1, published_fix=True)
    assert a.state.no_fix_s == 0.0
    assert drive(a, 120) == 0
    assert a.state.source == SOURCE_DEVICE

  def test_parked_is_not_evidence(self):
    # The device can sit offroad for days under a carport. None of that says anything
    # about the antenna.
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    assert drive(a, NO_FIX_TIMEOUT * 3, moving=False) == 0
    assert a.state.no_fix_s == 0.0
    assert a.state.source == SOURCE_DEVICE

  def test_will_not_switch_to_a_car_with_no_fix_of_its_own(self):
    # A Ford with no 0x462, or an APIM that has no fix right now: switching would trade one
    # silent source for another and leave the driver worse off than stock.
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    assert drive(a, NO_FIX_TIMEOUT + 60, can_fix=False) == 0
    assert a.state.source == SOURCE_DEVICE
    assert a.state.flips == 0
    # ...but it re-arms rather than giving up, so a car that acquires later still wins
    assert a.state.no_fix_s < NO_FIX_TIMEOUT
    assert drive(a, NO_FIX_TIMEOUT + 60) == 1
    assert a.state.source == SOURCE_CAN

  def test_switches_back_when_can_gps_also_fails(self):
    a = FallbackArbiter(FallbackState(vin="VIN1", source=SOURCE_CAN))
    # publishing mode: our own fix is the published one, and it is absent
    assert drive(a, NO_FIX_TIMEOUT + 60, can_fix=False) == 1
    assert a.state.source == SOURCE_DEVICE

  def test_stops_flipping_after_max_flips(self):
    # Neither source works. Alternate a bounded number of times, then stay put on stock
    # behaviour rather than restarting the daemon every twenty minutes forever.
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    switches = drive(a, NO_FIX_TIMEOUT * (MAX_FLIPS + 3))
    assert switches == MAX_FLIPS
    assert a.state.settled
    assert drive(a, NO_FIX_TIMEOUT * 2) == 0

  def test_settled_state_stops_accumulating(self):
    a = FallbackArbiter(FallbackState(vin="VIN1", flips=MAX_FLIPS))
    assert drive(a, NO_FIX_TIMEOUT * 2) == 0
    assert a.state.no_fix_s == 0.0

  def test_progress_is_persisted_periodically(self):
    # The accumulator has to survive the mid-drive resets this device suffers, or a long
    # commute chopped into routes never reaches the threshold.
    a = FallbackArbiter(FallbackState(vin="VIN1"))
    drive(a, 30)
    assert not a.take_dirty()
    drive(a, 40)
    assert a.take_dirty()
    assert not a.take_dirty()


class TestVehicleKey:
  def test_prefers_vin(self):
    assert vehicle_key(FakeCP("VIN123", "FORD_MUSTANG_MACH_E")) == "VIN123"

  def test_falls_back_to_fingerprint(self):
    assert vehicle_key(FakeCP("", "FORD_MUSTANG_MACH_E")) == "FORD_MUSTANG_MACH_E"

  def test_no_car(self):
    assert vehicle_key(None) == ""


class FakeCP:
  def __init__(self, vin, fingerprint):
    self.carVin = vin
    self.carFingerprint = fingerprint


class FakeParams:
  def __init__(self, value):
    self.value = value

  def get(self, key):
    return self.value


class TestLoadState:
  def test_reads_back_what_was_written(self):
    p = FakeParams({"vin": "VIN1", "source": SOURCE_CAN, "no_fix_s": 12.5, "flips": 2})
    state = load_state(p, "VIN1")
    assert state.source == SOURCE_CAN
    assert state.no_fix_s == 12.5
    assert state.flips == 2

  def test_a_different_car_starts_over(self):
    # The blocked windshield belongs to one physical car; moving the device re-tests.
    p = FakeParams({"vin": "VIN1", "source": SOURCE_CAN, "no_fix_s": 12.5, "flips": 2})
    state = load_state(p, "VIN2")
    assert state == FallbackState(vin="VIN2")

  def test_unknown_vin_keeps_the_stored_decision(self):
    # Before card has fingerprinted we have no key. Discarding the decision then would
    # bounce the source on every boot.
    p = FakeParams({"vin": "VIN1", "source": SOURCE_CAN, "no_fix_s": 0.0, "flips": 1})
    assert load_state(p, "").source == SOURCE_CAN

  # "ublox" is the name SOURCE_DEVICE carried in an unreleased draft. No device ever wrote
  # it -- the auto-detect code was never deployed under that name -- so there is no
  # migration path, only this: an unrecognised source resets to stock behaviour rather than
  # leaving the daemon holding a value it cannot act on.
  @pytest.mark.parametrize("raw", [None, {}, "garbage", {"source": "moon"},
                                   {"source": "ublox"}, {"no_fix_s": "abc"}])
  def test_junk_falls_back_to_stock_behaviour(self, raw):
    assert load_state(FakeParams(raw), "VIN1") == FallbackState(vin="VIN1")
