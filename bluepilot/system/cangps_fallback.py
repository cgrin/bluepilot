"""Pick a GPS source for this car without asking the user.

The problem cangpsd solves is invisible: a windshield that blocks the device's own GPS
antenna produces no error, no alert and no UI -- just routes stamped with the AGNOS flash
date and a nav stack that never sees a position. Nobody goes looking for a toggle for a
failure they have not noticed, so the toggle only ever helps people who already diagnosed
it. This module makes the choice automatically instead.

The two sources are the *device* receiver -- whichever daemon owns it, ubloxd/pigeond on a
comma three or qcomgpsd on a 3X -- and the car's own GPS on CAN. Which chip is fitted does
not change any of the reasoning here, so the state records "device" rather than naming one.

The rule is deliberately asymmetric. We leave the device receiver in charge until it has
*proven* it cannot work -- many minutes of driving with no fix at all -- and we only switch
away once the car's own CAN GPS has *positively* shown a fix in the same session. That
ordering matters: a Ford with no 0x462 (or an APIM that never gets a fix of its own) must
never end up worse off than stock, and "the device receiver is quiet" alone is not evidence
that anything else would do better.

Two things make the measurement honest:

  - Only moving time counts. A device sat in a garage for a week is not evidence about an
    antenna, and cold-start TTFF on a working receiver is seconds of driving, not minutes.
  - Any single fix resets the accumulator to zero. One fix means the antenna works; the
    threshold is only ever reached by a receiver that produces nothing at all.

The decision is per-vehicle and keyed on VIN, because a blocked windshield is a property
of one physical car -- moving the device to another car re-tests from scratch.

cangpsd can watch both sources at once even though only one may publish: msgq's
one-publisher-per-topic rule constrains sending on the GPS topic, not decoding CAN. So the
observe mode reads 0x462/0x463 off the bus exactly as it would when publishing, while the
device's own GPS daemon still owns the topic, and the comparison is real rather than
sequential.
"""
import json
from dataclasses import asdict, dataclass

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

FALLBACK_PARAM = "CanGpsFallbackState"

# Named for the receiver, not the daemon that reads it: "device" covers ubloxd/pigeond on a
# comma three and qcomgpsd on a 3X. An earlier draft called this "ublox", which was only ever
# true of one of the two.
SOURCE_DEVICE = "device"
SOURCE_CAN = "can"

# Seconds of *moving* with no fix before a source is declared dead. Twenty minutes is
# chosen to be far past any legitimate outage: cold-start TTFF is seconds, a long tunnel
# is single-digit minutes, and the worst real-world urban canyon still yields something
# inside a commute. Counting seconds rather than drives matters -- three short errands
# into a covered garage would trip a drive counter, and one long tree-lined commute would
# not trip it at all.
NO_FIX_TIMEOUT = 20 * 60.0

# vEgo below this is not driving, so it is not evidence either way.
MIN_SPEED = 0.5

# Persist the accumulator this often. It has to survive a reboot -- this device hard-resets
# mid-drive on stock software, which chops one commute into several routes, and an
# in-memory counter would restart from zero each time and never reach the threshold.
PERSIST_INTERVAL = 60.0

# Stop switching after this many changes on one VIN. On a car where neither source works
# the rule above would otherwise alternate forever, one flip per threshold. Three attempts
# is enough to have genuinely tried both, after which we park on ublox (the stock
# behaviour) and leave it alone until the car changes.
MAX_FLIPS = 3


@dataclass
class FallbackState:
  """What we have decided about one vehicle, and how far along the current measurement is."""
  vin: str = ""
  source: str = SOURCE_DEVICE
  no_fix_s: float = 0.0
  flips: int = 0

  @property
  def settled(self) -> bool:
    """Out of attempts: stop measuring and stay where we are."""
    return self.flips >= MAX_FLIPS


def vehicle_key(CP) -> str:
  """A stable per-vehicle identity, or "" if we do not have one yet.

  VIN is the right key -- the blocked windshield belongs to one physical car, not to a
  model -- but it is queried during fingerprinting and some cars never answer, so fall
  back to the fingerprint. That is coarser (every Mach-E shares it) but still resets when
  the device moves to a different kind of car, which is the case that matters most.
  """
  if CP is None:
    return ""
  return CP.carVin or CP.carFingerprint or ""


def load_state(params: Params, vin: str) -> FallbackState:
  """Read the stored decision, discarding one made about a different car."""
  raw = params.get(FALLBACK_PARAM)
  if not isinstance(raw, dict) or not raw:
    return FallbackState(vin=vin)
  try:
    state = FallbackState(
      vin=str(raw.get("vin", "")),
      source=str(raw.get("source", SOURCE_DEVICE)),
      no_fix_s=float(raw.get("no_fix_s", 0.0)),
      flips=int(raw.get("flips", 0)),
    )
  except (TypeError, ValueError):
    cloudlog.exception(f"cangps_fallback: {FALLBACK_PARAM} is malformed, starting over")
    return FallbackState(vin=vin)

  if state.source not in (SOURCE_DEVICE, SOURCE_CAN):
    return FallbackState(vin=vin)
  # A decision about another car tells us nothing about this one.
  if vin and state.vin != vin:
    return FallbackState(vin=vin)
  return state


def save_state(params: Params, state: FallbackState) -> None:
  params.put(FALLBACK_PARAM, asdict(state))


def can_gps_selected(params: Params, vin: str) -> bool:
  """Has CAN GPS been selected for this vehicle? Read by manager to stand ubloxd down."""
  return load_state(params, vin).source == SOURCE_CAN


def json_state(state: FallbackState) -> str:
  """Compact rendering for logs."""
  return json.dumps(asdict(state), sort_keys=True)


class FallbackArbiter:
  """Accumulates evidence about the active GPS source and decides when to switch.

  Fed once per cycle with what the daemon can see: whether the car is moving, whether
  anything is publishing a fix on gpsLocationExternal, and whether the CAN GPS has a fix
  of its own. `update` returns True on the cycle the selection changes; the caller is
  expected to persist the state and restart so the new source owns the topic cleanly.
  """

  def __init__(self, state: FallbackState):
    self.state = state
    self.since_persist = 0.0
    self.dirty = False

  def update(self, dt: float, moving: bool, published_fix: bool, can_fix: bool) -> bool:
    """Advance the measurement by `dt` seconds. True if the selected source changed."""
    if self.state.settled:
      return False

    if published_fix:
      # One fix is proof the active source works. Anything we had accumulated against it
      # was a transient -- a tunnel, a parking structure -- not a dead antenna.
      if self.state.no_fix_s != 0.0:
        self.state.no_fix_s = 0.0
        self.dirty = True
      return False

    if not moving:
      # Parked with no fix is not evidence. Sky view offroad is whatever the driver
      # happened to park under, and the device may sit there for days.
      return False

    self.state.no_fix_s += dt
    self.since_persist += dt
    if self.since_persist >= PERSIST_INTERVAL:
      self.since_persist = 0.0
      self.dirty = True

    if self.state.no_fix_s < NO_FIX_TIMEOUT:
      return False

    return self._switch(can_fix)

  def _switch(self, can_fix: bool) -> bool:
    """The active source has failed its threshold. Move only if there is somewhere to go."""
    if self.state.source == SOURCE_DEVICE:
      if not can_fix:
        # The device receiver is dead, but the car is not offering a working fix either
        # -- this is a Ford with no GPS on CAN, or an APIM that has none right now.
        # Switching would trade one silent source for another, so keep waiting and
        # re-check in another NO_FIX_TIMEOUT of driving.
        self.state.no_fix_s = 0.0
        self.dirty = True
        return False
      new_source = SOURCE_CAN
    else:
      # CAN GPS was selected and has now gone this long without a fix. Hand the topic back
      # and let the device receiver have another go; if it fails too we will come back
      # here, and the flip count bounds the loop.
      new_source = SOURCE_DEVICE

    reason = f"after {self.state.no_fix_s:.0f}s of driving with no fix"
    cloudlog.warning(f"cangps_fallback: switching GPS source {self.state.source} -> {new_source} {reason}")
    self.state.source = new_source
    self.state.no_fix_s = 0.0
    self.state.flips += 1
    self.dirty = True
    if self.state.settled:
      cloudlog.warning(f"cangps_fallback: out of attempts for this vehicle, staying on {new_source}")
    return True

  def take_dirty(self) -> bool:
    """True once per pending change, so the caller knows to write the param."""
    dirty, self.dirty = self.dirty, False
    return dirty
