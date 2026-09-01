"""Pick a GPS source for this car without asking the user.

The problem cangpsd solves is invisible: a windshield that blocks the device's own GPS
antenna produces no error, no alert and no UI -- just routes stamped with the AGNOS flash
date and a nav stack that never sees a position. Nobody goes looking for a toggle for a
failure they have not noticed, so the toggle only ever helps people who already diagnosed
it. This module makes the choice automatically instead.

The two sources are the *device* receiver -- whichever daemon owns it, ubloxd/pigeond or
qcomgpsd, decided by the ublox_available() probe rather than by which comma the code is
running on -- and the car's own GPS on CAN. Which receiver is fitted changes none of the
reasoning here, so the state records "device" rather than naming one.

The rule is deliberately asymmetric. We leave the device receiver in charge until it has
*proven* it cannot work -- many minutes of driving with no fix at all -- and we only switch
away once the car's own CAN GPS has positively shown a fix of its own that we believe. That
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

# Named for the receiver, not the daemon that reads it: "device" covers both ubloxd/pigeond
# and qcomgpsd. An earlier draft called this "ublox", which was only ever true of some
# devices -- and which model has which receiver is not something this file should encode.
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
# is enough to have genuinely tried both, after which the state freezes until the car
# changes.
#
# Note where it freezes: the flips strictly alternate from SOURCE_DEVICE, so the third one
# always lands on SOURCE_CAN. That is deliberate, not an accident of the count -- switching
# *to* CAN requires positive proof that the car has a real fix of its own, while
# switching back to the device receiver requires no such evidence. The transitions toward
# CAN are the better-evidenced ones, so if we must give up somewhere, give up there. An
# even MAX_FLIPS would settle on the device receiver instead; that would be the choice to
# make if resting on stock behaviour ever matters more than resting on the source we have
# actually seen work.
MAX_FLIPS = 3

# How much of the no-fix window the car must have covered with a real fix of its own before
# we will switch to it. This is a *differential* measurement -- time where the car had an
# actual fix and the active source had none -- which is the only thing that distinguishes
# the two cases that both end in "twenty minutes of moving with no fix":
#
#   Blocked windshield: the device receiver produces nothing across the whole window,
#   including long stretches of open sky where the car is fixed the entire time. Minutes of
#   direct head-to-head evidence that one source works here and the other does not.
#
#   Long tunnel, working receiver: outside, both sources are fixed, so any device fix zeroes
#   the window before it can grow. Inside, the device receiver has nothing -- but neither
#   does the car, which is dead reckoning through the same tunnel. The window can only reach
#   the threshold on tunnel time, and tunnel time contributes no differential evidence.
#
# Hence *actual* fixes only: the APIM asserts a fix while inferring (the 15.7% of frames the
# drives found flagged GPS_Actual_vs_Infer_pos), so counting inferred ones would make a
# tunnel look like proof that the car's GPS works underground.
#
# Accumulated across the window rather than sampled at its end, and cumulative rather than
# continuous. Sampling at the end would discard fifteen minutes of open-sky evidence just
# because the threshold happened to fall inside a tunnel -- and since a refusal restarts the
# window, a tunnel at a fixed point in a commute could defer the switch forever.
#
# Thirty seconds is a low bar deliberately: a genuinely blocked windshield clears it by
# orders of magnitude (minutes to tens of minutes), while the tunnel case sits at zero. It
# exists to reject noise, not to be a hurdle.
CAN_FIX_MIN_S = 30.0


@dataclass
class FallbackState:
  """What we have decided about one vehicle, and how far along the current measurement is."""
  vin: str = ""
  source: str = SOURCE_DEVICE
  no_fix_s: float = 0.0
  # Of that no_fix_s, how much the car covered with an actual fix. Persisted alongside it
  # because they describe one window: this device's mid-drive resets would otherwise drop
  # the evidence while keeping the accumulator that needs it.
  can_only_s: float = 0.0
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
      can_only_s=float(raw.get("can_only_s", 0.0)),
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
  anything is publishing a fix on the GPS topic, and whether the CAN GPS currently has a
  real (not dead-reckoned) fix of its own. `update` returns True on the cycle the selection
  changes; the caller is expected to persist the state and restart so the new source owns
  the topic cleanly.
  """

  def __init__(self, state: FallbackState):
    self.state = state
    self.since_persist = 0.0
    self.dirty = False

  def update(self, dt: float, moving: bool, published_fix: bool, can_actual_fix: bool) -> bool:
    """Advance the measurement by `dt` seconds. True if the selected source changed.

    `can_actual_fix` must be an *actual* CAN fix -- the caller is responsible for excluding
    dead-reckoned ones, since the APIM reports a fix either way.
    """
    if self.state.settled:
      return False

    if published_fix:
      # One fix is proof the active source works. Anything we had accumulated against it
      # was a transient -- a tunnel, a parking structure -- not a dead antenna. The
      # differential evidence goes with it: it only means anything within one window.
      if self.state.no_fix_s != 0.0 or self.state.can_only_s != 0.0:
        self.state.no_fix_s = 0.0
        self.state.can_only_s = 0.0
        self.dirty = True
      return False

    if not moving:
      # Parked with no fix is not evidence. Sky view offroad is whatever the driver
      # happened to park under, and the device may sit there for days.
      return False

    self.state.no_fix_s += dt
    # The active source has nothing right now. If the car does -- a real fix, not a
    # dead-reckoned one -- then this second is head-to-head evidence between them.
    if can_actual_fix:
      self.state.can_only_s += dt
    self.since_persist += dt
    if self.since_persist >= PERSIST_INTERVAL:
      self.since_persist = 0.0
      self.dirty = True

    if self.state.no_fix_s < NO_FIX_TIMEOUT:
      return False

    return self._switch(self.state.can_only_s >= CAN_FIX_MIN_S)

  def _switch(self, can_ready: bool) -> bool:
    """The active source has failed its threshold. Move only if there is somewhere to go."""
    if self.state.source == SOURCE_DEVICE:
      if not can_ready:
        # The window filled up, but almost none of it was time where the car had a real
        # fix and the device receiver did not -- a Ford with no GPS on CAN, or a tunnel
        # both receivers spent inferring or blind. Nothing here says CAN GPS would do
        # better, so restart the window rather than trade one silent source for another.
        self.state.no_fix_s = 0.0
        self.state.can_only_s = 0.0
        self.dirty = True
        return False
      new_source = SOURCE_CAN
    else:
      # CAN GPS was selected and has now gone this long without a fix. Hand the topic back
      # and let the device receiver have another go; if it fails too we will come back
      # here, and the flip count bounds the loop.
      new_source = SOURCE_DEVICE

    evidence = f"{self.state.can_only_s:.0f}s of it with a real CAN fix"
    reason = f"after {self.state.no_fix_s:.0f}s of driving with no fix ({evidence})"
    cloudlog.warning(f"cangps_fallback: switching GPS source {self.state.source} -> {new_source} {reason}")
    self.state.source = new_source
    self.state.no_fix_s = 0.0
    self.state.can_only_s = 0.0
    self.state.flips += 1
    self.dirty = True
    if self.state.settled:
      cloudlog.warning(f"cangps_fallback: out of attempts for this vehicle, staying on {new_source}")
    return True

  def take_dirty(self) -> bool:
    """True once per pending change, so the caller knows to write the param."""
    dirty, self.dirty = self.dirty, False
    return dirty
