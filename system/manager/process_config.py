import functools
import os
import operator
import platform

import cereal.messaging as messaging
from cereal import car, custom
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.bluepilot import is_bluepilot
from openpilot.bluepilot.system.cangps_fallback import can_gps_selected, vehicle_key
from openpilot.system.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess
from openpilot.system.hardware.hw import Paths

from openpilot.sunnypilot.mapd.mapd_manager import MAPD_PATH

from openpilot.sunnypilot.models.helpers import get_active_model_runner
from openpilot.sunnypilot.sunnylink.utils import sunnylink_need_register, sunnylink_ready, use_sunnylink_uploader

WEBCAM = os.getenv("USE_WEBCAM") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

@functools.cache
def prev_route_car() -> tuple[str, str]:
  """The (brand, vehicle key) recorded on the previous drive, or ("", "") if there isn't one.

  manager evaluates process gates against the live `carParams` message, which card only
  publishes once it has fingerprinted the car -- about six seconds into a boot. Any gate
  that keys off the brand is therefore wrong for those six seconds, which is long enough
  for ubloxd/pigeond to start, claim gpsLocationExternal, and be killed again.
  CarParamsPersistent is written by card on every drive and survives reboots, so on any
  boot after the first in a given car it supplies the brand immediately.

  Cached for the life of the process: card is the only writer, and by the time it has run
  the live CP is available and takes precedence below anyway.
  """
  cp_bytes = Params().get("CarParamsPersistent")
  if cp_bytes is None:
    return "", ""
  try:
    CP = messaging.log_from_bytes(cp_bytes, car.CarParams)
  except Exception:
    cloudlog.exception("process_config: failed to deserialize CarParamsPersistent")
    return "", ""
  return CP.brand, vehicle_key(CP)

def _car_identity(CP: car.CarParams) -> tuple[str, str]:
  # Prefer the live CP; fall back to the last drive's only while card is still
  # fingerprinting, so ubloxd never gets a head start on the topic cangpsd is about to own.
  prev_brand, prev_key = prev_route_car()
  return (CP.brand or prev_brand), (vehicle_key(CP) or prev_key)


def can_gps_capable(started: bool, params: Params, CP: car.CarParams) -> bool:
  """Could CAN GPS work here at all? Ford is the only brand whose DBC carries a fix.

  Deliberately not gated on ublox_available(). It was, back when cangpsd hardcoded
  gpsLocationExternal -- on a Quectel device every consumer reads gpsLocation instead, so
  that publish would have reached nobody and the toggle only looked like it worked. cangpsd
  now resolves its topic through get_gps_location_service(), the same call the consumers
  make, so it is useful on both; whichever device GPS daemon owns that topic yields below.
  """
  brand, _ = _car_identity(CP)
  return started and brand == "ford"


def cangpsd(started: bool, params: Params, CP: car.CarParams) -> bool:
  """Run the daemon -- as the publisher when it is the selected source, else as an observer.

  Two independent flags, so the automatic path can be trialled without disturbing anyone
  relying on the manual one. FordPrefUseVehicleGps forces CAN GPS on, as it always has;
  FordPrefAutoVehicleGps hands the choice to cangps_fallback, which needs the daemon
  running even while the device receiver still owns the topic, so it has something to
  compare against.
  """
  if not can_gps_capable(started, params, CP):
    return False
  return params.get_bool("FordPrefUseVehicleGps") or params.get_bool("FordPrefAutoVehicleGps")


def can_gps_publishing(started: bool, params: Params, CP: car.CarParams) -> bool:
  """Does cangpsd own the GPS topic right now? If so the device GPS daemon stands down.

  Which daemon that is follows the hardware: ubloxd/pigeond on a ublox device, qcomgpsd on
  a Quectel one. Both gates below consult this, and observer mode deliberately does not
  reach it -- an observer publishes nothing, so nothing has to yield to it.
  """
  if not can_gps_capable(started, params, CP):
    return False
  if params.get_bool("FordPrefUseVehicleGps"):
    return True
  if not params.get_bool("FordPrefAutoVehicleGps"):
    return False
  _, key = _car_identity(CP)
  return can_gps_selected(params, key)

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox, block=True)
  # Gate only the ubloxd/pigeond process start here; the UbloxAvailable param above stays driven
  # by raw ublox_available() so gpsLocationExternal/gpsLocation routing (common/gps.py,
  # locationd.cc) is unaffected by whether cangpsd is providing GPS instead. cangpsd reads
  # that same routing to pick its topic, so flipping the param here would move it too.
  return started and use_ublox and not can_gps_publishing(started, params, CP)

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LateralManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  # Yield gpsLocation to cangpsd when it is the selected source, mirroring what the ublox
  # gate does for ubloxd/pigeond. msgq allows only one publisher per topic.
  return started and not ublox_available() and not can_gps_publishing(started, params, CP)

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def use_github_runner(started, params, CP: car.CarParams) -> bool:
  return not PC and params.get_bool("EnableGithubRunner") and (
    not params.get_bool("NetworkMetered") and not params.get_bool("GithubRunnerSufficientVoltage"))

def use_copyparty(started, params, CP: car.CarParams) -> bool:
  return bool(params.get_bool("EnableCopyparty"))

def sunnylink_ready_shim(started, params, CP: car.CarParams) -> bool:
  """Shim for sunnylink_ready to match the process manager signature."""
  return sunnylink_ready(params)

def sunnylink_need_register_shim(started, params, CP: car.CarParams) -> bool:
  """Shim for sunnylink_need_register to match the process manager signature."""
  return sunnylink_need_register(params)

def use_sunnylink_uploader_shim(started, params, CP: car.CarParams) -> bool:
  """Shim for use_sunnylink_uploader to match the process manager signature."""
  return use_sunnylink_uploader(params)

def is_tinygrad_model(started, params, CP: car.CarParams) -> bool:
  """Check if the active model runner is SNPE."""
  return bool(get_active_model_runner(params, not started) == custom.ModelManagerSP.Runner.tinygrad)

def is_stock_model(started, params, CP: car.CarParams) -> bool:
  """Check if the active model runner is stock."""
  return bool(get_active_model_runner(params, not started) == custom.ModelManagerSP.Runner.stock)

def mapd_ready(started: bool, params: Params, CP: car.CarParams) -> bool:
  return bool(os.path.exists(Paths.mapd_root()))

def uploader_ready(started: bool, params: Params, CP: car.CarParams) -> bool:
  if not params.get_bool("OnroadUploads"):
    return only_offroad(started, params, CP)

  return always_run(started, params, CP)

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

procs = [
  DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "system/loggerd", ["./loggerd"], logging),
  NativeProcess("encoderd", "system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], notcar),
  PythonProcess("logmessaged", "system.logmessaged", always_run),

  NativeProcess("camerad", "system/camerad", ["./camerad"], driverview, enabled=not WEBCAM),
  PythonProcess("webcamerad", "tools.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "system.proclogd", only_onroad, enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "system.journald", only_onroad, platform.system() != "Darwin"),
  PythonProcess("micd", "system.micd", iscar),
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "selfdrive.modeld.modeld", and_(only_onroad, is_stock_model)),
  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", driverview, enabled=(WEBCAM or not PC)),

  PythonProcess("sensord", "system.sensord.sensord", only_onroad, enabled=not PC),
  PythonProcess("ui", "selfdrive.ui.ui", always_run, restart_if_crash=True),
  # BluePilot: use a fork-local subclass for optional custom sounds; upstream soundd remains unchanged.
  PythonProcess("soundd", "selfdrive.ui.bp.soundd_bp" if is_bluepilot() else "selfdrive.ui.soundd", driverview),
  # End BluePilot
  PythonProcess("locationd", "selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", driverview, enabled=(WEBCAM or not PC)),
  # BluePilot: restart_if_crash -- a diag-port serial fault (see qcomgpsd.py's reconnect
  # handling) is now recovered in-process, but this is a backstop for anything else that
  # still takes the process down; without it a crash meant no GPS for the rest of the drive.
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI, restart_if_crash=True),
  PythonProcess("pandad", "selfdrive.pandad.pandad", always_run),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("lateral_maneuversd", "tools.lateral_maneuvers.lateral_maneuversd", lat_maneuver),
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  PythonProcess("hardwared", "system.hardware.hardwared", always_run),
  PythonProcess("modem", "system.hardware.tici.modem", always_run, enabled=TICI),
  PythonProcess("tombstoned", "system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "system.updated.updated", only_offroad, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", uploader_ready),
  PythonProcess("statsd", "system.statsd", always_run),
  PythonProcess("feedbackd", "selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", notcar),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),
  PythonProcess("joystick", "tools.joystick.joystick_control", and_(joystick, iscar)),

  # sunnylink <3
  DaemonProcess("manage_sunnylinkd", "sunnypilot.sunnylink.athena.manage_sunnylinkd", "SunnylinkdPid"),
  PythonProcess("sunnylink_registration_manager", "sunnypilot.sunnylink.registration_manager", sunnylink_need_register_shim),
  PythonProcess("statsd_sp", "sunnypilot.sunnylink.statsd", and_(always_run, sunnylink_ready_shim)),
]

# sunnypilot
procs += [
  # Models
  PythonProcess("models_manager", "sunnypilot.models.manager", only_offroad),
  NativeProcess("modeld_tinygrad", "sunnypilot/modeld_v2", ["./modeld"], and_(only_onroad, is_tinygrad_model)),

  # Backup
  PythonProcess("backup_manager", "sunnypilot.sunnylink.backups.manager", and_(only_offroad, sunnylink_ready_shim)),

  # mapd
  NativeProcess("mapd", Paths.mapd_root(), ["bash", "-c", f"{MAPD_PATH} > /dev/null 2>&1"], mapd_ready),
  PythonProcess("mapd_manager", "sunnypilot.mapd.mapd_manager", always_run),

  # locationd
  NativeProcess("locationd_llk", "sunnypilot/selfdrive/locationd", ["./locationd"], only_onroad),
]

# BluePilot: portal and route preprocessor processes
if is_bluepilot():
  def _bp_portal_enabled(started, params, CP):
    return params.get_bool("EnableWebRoutesServer")
  def _bp_route_preprocessor_enabled(started, params, CP):
    return params.get_bool("EnableWebRoutesServer") and only_offroad(started, params, CP)
  procs += [
    PythonProcess("bp_portal", "bluepilot.backend.bp_portal", _bp_portal_enabled),
    PythonProcess("bp_route_preprocessor", "bluepilot.backend.routes.preprocessor", _bp_route_preprocessor_enabled),
    # restart_if_crash because cangpsd exits deliberately when the arbiter changes GPS
    # source -- process death is how it releases (or is guaranteed not to hold) the
    # gpsLocationExternal publisher before the other source takes over.
    PythonProcess("cangpsd", "bluepilot.system.cangpsd", cangpsd, enabled=TICI, restart_if_crash=True),
  ]

if os.path.exists("./github_runner.sh"):
  procs += [NativeProcess("github_runner_start", "system/manager", ["./github_runner.sh", "start"], and_(only_offroad, use_github_runner), sigkill=False)]

if os.path.exists("../../sunnypilot/sunnylink/uploader.py"):
  procs += [PythonProcess("sunnylink_uploader", "sunnypilot.sunnylink.uploader", use_sunnylink_uploader_shim)]

if os.path.exists("../../third_party/copyparty/copyparty-sfx.py"):
  sunnypilot_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
  copyparty_args = [f"-v{Paths.crash_log_root()}:/swaglogs:r"]
  copyparty_args += [f"-v{Paths.log_root()}:/routes:r"]
  copyparty_args += [f"-v{Paths.model_root()}:/models:rw"]
  copyparty_args += [f"-v{sunnypilot_root}:/sunnypilot:rw"]
  copyparty_args += ["-p8080"]
  copyparty_args += ["-z"]
  copyparty_args += ["-q"]
  procs += [NativeProcess("copyparty-sfx", "third_party/copyparty", ["./copyparty-sfx.py", *copyparty_args], and_(only_offroad, use_copyparty))]

managed_processes = {p.name: p for p in procs}
