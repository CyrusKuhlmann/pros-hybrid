"""Robot configuration — load from TOML, provide typed dataclass."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class SensorMount:
    """Position and direction of a sensor mounted on the robot.

    All values are in inches, relative to the robot centre.
    *offset_forward* is positive toward the front of the robot.
    *offset_right* is positive toward the right side.
    *direction* is one of "forward", "backward", "left", "right".
    """

    port: int
    offset_forward: float
    offset_right: float
    direction: str

    # Pre-computed unit direction vector (set by post_init)
    _dx: float = field(init=False, repr=False, default=0.0)
    _dy: float = field(init=False, repr=False, default=0.0)

    def __post_init__(self) -> None:
        _dir_map = {
            "forward": (1.0, 0.0),
            "backward": (-1.0, 0.0),
            "left": (0.0, 1.0),
            "right": (0.0, -1.0),
        }
        self._dx, self._dy = _dir_map.get(self.direction, (1.0, 0.0))


@dataclass
class DeadWheelMount:
    """A dead-wheel (unpowered tracking wheel) mounted on the robot.

    All offsets are in inches, relative to the robot centre.
    *offset_forward* is positive toward the front of the robot.
    *offset_right* is positive toward the right side.
    *orientation* is ``"forward"`` (wheel rolls with forward motion)
    or ``"lateral"`` (wheel rolls with sideways motion).
    *wheel_diameter_in* is the dead-wheel diameter in inches.
    """

    port: int
    offset_forward: float
    offset_right: float
    orientation: str  # "forward" or "lateral"
    wheel_diameter_in: float

    @property
    def wheel_radius_m(self) -> float:
        """Wheel radius in metres."""
        return (self.wheel_diameter_in / 2.0) * 0.0254


@dataclass
class RobotConfig:
    """All tuneable parameters that define a specific robot build."""

    # -- Drivetrain ----------------------------------------------------------
    wheel_diameter_in: float
    track_width_in: float
    motor_max_rpm: float
    left_motor_ports: list[int]
    right_motor_ports: list[int]

    # -- Chassis -------------------------------------------------------------
    robot_size_in: float

    # -- Starting pose -------------------------------------------------------
    start_x_in: float
    start_y_in: float
    start_heading_deg: float

    # -- Physics tuning (reasonable defaults) --------------------------------
    robot_mass_kg: float = 7.0  # ~15 lbs competition robot
    robot_moi_kg_m2: float = 0.12  # Moment of inertia about vertical axis
    rolling_friction_coeff: float = 0.02  # Coulomb friction (force = coeff * m * g)

    # -- Sensor mounts -------------------------------------------------------
    distance_sensors: list[SensorMount] = field(default_factory=list)
    imu_port: int | None = None
    rotation_sensors: list[DeadWheelMount] = field(default_factory=list)

    # -- Derived (computed once) ---------------------------------------------
    @property
    def wheel_radius_m(self) -> float:
        """Wheel radius in metres."""
        return (self.wheel_diameter_in / 2.0) * 0.0254

    @property
    def track_width_m(self) -> float:
        """Track width (centre-to-centre) in metres."""
        return self.track_width_in * 0.0254

    @property
    def half_robot_in(self) -> float:
        """Half the robot size in inches (for wall collision)."""
        return self.robot_size_in / 2.0

    @property
    def motor_free_speed_rads(self) -> float:
        """Motor free speed in rad/s."""
        import math

        return self.motor_max_rpm * math.pi / 30.0


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG_NAME = "robot_config.toml"


def load_config(path: str | Path | None = None) -> RobotConfig:
    """Load a `RobotConfig` from a TOML file.

    If *path* is ``None`` the loader looks for ``robot_config.toml``
    in the current working directory.  The file **must** exist.
    """
    if path is None:
        path = Path.cwd() / _DEFAULT_CONFIG_NAME
    else:
        path = Path(path)

    if not path.exists():
        raise FileNotFoundError(
            f"Robot configuration file not found: {path}\n"
            f"Please create a '{_DEFAULT_CONFIG_NAME}' file."
        )

    with path.open("rb") as fh:
        raw = tomllib.load(fh)

    return _parse(raw)


def _parse(raw: dict) -> RobotConfig:
    """Convert raw TOML dict into a `RobotConfig`.

    All required sections and keys must be present in the TOML file.
    """
    dt = raw["drivetrain"]
    ch = raw["chassis"]
    sp = raw["start_pose"]
    sensors = raw.get("sensors", {})
    phys = raw.get("physics", {})

    distance_sensors = [
        SensorMount(
            port=int(ds["port"]),
            offset_forward=float(ds["offset_forward"]),
            offset_right=float(ds["offset_right"]),
            direction=str(ds["direction"]),
        )
        for ds in sensors.get("distance", [])
    ]

    rotation_sensors = [
        DeadWheelMount(
            port=int(rs["port"]),
            offset_forward=float(rs["offset_forward"]),
            offset_right=float(rs["offset_right"]),
            orientation=str(rs["orientation"]),
            wheel_diameter_in=float(rs["wheel_diameter_in"]),
        )
        for rs in sensors.get("rotation", [])
    ]

    return RobotConfig(
        wheel_diameter_in=float(dt["wheel_diameter_in"]),
        track_width_in=float(dt["track_width_in"]),
        motor_max_rpm=float(dt["motor_max_rpm"]),
        left_motor_ports=list(dt["left_motor_ports"]),
        right_motor_ports=list(dt["right_motor_ports"]),
        robot_size_in=float(ch["robot_size_in"]),
        start_x_in=float(sp["x_in"]),
        start_y_in=float(sp["y_in"]),
        start_heading_deg=float(sp["heading_deg"]),
        distance_sensors=distance_sensors,
        imu_port=int(sensors["imu_port"]) if "imu_port" in sensors else None,
        rotation_sensors=rotation_sensors,
        robot_mass_kg=float(phys.get("robot_mass_kg", 7.0)),
        robot_moi_kg_m2=float(phys.get("robot_moi_kg_m2", 0.12)),
        rolling_friction_coeff=float(phys.get("rolling_friction_coeff", 0.02)),
    )
