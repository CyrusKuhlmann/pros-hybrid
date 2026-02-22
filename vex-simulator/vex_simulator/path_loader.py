"""Load paths defined in the C++ pros-sim source and provide field-coordinate spline points.

The C++ code defines paths using CatmullRomPath with waypoints relative to the
robot's starting position (0, 0).  This module:

1. Recursively scans all ``.cpp`` and ``.h`` files under the ``pros-sim/src``
   and ``pros-sim/include`` directories to extract CatmullRomPath definitions.
2. Implements the *same* Hermite-spline maths as ``path.cpp`` so the rendered
   curve matches exactly what the C++ sim follows.
3. Transforms the spline samples from robot-start-relative coordinates to
   absolute field coordinates using the configured starting pose.

Coordinate convention (matches the C++ code):
    +x = right,  +y = forward (north)
    θ in degrees, CW-positive from +y (forward)
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence


# ────────────────────────────────────────────────────────────────────────
# Data types
# ────────────────────────────────────────────────────────────────────────


@dataclass
class Pose:
    """2-D position + heading (mirrors the C++ ``Pose`` struct).

    x, y are in inches.  theta is in degrees, CW-positive from +y.
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # degrees, CW from +y

    def theta_rad(self) -> float:
        """Convert theta to radians (same CW-from-+y convention)."""
        return math.radians(self.theta)


@dataclass
class PathPoint:
    """Single sample on a Catmull-Rom spline."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # degrees, CW from +y
    curvature: float = 0.0
    arc_length: float = 0.0


@dataclass
class SplinePath:
    """A named path consisting of waypoints and interpolated samples."""

    name: str
    waypoints: list[Pose]
    points: list[PathPoint] = field(default_factory=list)


# ────────────────────────────────────────────────────────────────────────
# Hermite spline maths (port of path.cpp)
# ────────────────────────────────────────────────────────────────────────


def _hermite(
    p0: tuple[float, float],
    p1: tuple[float, float],
    m0: tuple[float, float],
    m1: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Evaluate cubic Hermite at *t* ∈ [0, 1]."""
    t2 = t * t
    t3 = t2 * t
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2
    return (
        h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
        h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
    )


def _hermite_deriv(
    p0: tuple[float, float],
    p1: tuple[float, float],
    m0: tuple[float, float],
    m1: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """First derivative of cubic Hermite at *t*."""
    t2 = t * t
    return (
        (6 * t2 - 6 * t) * p0[0]
        + (3 * t2 - 4 * t + 1) * m0[0]
        + (-6 * t2 + 6 * t) * p1[0]
        + (3 * t2 - 2 * t) * m1[0],
        (6 * t2 - 6 * t) * p0[1]
        + (3 * t2 - 4 * t + 1) * m0[1]
        + (-6 * t2 + 6 * t) * p1[1]
        + (3 * t2 - 2 * t) * m1[1],
    )


def _normalise_deg(deg: float) -> float:
    """Normalise an angle in degrees to [0, 360)."""
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


# ────────────────────────────────────────────────────────────────────────
# Catmull-Rom spline builder (mirrors CatmullRomPath::buildSpline)
# ────────────────────────────────────────────────────────────────────────


def build_spline(
    waypoints: Sequence[Pose],
    samples_per_segment: int = 100,
) -> list[PathPoint]:
    """Build a Catmull-Rom spline through *waypoints* and return samples.

    The maths exactly mirror ``CatmullRomPath::buildSpline()`` from
    ``path.cpp`` so the rendered curve matches the C++ path.

    Coordinate convention:
        +x = right, +y = forward
        theta in degrees, CW from +y
        Tangent direction at theta: (sin theta, cos theta)
    """
    if len(waypoints) < 2:
        return []

    points: list[PathPoint] = []
    n = len(waypoints)

    for seg in range(n - 1):
        wp0 = waypoints[seg]
        wp1 = waypoints[seg + 1]

        p0 = (wp0.x, wp0.y)
        p1 = (wp1.x, wp1.y)

        chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if chord < 1e-9:
            chord = 1.0

        scale = chord

        # Tangent from heading (CW-from-+y): forward = (sin θ, cos θ)
        t0 = math.radians(wp0.theta)
        t1 = math.radians(wp1.theta)
        m0 = (math.sin(t0) * scale, math.cos(t0) * scale)
        m1 = (math.sin(t1) * scale, math.cos(t1) * scale)

        start_t = 0 if seg == 0 else 1
        for i in range(start_t, samples_per_segment + 1):
            t = i / samples_per_segment
            pos = _hermite(p0, p1, m0, m1, t)
            d1 = _hermite_deriv(p0, p1, m0, m1, t)

            pp = PathPoint()
            pp.x = pos[0]
            pp.y = pos[1]
            # Heading: atan2(dx, dy) gives CW-from-+y angle
            pp.theta = _normalise_deg(math.degrees(math.atan2(d1[0], d1[1])))
            points.append(pp)

    # Cumulative arc-length
    if points:
        points[0].arc_length = 0.0
        for i in range(1, len(points)):
            dx = points[i].x - points[i - 1].x
            dy = points[i].y - points[i - 1].y
            points[i].arc_length = points[i - 1].arc_length + math.hypot(dx, dy)

    return points


# ────────────────────────────────────────────────────────────────────────
# C++ source parser
# ────────────────────────────────────────────────────────────────────────

# Regex to find CatmullRomPath variable definitions in C++ source.
# Handles:
#   CatmullRomPath name({ {x,y,theta}, ... });
#   CatmullRomPath name({ {x,y,theta}, ... }, alpha, samples);
_PATH_RE = re.compile(
    r"CatmullRomPath\s+(\w+)\s*\(\s*\{([\s\S]*?)\}\s*(?:,\s*[\d.]+\s*)?(?:,\s*\d+\s*)?\)\s*;",
)

# Regex to pull individual {x, y, theta} tuples from the waypoint list.
_WP_RE = re.compile(
    r"\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*(?:,\s*(-?[\d.]+)\s*)?\}",
)


def parse_paths_from_cpp(source_path: str | Path) -> list[SplinePath]:
    """Parse ``CatmullRomPath`` definitions from a single C++ source file.

    Returns a list of `SplinePath` objects with waypoints populated and
    spline samples computed.  Theta values in the C++ source are in
    degrees (CW from +y), matching the new convention.
    """
    source_path = Path(source_path)
    if not source_path.exists():
        print(f"[path_loader] C++ source not found: {source_path}")
        return []

    text = source_path.read_text(encoding="utf-8", errors="replace")
    paths: list[SplinePath] = []

    for m in _PATH_RE.finditer(text):
        name = m.group(1)
        body = m.group(2)
        waypoints: list[Pose] = []
        for wm in _WP_RE.finditer(body):
            x = float(wm.group(1))
            y = float(wm.group(2))
            theta = float(wm.group(3)) if wm.group(3) else 0.0
            waypoints.append(Pose(x, y, theta))
        if len(waypoints) >= 2:
            sp = SplinePath(name=name, waypoints=waypoints)
            sp.points = build_spline(waypoints)
            paths.append(sp)

    if paths:
        print(f"[path_loader] Loaded {len(paths)} path(s) from {source_path.name}")

    return paths


def parse_paths_from_directory(root: str | Path) -> list[SplinePath]:
    """Recursively scan a directory for CatmullRomPath definitions in all C++ files.

    Searches every ``.cpp``, ``.h``, and ``.hpp`` file under *root* (including
    subdirectories) and returns all discovered paths.

    Parameters
    ----------
    root:
        Top-level project directory (e.g. ``pros-sim/``).  Both ``src/`` and
        ``include/`` subtrees are scanned.
    """
    root = Path(root)
    if not root.exists():
        print(f"[path_loader] Project directory not found: {root}")
        return []

    all_paths: list[SplinePath] = []
    seen_names: set[str] = set()
    cpp_extensions = {".cpp", ".h", ".hpp", ".cc", ".cxx", ".hxx"}

    source_files = sorted(
        f for f in root.rglob("*") if f.suffix in cpp_extensions and f.is_file()
    )

    for src_file in source_files:
        file_paths = parse_paths_from_cpp(src_file)
        for sp in file_paths:
            if sp.name not in seen_names:
                seen_names.add(sp.name)
                all_paths.append(sp)
            else:
                print(
                    f"[path_loader] Skipping duplicate path '{sp.name}' in "
                    f"{src_file.name} (already loaded)"
                )

    if all_paths:
        print(
            f"[path_loader] Total: {len(all_paths)} path(s) from "
            f"{len(source_files)} file(s) scanned under {root.name}/"
        )
    else:
        print(
            f"[path_loader] No CatmullRomPath definitions found in any file "
            f"under {root.name}/"
        )

    return all_paths


# ────────────────────────────────────────────────────────────────────────
# Coordinate transform: robot-start-relative → field absolute
# ────────────────────────────────────────────────────────────────────────


def path_to_field_coords(
    points: Sequence[PathPoint],
    start_x: float,
    start_y: float,
    start_heading_rad: float,
) -> list[tuple[float, float]]:
    """Convert path samples from robot-start-relative to field coordinates.

    Path coordinate system (robot-local, matches C++ odom):
        (0, 0) = robot starting position
        +x = right,  +y = forward

    Field coordinate system (visualiser):
        +x = right (east),  +y = up (north)
        heading 0 = north = +y,  CW positive

    At heading 0 the two frames are aligned (+x right, +y north/forward).

    Parameters
    ----------
    points:
        Spline samples in robot-start-relative coordinates.
    start_x, start_y:
        Robot starting position on the field (inches).
    start_heading_rad:
        Robot starting heading on the field (radians, 0 = north, CW +).
    """
    h = start_heading_rad
    cos_h = math.cos(h)
    sin_h = math.sin(h)

    result: list[tuple[float, float]] = []
    for pp in points:
        # Rotate local (x_right, y_forward) by heading into field frame:
        #   field_x = start_x + x_local * cos(h) + y_local * sin(h)
        #   field_y = start_y - x_local * sin(h) + y_local * cos(h)
        fx = start_x + pp.x * cos_h + pp.y * sin_h
        fy = start_y - pp.x * sin_h + pp.y * cos_h
        result.append((fx, fy))

    return result


def waypoints_to_field_coords(
    waypoints: Sequence[Pose],
    start_x: float,
    start_y: float,
    start_heading_rad: float,
) -> list[tuple[float, float]]:
    """Convert raw waypoints to field coordinates (same transform)."""
    h = start_heading_rad
    cos_h = math.cos(h)
    sin_h = math.sin(h)
    result: list[tuple[float, float]] = []
    for wp in waypoints:
        fx = start_x + wp.x * cos_h + wp.y * sin_h
        fy = start_y - wp.x * sin_h + wp.y * cos_h
        result.append((fx, fy))
    return result
