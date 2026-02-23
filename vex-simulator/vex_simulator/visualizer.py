"""Pygame visual simulator for VEX V5 robot on a 144×144‑inch field."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING

import pygame  # type: ignore[import-untyped]

from vex_simulator.path_loader import (
    SplinePath,
    path_to_field_coords,
    waypoints_to_field_coords,
)

if TYPE_CHECKING:
    from vex_simulator.robot_state import RobotState

# ────────────────────────────────────────────────────────────────────────
# Layout
# ────────────────────────────────────────────────────────────────────────
FIELD_IN: int = 144
ROBOT_IN: int = 18
PPI: int = 5  # pixels per inch
FIELD_PX: int = FIELD_IN * PPI  # 720
PANEL_W: int = 280
WIN_W: int = FIELD_PX + PANEL_W  # 1000
WIN_H: int = FIELD_PX  # 720
FPS: int = 60
KV_OFFSET: int = 100  # px indent for values in side‑panel rows

# ────────────────────────────────────────────────────────────────────────
# Colour palette (dark‑theme)
# ────────────────────────────────────────────────────────────────────────
C_PANEL_BG = (25, 25, 30)
C_DIVIDER = (55, 55, 65)
C_LABEL = (140, 140, 155)
C_VALUE = (225, 228, 235)
C_TITLE = (255, 255, 255)
C_SECTION = (100, 195, 255)

C_ROBOT_FILL = (50, 55, 68)
C_ROBOT_EDGE = (130, 140, 160)
C_DIR_ARROW = (255, 220, 50)

C_SENSOR_F = (255, 90, 90)
C_SENSOR_R = (90, 130, 255)
C_SENSOR_L = (65, 200, 115)

C_BEAM_F = (255, 90, 90, 120)
C_BEAM_R = (90, 130, 255, 120)
C_BEAM_L = (65, 200, 115, 120)

BEAM_THICKNESS = 3

C_TRAIL = (100, 180, 255, 90)
C_FIELD_FALLBACK = (60, 90, 60)

# Path overlay colours
C_PATH_LINE = (255, 160, 40, 160)  # warm orange spline curve
C_PATH_WAYPOINT = (255, 80, 80)  # red waypoint dots
C_PATH_WP_RING = (255, 200, 200, 120)  # light ring around waypoints

# ────────────────────────────────────────────────────────────────────────
# Robot geometry (robot‑local frame, inches, origin=centre, +x=forward)
# ────────────────────────────────────────────────────────────────────────
# Distance‑sensor origins on robot edges
SEN_FRONT_POS = (9.0, 0.0)
SEN_RIGHT_POS = (0.0, -9.0)
SEN_LEFT_POS = (0.0, 9.0)
SEN_FRONT_DIR = (1.0, 0.0)
SEN_RIGHT_DIR = (0.0, -1.0)
SEN_LEFT_DIR = (0.0, 1.0)

# Direction arrow at front
ARROW_TIP = (9.0, 0.0)
ARROW_L = (6.5, 1.8)
ARROW_R = (6.5, -1.8)

# ────────────────────────────────────────────────────────────────────────
# Default field background image
# ────────────────────────────────────────────────────────────────────────
DEFAULT_FIELD_IMG = Path(__file__).resolve().parent.parent / "assets" / "Skills_field.png"


# ════════════════════════════════════════════════════════════════════════
class Visualizer:
    """Pygame‑based VEX V5 field + robot visualizer."""

    # ── construction ────────────────────────────────────────────────────
    def __init__(
        self,
        robot_state: RobotState,
        field_image: str | Path = DEFAULT_FIELD_IMG,
        paths: list[SplinePath] | None = None,
    ) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("VEX V5 Simulator")
        self.clock = pygame.time.Clock()
        self.robot = robot_state

        # Fonts (Consolas for a clean monospace look)
        self.font_title = pygame.font.SysFont("Consolas", 20, bold=True)
        self.font_section = pygame.font.SysFont("Consolas", 14, bold=True)
        self.font_label = pygame.font.SysFont("Consolas", 13)
        self.font_value = pygame.font.SysFont("Consolas", 13, bold=True)
        self.font_small = pygame.font.SysFont("Consolas", 11)

        # Transparent overlay for sensor beams + trail
        self.overlay = pygame.Surface((FIELD_PX, FIELD_PX), pygame.SRCALPHA)

        # Field background
        self.field_bg = self._load_field(field_image)

        # Movement trail
        self._trail: list[tuple[float, float]] = []
        self._trail_max = 800

        # Pre-computed path overlay (field-coordinate polylines)
        self._paths = paths or []
        self._path_field_lines: list[list[tuple[float, float]]] = []
        self._path_field_waypoints: list[list[tuple[float, float]]] = []
        self._precompute_paths()

        self.running = True

    # ── path pre-computation ────────────────────────────────────────────
    def _precompute_paths(self) -> None:
        """Transform loaded spline paths into field-coordinate polylines."""
        start_x = self.robot.cfg.start_x_in
        start_y = self.robot.cfg.start_y_in
        start_h = math.radians(self.robot.cfg.start_heading_deg)

        for sp in self._paths:
            field_pts = path_to_field_coords(
                sp.points,
                start_x,
                start_y,
                start_h,
            )
            self._path_field_lines.append(field_pts)

            wp_pts = waypoints_to_field_coords(
                sp.waypoints,
                start_x,
                start_y,
                start_h,
            )
            self._path_field_waypoints.append(wp_pts)

    # ── helpers ──────────────────────────────────────────────────────────
    @staticmethod
    def _load_field(path: str | Path) -> pygame.Surface | None:
        try:
            img = pygame.image.load(str(path)).convert()
            img = pygame.transform.smoothscale(img, (FIELD_PX, FIELD_PX))
            img = pygame.transform.rotate(img, 90)  # rotate 90° CW
            return img
        except (pygame.error, FileNotFoundError):
            print("[visualizer] field image not loaded — using fallback colour")
            return None

    def _f2s(self, fx: float, fy: float) -> tuple[int, int]:
        """Field coords (inches, origin bottom‑left) → screen px."""
        return int(fx * PPI), int((FIELD_IN - fy) * PPI)

    def _l2s(self, lx: float, ly: float) -> tuple[int, int]:
        """Robot‑local coords → screen px."""
        c = math.cos(self.robot.heading_rad)
        s = math.sin(self.robot.heading_rad)
        fx = self.robot.x + lx * s - ly * c
        fy = self.robot.y + lx * c + ly * s
        return self._f2s(fx, fy)

    def _l2f(self, lx: float, ly: float) -> tuple[float, float]:
        """Robot‑local coords → field coords (inches)."""
        c = math.cos(self.robot.heading_rad)
        s = math.sin(self.robot.heading_rad)
        return (self.robot.x + lx * s - ly * c, self.robot.y + lx * c + ly * s)

    def _rect_pts(
        self,
        cx: float,
        cy: float,
        w: float,
        h: float,
    ) -> list[tuple[int, int]]:
        """Screen polygon for a rectangle centred at (cx, cy) in local frame."""
        hw, hh = w / 2, h / 2
        return [
            self._l2s(cx + dx, cy + dy)
            for dx, dy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        ]

    # ── field ────────────────────────────────────────────────────────────
    def _draw_field(self) -> None:
        if self.field_bg:
            self.screen.blit(self.field_bg, (0, 0))
        else:
            pygame.draw.rect(
                self.screen,
                C_FIELD_FALLBACK,
                (0, 0, FIELD_PX, FIELD_PX),
            )
            # subtle grid every 24 in
            for i in range(0, FIELD_IN + 1, 24):
                px = i * PPI
                col = (80, 115, 80)
                pygame.draw.line(self.screen, col, (px, 0), (px, FIELD_PX), 1)
                pygame.draw.line(self.screen, col, (0, px), (FIELD_PX, px), 1)

    # ── path overlay ──────────────────────────────────────────────────────
    def _draw_paths(self) -> None:
        """Draw all loaded spline paths onto the transparent overlay."""
        for pts, wps in zip(self._path_field_lines, self._path_field_waypoints):
            if len(pts) >= 2:
                screen_pts = [self._f2s(fx, fy) for fx, fy in pts]
                pygame.draw.lines(self.overlay, C_PATH_LINE, False, screen_pts, 2)

            # Waypoint markers
            for fx, fy in wps:
                sx, sy = self._f2s(fx, fy)
                pygame.draw.circle(self.overlay, C_PATH_WP_RING, (sx, sy), 7)
                pygame.draw.circle(self.overlay, C_PATH_WAYPOINT, (sx, sy), 4)

    # ── trail ────────────────────────────────────────────────────────────
    def _update_trail(self) -> None:
        x, y = self.robot.x, self.robot.y
        if not self._trail or (
            abs(x - self._trail[-1][0]) > 0.15 or abs(y - self._trail[-1][1]) > 0.15
        ):
            self._trail.append((x, y))
            if len(self._trail) > self._trail_max:
                self._trail = self._trail[-self._trail_max // 2 :]

    def _draw_trail(self) -> None:
        if len(self._trail) < 2:
            return
        pts = [self._f2s(px, py) for px, py in self._trail]
        pygame.draw.lines(self.overlay, C_TRAIL, False, pts, 2)

    # ── sensor beams ─────────────────────────────────────────────────────
    def _beam_endpoints(
        self,
        local_pos: tuple[float, float],
        local_dir: tuple[float, float],
        dist_mm: float,
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        c = math.cos(self.robot.heading_rad)
        s = math.sin(self.robot.heading_rad)
        # field origin
        fx, fy = self._l2f(*local_pos)
        # field direction
        dx = local_dir[0] * s - local_dir[1] * c
        dy = local_dir[0] * c + local_dir[1] * s
        dist_in = dist_mm / 25.4
        return self._f2s(fx, fy), self._f2s(fx + dx * dist_in, fy + dy * dist_in)

    def _draw_sensor_beams(self) -> None:
        if not self.robot.cfg.distance_sensors:
            return

        _dir_to_visual = {
            "forward": (SEN_FRONT_POS, SEN_FRONT_DIR, C_BEAM_F),
            "right":   (SEN_RIGHT_POS, SEN_RIGHT_DIR, C_BEAM_R),
            "left":    (SEN_LEFT_POS,  SEN_LEFT_DIR,  C_BEAM_L),
            "backward": ((-9.0, 0.0), (-1.0, 0.0), C_BEAM_F),
        }
        _dir_to_dist = {
            "forward":  lambda: self.robot.dist_front_mm,
            "right":    lambda: self.robot.dist_right_mm,
            "left":     lambda: self.robot.dist_left_mm,
            "backward": lambda: self.robot.dist_front_mm,
        }
        for mount in self.robot.cfg.distance_sensors:
            vis = _dir_to_visual.get(mount.direction)
            get_dist = _dir_to_dist.get(mount.direction)
            if vis is None or get_dist is None:
                continue
            pos, direction, col = vis
            # Use actual configured offsets for position
            pos = (mount.offset_forward, -mount.offset_right)
            start, end = self._beam_endpoints(pos, direction, get_dist())
            pygame.draw.line(self.overlay, col, start, end, BEAM_THICKNESS)
            # hit‑point dot (more opaque)
            pygame.draw.circle(self.overlay, col[:3] + (180,), end, 5)

    # ── robot body ───────────────────────────────────────────────────────
    def _draw_robot(self) -> None:
        # Body rectangle (config-driven size)
        rsz = self.robot.cfg.robot_size_in
        body = self._rect_pts(0, 0, rsz, rsz)
        pygame.draw.polygon(self.screen, C_ROBOT_FILL, body)
        pygame.draw.polygon(self.screen, C_ROBOT_EDGE, body, 2)

        # Direction arrow at front so orientation is clear
        arrow = [self._l2s(*p) for p in (ARROW_TIP, ARROW_L, ARROW_R)]
        pygame.draw.polygon(self.screen, C_DIR_ARROW, arrow)

    # ── side panel ───────────────────────────────────────────────────────
    def _draw_panel(self) -> None:
        px = FIELD_PX + 20  # text left edge
        pygame.draw.rect(self.screen, C_PANEL_BG, (FIELD_PX, 0, PANEL_W, WIN_H))
        pygame.draw.line(self.screen, C_DIVIDER, (FIELD_PX, 0), (FIELD_PX, WIN_H), 2)

        y = 15
        # Title
        self.screen.blit(
            self.font_title.render("VEX V5 SIMULATOR", True, C_TITLE),
            (px, y),
        )
        y += 30
        y = self._divider(px, y)

        # ── Position ──
        y = self._sec(px, y, "POSITION")
        y = self._kv(px, y, "X", f"{self.robot.x:.1f} in")
        y = self._kv(px, y, "Y", f"{self.robot.y:.1f} in")
        hdeg = math.degrees(self.robot.heading_rad) % 360
        y = self._kv(px, y, "Heading", f"{hdeg:.1f}\u00b0")
        y += 6
        y = self._divider(px, y)

        # ── Left motors ──
        y = self._sec(px, y, "LEFT MOTORS")
        for p in self.robot.cfg.left_motor_ports:
            ap = abs(p)
            m = self.robot.motors.get(ap)
            rev = " \u21c4" if p < 0 else ""
            val = f"{m.velocity_rpm:+6.0f} RPM" if m else "\u2014"
            y = self._kv(px, y, f"Port {ap}{rev}", val)
        y += 4

        # ── Right motors ──
        y = self._sec(px, y, "RIGHT MOTORS")
        for p in self.robot.cfg.right_motor_ports:
            ap = abs(p)
            m = self.robot.motors.get(ap)
            rev = " \u21c4" if p < 0 else ""
            val = f"{m.velocity_rpm:+6.0f} RPM" if m else "\u2014"
            y = self._kv(px, y, f"Port {ap}{rev}", val)
        y += 4
        y = self._divider(px, y)

        # ── Distance sensors ──
        y = self._sec(px, y, "DISTANCE SENSORS")
        if self.robot.cfg.distance_sensors:
            _dir_colors = {
                "forward": ("Front", C_SENSOR_F),
                "right": ("Right", C_SENSOR_R),
                "left": ("Left", C_SENSOR_L),
                "backward": ("Back", C_SENSOR_F),
            }
            _dir_attrs = {
                "forward": "dist_front_mm",
                "right": "dist_right_mm",
                "left": "dist_left_mm",
            }
            for mount in self.robot.cfg.distance_sensors:
                label, col = _dir_colors.get(
                    mount.direction, (mount.direction.title(), C_SENSOR_F)
                )
                attr = _dir_attrs.get(mount.direction)
                val = getattr(self.robot, attr, 0.0) if attr else 0.0
                y = self._kv_dot(px, y, label, f"{val:.0f} mm", col)
        else:
            y = self._kv(px, y, "Status", "\u2014 none")
        y += 4
        y = self._divider(px, y)

        # ── Rotation sensors ──
        y = self._sec(px, y, "ROTATION SENSORS")
        if self.robot.rotation_sensors:
            for port in sorted(self.robot.rotation_sensors):
                rs = self.robot.rotation_sensors[port]
                deg = rs.position_cdeg / 100.0
                y = self._kv(px, y, f"Port {port}", f"{deg:.1f}\u00b0")
        else:
            y = self._kv(px, y, "Status", "\u2014 none")
        y += 4
        y = self._divider(px, y)

        # ── IMU ──
        y = self._sec(px, y, "IMU")
        if self.robot.imus:
            for port in sorted(self.robot.imus):
                imu = self.robot.imus[port]
                y = self._kv(px, y, f"Port {port}", f"{imu.heading_deg:.1f}\u00b0")
        else:
            y = self._kv(px, y, "Status", "\u2014 none")

        # ── Footer ──
        fps_txt = self.font_small.render(
            f"FPS: {self.clock.get_fps():.0f}",
            True,
            C_LABEL,
        )
        self.screen.blit(fps_txt, (px, WIN_H - 25))

    # panel helpers
    def _sec(self, x: int, y: int, title: str) -> int:
        self.screen.blit(
            self.font_section.render(title, True, C_SECTION),
            (x, y),
        )
        return y + 22

    def _kv(self, x: int, y: int, key: str, val: str) -> int:
        self.screen.blit(self.font_label.render(key, True, C_LABEL), (x + 4, y))
        self.screen.blit(self.font_value.render(val, True, C_VALUE), (x + KV_OFFSET, y))
        return y + 18

    def _kv_dot(
        self,
        x: int,
        y: int,
        key: str,
        val: str,
        col: tuple[int, int, int],
    ) -> int:
        pygame.draw.circle(self.screen, col, (x + 8, y + 7), 4)
        self.screen.blit(self.font_label.render(key, True, C_LABEL), (x + 18, y))
        self.screen.blit(self.font_value.render(val, True, C_VALUE), (x + KV_OFFSET, y))
        return y + 18

    def _divider(self, x: int, y: int) -> int:
        pygame.draw.line(
            self.screen,
            C_DIVIDER,
            (x, y),
            (x + PANEL_W - 40, y),
            1,
        )
        return y + 12

    # ── main loop ────────────────────────────────────────────────────────
    def run(self) -> None:
        """Main render loop — must be called from the main thread."""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self.running = False

            # Update trail
            self._update_trail()

            # Clear overlay
            self.overlay.fill((0, 0, 0, 0))

            # Draw layers
            self._draw_field()
            self._draw_paths()
            self._draw_trail()
            self._draw_sensor_beams()
            self.screen.blit(self.overlay, (0, 0))
            self._draw_robot()
            self._draw_panel()

            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()
