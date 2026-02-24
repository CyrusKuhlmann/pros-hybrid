"""Interactive path editor for creating CatmullRomPath waypoints on the VEX field.

Launch with ``python -m vex_simulator editor`` to open the editor.  Click on
the field to place waypoints, drag them to reposition, and drag the heading
handle (small circle at the end of the yellow line) to adjust theta.  Press
**S** to export the path as a C++ ``CatmullRomPath`` definition that can be
pasted directly into the pros-sim source.
"""

from __future__ import annotations

import math
from pathlib import Path

import pygame  # type: ignore[import-untyped]

from vex_simulator.path_loader import Pose, build_spline

# ────────────────────────────────────────────────────────────────────────
# Layout
# ────────────────────────────────────────────────────────────────────────
FIELD_IN: int = 144
PPI: int = 5
FIELD_PX: int = FIELD_IN * PPI  # 720
PANEL_W: int = 320
WIN_W: int = FIELD_PX + PANEL_W  # 1040
WIN_H: int = FIELD_PX  # 720
FPS: int = 60
KV_OFFSET: int = 105

# ────────────────────────────────────────────────────────────────────────
# Colour palette
# ────────────────────────────────────────────────────────────────────────
C_BG = (25, 25, 30)
C_DIVIDER = (55, 55, 65)
C_LABEL = (140, 140, 155)
C_VALUE = (225, 228, 235)
C_TITLE = (255, 255, 255)
C_SECTION = (100, 195, 255)
C_FIELD_FALLBACK = (60, 90, 60)

# Waypoint / handle colours
C_WP_NORMAL = (255, 80, 80)
C_WP_SELECTED = (255, 220, 50)
C_WP_RING = (255, 200, 200, 120)
C_WP_RING_SELECTED = (255, 255, 100, 180)
C_WP_RING_HOVER = (255, 220, 180, 150)

C_THETA_HANDLE = (255, 220, 50)
C_THETA_HANDLE_HOVER = (255, 255, 180)
C_THETA_LINE = (255, 220, 50, 180)
C_THETA_LINE_SELECTED = (255, 255, 100, 220)

# Spline colour
C_SPLINE = (255, 160, 40, 200)

# Saved paths (faded)
C_SAVED_PATH = (150, 150, 150, 80)

# Start pose marker
C_START_MARKER = (0, 255, 100, 150)

# Mouse coord display
C_MOUSE_COORD = (200, 200, 210)

# ────────────────────────────────────────────────────────────────────────
# Sizes
# ────────────────────────────────────────────────────────────────────────
WP_RADIUS: int = 8
THETA_HANDLE_LEN: int = 35  # pixels
THETA_HANDLE_RADIUS: int = 6
GRAB_RADIUS: int = 15  # pixel distance for selection

# ────────────────────────────────────────────────────────────────────────
# Default field image (same one used by Visualizer)
# ────────────────────────────────────────────────────────────────────────
DEFAULT_FIELD_IMG = (
    Path(__file__).resolve().parent.parent / "assets" / "Skills_field.png"
)


# ════════════════════════════════════════════════════════════════════════
class PathEditor:
    """Interactive VEX field path editor with live Catmull-Rom preview."""

    # ── construction ────────────────────────────────────────────────────
    def __init__(
        self,
        start_x: float = 72.0,
        start_y: float = 72.0,
        start_heading_deg: float = 0.0,
        field_image: str | Path = DEFAULT_FIELD_IMG,
        output_dir: str | Path = ".",
    ) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("VEX Path Editor")
        self.clock = pygame.time.Clock()

        self.start_x = start_x
        self.start_y = start_y
        self.start_heading_deg = start_heading_deg
        self.output_dir = Path(output_dir)

        # Fonts
        self.font_title = pygame.font.SysFont("Consolas", 20, bold=True)
        self.font_section = pygame.font.SysFont("Consolas", 14, bold=True)
        self.font_label = pygame.font.SysFont("Consolas", 13)
        self.font_value = pygame.font.SysFont("Consolas", 13, bold=True)
        self.font_small = pygame.font.SysFont("Consolas", 11)
        self.font_help = pygame.font.SysFont("Consolas", 10)

        # Field image
        self.field_bg = self._load_field(field_image)

        # Transparent overlay
        self.overlay = pygame.Surface((FIELD_PX, FIELD_PX), pygame.SRCALPHA)

        # ── Path state ──────────────────────────────────────────────────
        self.path_name: str = "path1"
        self.waypoints: list[Pose] = []  # stored in FIELD coordinates
        self.spline_field_pts: list[tuple[float, float]] = []

        # ── Interaction state ───────────────────────────────────────────
        self.selected_idx: int | None = None
        self.dragging_wp: bool = False
        self.dragging_theta: bool = False
        self.hover_wp: int | None = None
        self.hover_theta: int | None = None

        # Text-input mode (for path name)
        self.naming_mode: bool = False
        self.name_buffer: str = ""

        # Mouse position (field-coords, updated every frame)
        self.mouse_field: tuple[float, float] | None = None

        # Status bar
        self._status_msg: str = ""
        self._status_frames: int = 0

        # Previously saved paths (rendered faded on field)
        self.saved_paths: list[tuple[str, list[tuple[float, float]]]] = []

        self.running: bool = True

    # ── field image ─────────────────────────────────────────────────────
    @staticmethod
    def _load_field(path: str | Path) -> pygame.Surface | None:
        try:
            img = pygame.image.load(str(path)).convert()
            img = pygame.transform.smoothscale(img, (FIELD_PX, FIELD_PX))
            img = pygame.transform.rotate(img, 90)
            return img
        except (pygame.error, FileNotFoundError):
            print("[path_editor] field image not loaded — using fallback colour")
            return None

    # ────────────────────────────────────────────────────────────────────
    # Coordinate helpers
    # ────────────────────────────────────────────────────────────────────
    def _f2s(self, fx: float, fy: float) -> tuple[int, int]:
        """Field coords (inches, origin bottom-left) → screen px."""
        return int(fx * PPI), int((FIELD_IN - fy) * PPI)

    def _s2f(self, sx: int, sy: int) -> tuple[float, float]:
        """Screen px → field coords (inches)."""
        return sx / PPI, FIELD_IN - sy / PPI

    def _field_to_robot_rel(
        self, fx: float, fy: float, ftheta: float
    ) -> tuple[float, float, float]:
        """Convert a field-coordinate waypoint to robot-start-relative coords.

        The inverse of the transform in ``path_loader.path_to_field_coords``.
        """
        h = math.radians(self.start_heading_deg)
        dx = fx - self.start_x
        dy = fy - self.start_y
        rx = dx * math.cos(h) - dy * math.sin(h)
        ry = dx * math.sin(h) + dy * math.cos(h)
        rtheta = ftheta - self.start_heading_deg
        # Keep in [-360, 360] to match typical C++ usage
        while rtheta > 360:
            rtheta -= 360
        while rtheta < -360:
            rtheta += 360
        return rx, ry, rtheta

    # ────────────────────────────────────────────────────────────────────
    # Spline recomputation
    # ────────────────────────────────────────────────────────────────────
    def _recompute_spline(self) -> None:
        """Rebuild the spline curve from the current waypoints."""
        if len(self.waypoints) < 2:
            self.spline_field_pts = []
            return
        # build_spline works in any coordinate frame — pass field-coords
        samples = build_spline(self.waypoints, samples_per_segment=100)
        self.spline_field_pts = [(p.x, p.y) for p in samples]

    # ────────────────────────────────────────────────────────────────────
    # Theta-handle geometry
    # ────────────────────────────────────────────────────────────────────
    def _theta_handle_screen(self, idx: int) -> tuple[int, int]:
        """Screen position of the theta-drag handle for waypoint *idx*."""
        wp = self.waypoints[idx]
        sx, sy = self._f2s(wp.x, wp.y)
        a = math.radians(wp.theta)
        # sin(θ) → right on screen, cos(θ) → up on screen
        hx = sx + THETA_HANDLE_LEN * math.sin(a)
        hy = sy - THETA_HANDLE_LEN * math.cos(a)
        return int(hx), int(hy)

    # ────────────────────────────────────────────────────────────────────
    # Hit testing
    # ────────────────────────────────────────────────────────────────────
    def _wp_at(self, sx: int, sy: int) -> int | None:
        for i, wp in enumerate(self.waypoints):
            wx, wy = self._f2s(wp.x, wp.y)
            if math.hypot(sx - wx, sy - wy) < GRAB_RADIUS:
                return i
        return None

    def _theta_at(self, sx: int, sy: int) -> int | None:
        for i in range(len(self.waypoints)):
            hx, hy = self._theta_handle_screen(i)
            if math.hypot(sx - hx, sy - hy) < GRAB_RADIUS:
                return i
        return None

    # ────────────────────────────────────────────────────────────────────
    # Event handling
    # ────────────────────────────────────────────────────────────────────
    def _handle_events(self) -> None:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.running = False
                return

            # ── naming mode eats keyboard events ──
            if self.naming_mode:
                if ev.type == pygame.KEYDOWN:
                    self._handle_naming_key(ev)
                continue

            if ev.type == pygame.KEYDOWN:
                self._handle_key(ev)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                self._handle_mouse_down(ev)
            elif ev.type == pygame.MOUSEBUTTONUP:
                self._handle_mouse_up(ev)
            elif ev.type == pygame.MOUSEMOTION:
                self._handle_mouse_motion(ev)

    # ── naming ──────────────────────────────────────────────────────────
    def _handle_naming_key(self, ev: pygame.event.Event) -> None:
        if ev.key == pygame.K_RETURN:
            self.path_name = self.name_buffer or self.path_name
            self.naming_mode = False
            self._flash(f"Path name → {self.path_name}")
        elif ev.key == pygame.K_ESCAPE:
            self.naming_mode = False
        elif ev.key == pygame.K_BACKSPACE:
            self.name_buffer = self.name_buffer[:-1]
        elif ev.unicode and ev.unicode.isprintable() and len(self.name_buffer) < 30:
            ch = ev.unicode
            if ch.isalnum() or ch == "_":
                self.name_buffer += ch

    # ── keyboard ────────────────────────────────────────────────────────
    def _handle_key(self, ev: pygame.event.Event) -> None:
        if ev.key == pygame.K_ESCAPE:
            self.running = False
        elif ev.key == pygame.K_n:
            self.naming_mode = True
            self.name_buffer = self.path_name
        elif ev.key == pygame.K_s:
            self._save_path()
        elif ev.key == pygame.K_c:
            self.waypoints.clear()
            self.spline_field_pts.clear()
            self.selected_idx = None
            self._flash("Path cleared")
        elif ev.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
            self._delete_selected()
        elif ev.key == pygame.K_TAB:
            # Cycle selection through waypoints
            if self.waypoints:
                if self.selected_idx is None:
                    self.selected_idx = 0
                else:
                    self.selected_idx = (self.selected_idx + 1) % len(self.waypoints)

    # ── mouse ───────────────────────────────────────────────────────────
    def _handle_mouse_down(self, ev: pygame.event.Event) -> None:
        sx, sy = ev.pos
        if sx >= FIELD_PX:
            return  # click on panel — ignore

        if ev.button == 1:  # left click
            # Priority: theta handle → waypoint → add new
            tidx = self._theta_at(sx, sy)
            if tidx is not None:
                self.selected_idx = tidx
                self.dragging_theta = True
                return

            widx = self._wp_at(sx, sy)
            if widx is not None:
                self.selected_idx = widx
                self.dragging_wp = True
                return

            # Add a new waypoint
            fx, fy = self._s2f(sx, sy)
            theta = 0.0
            if self.waypoints:
                prev = self.waypoints[-1]
                dx, dy = fx - prev.x, fy - prev.y
                if math.hypot(dx, dy) > 0.5:
                    theta = math.degrees(math.atan2(dx, dy))
            self.waypoints.append(Pose(fx, fy, theta))
            self.selected_idx = len(self.waypoints) - 1
            self._recompute_spline()
            self._flash(f"Waypoint {len(self.waypoints)} added")

        elif ev.button == 3:  # right click — delete nearest
            widx = self._wp_at(sx, sy)
            if widx is not None:
                del self.waypoints[widx]
                if self.selected_idx == widx:
                    self.selected_idx = None
                elif self.selected_idx is not None and self.selected_idx > widx:
                    self.selected_idx -= 1
                self._recompute_spline()
                self._flash("Waypoint deleted")

    def _handle_mouse_up(self, _ev: pygame.event.Event) -> None:
        self.dragging_wp = False
        self.dragging_theta = False

    def _handle_mouse_motion(self, ev: pygame.event.Event) -> None:
        sx, sy = ev.pos

        # Update mouse field position for coordinate readout
        if sx < FIELD_PX:
            self.mouse_field = self._s2f(sx, sy)
        else:
            self.mouse_field = None

        # Dragging a waypoint
        if self.dragging_wp and self.selected_idx is not None:
            fx, fy = self._s2f(sx, sy)
            fx = max(0.0, min(float(FIELD_IN), fx))
            fy = max(0.0, min(float(FIELD_IN), fy))
            self.waypoints[self.selected_idx].x = fx
            self.waypoints[self.selected_idx].y = fy
            self._recompute_spline()
            return

        # Dragging a theta handle
        if self.dragging_theta and self.selected_idx is not None:
            wp = self.waypoints[self.selected_idx]
            wx, wy = self._f2s(wp.x, wp.y)
            dx = sx - wx
            dy = wy - sy  # invert for screen-y → field-y
            if math.hypot(dx, dy) > 3:
                wp.theta = math.degrees(math.atan2(dx, dy))
                self._recompute_spline()
            return

        # Hover detection
        if sx < FIELD_PX:
            self.hover_wp = self._wp_at(sx, sy)
            self.hover_theta = self._theta_at(sx, sy) if self.hover_wp is None else None
        else:
            self.hover_wp = None
            self.hover_theta = None

    # ────────────────────────────────────────────────────────────────────
    # Delete selected waypoint
    # ────────────────────────────────────────────────────────────────────
    def _delete_selected(self) -> None:
        if self.selected_idx is not None and self.selected_idx < len(self.waypoints):
            del self.waypoints[self.selected_idx]
            self.selected_idx = None
            self._recompute_spline()
            self._flash("Waypoint deleted")

    # ────────────────────────────────────────────────────────────────────
    # Status flash
    # ────────────────────────────────────────────────────────────────────
    def _flash(self, msg: str, frames: int = 150) -> None:
        self._status_msg = msg
        self._status_frames = frames

    # ════════════════════════════════════════════════════════════════════
    # Drawing
    # ════════════════════════════════════════════════════════════════════

    def _draw_field(self) -> None:
        if self.field_bg:
            self.screen.blit(self.field_bg, (0, 0))
        else:
            pygame.draw.rect(self.screen, C_FIELD_FALLBACK, (0, 0, FIELD_PX, FIELD_PX))
            for i in range(0, FIELD_IN + 1, 24):
                px = i * PPI
                col = (80, 115, 80)
                pygame.draw.line(self.screen, col, (px, 0), (px, FIELD_PX), 1)
                pygame.draw.line(self.screen, col, (0, px), (FIELD_PX, px), 1)

    def _draw_start_marker(self) -> None:
        """Show robot starting position + heading direction."""
        sx, sy = self._f2s(self.start_x, self.start_y)
        pygame.draw.circle(self.overlay, C_START_MARKER, (sx, sy), 10, 2)
        h = math.radians(self.start_heading_deg)
        ex = sx + 22 * math.sin(h)
        ey = sy - 22 * math.cos(h)
        pygame.draw.line(self.overlay, C_START_MARKER, (sx, sy), (int(ex), int(ey)), 2)
        # label
        lbl = self.font_small.render("START", True, (0, 220, 90))
        self.overlay.blit(lbl, (sx - lbl.get_width() // 2, sy + 14))

    def _draw_saved_paths(self) -> None:
        for _name, pts in self.saved_paths:
            if len(pts) >= 2:
                scr = [self._f2s(x, y) for x, y in pts]
                pygame.draw.lines(self.overlay, C_SAVED_PATH, False, scr, 2)

    def _draw_spline(self) -> None:
        if len(self.spline_field_pts) < 2:
            return
        scr = [self._f2s(x, y) for x, y in self.spline_field_pts]
        pygame.draw.lines(self.overlay, C_SPLINE, False, scr, 3)

    def _draw_waypoints(self) -> None:
        for i, wp in enumerate(self.waypoints):
            sx, sy = self._f2s(wp.x, wp.y)
            is_sel = i == self.selected_idx
            is_hov = i == self.hover_wp

            # ── theta handle ──
            hx, hy = self._theta_handle_screen(i)
            lcol = C_THETA_LINE_SELECTED if is_sel else C_THETA_LINE
            pygame.draw.line(self.overlay, lcol, (sx, sy), (hx, hy), 2)
            hcol = C_THETA_HANDLE_HOVER if i == self.hover_theta else C_THETA_HANDLE
            pygame.draw.circle(self.overlay, hcol, (hx, hy), THETA_HANDLE_RADIUS)

            # ── waypoint ring ──
            if is_sel:
                ring = C_WP_RING_SELECTED
            elif is_hov:
                ring = C_WP_RING_HOVER
            else:
                ring = C_WP_RING
            pygame.draw.circle(self.overlay, ring, (sx, sy), WP_RADIUS + 3)

            # ── waypoint dot ──
            dot = C_WP_SELECTED if is_sel else C_WP_NORMAL
            pygame.draw.circle(self.overlay, dot, (sx, sy), WP_RADIUS)

            # ── index label ──
            idx_s = self.font_small.render(str(i + 1), True, (255, 255, 255))
            self.overlay.blit(
                idx_s, (sx - idx_s.get_width() // 2, sy - idx_s.get_height() // 2)
            )

    def _draw_mouse_coords(self) -> None:
        """Show field coordinates at mouse position (top-left of field)."""
        if self.mouse_field is None:
            return
        fx, fy = self.mouse_field
        txt = f"({fx:.1f}, {fy:.1f})"
        surf = self.font_small.render(txt, True, C_MOUSE_COORD)
        bg = pygame.Surface(
            (surf.get_width() + 8, surf.get_height() + 4), pygame.SRCALPHA
        )
        bg.fill((0, 0, 0, 130))
        self.screen.blit(bg, (4, 4))
        self.screen.blit(surf, (8, 6))

    # ── side panel ──────────────────────────────────────────────────────
    def _draw_panel(self) -> None:
        px = FIELD_PX + 15
        pygame.draw.rect(self.screen, C_BG, (FIELD_PX, 0, PANEL_W, WIN_H))
        pygame.draw.line(self.screen, C_DIVIDER, (FIELD_PX, 0), (FIELD_PX, WIN_H), 2)

        y = 12
        self.screen.blit(self.font_title.render("PATH EDITOR", True, C_TITLE), (px, y))
        y += 28
        y = self._div(px, y)

        # ── path name ──
        y = self._sec(px, y, "PATH NAME")
        if self.naming_mode:
            txt = self.name_buffer + "\u2502"  # cursor
            col = (255, 255, 100)
        else:
            txt = self.path_name
            col = C_VALUE
        self.screen.blit(self.font_value.render(txt, True, col), (px + 4, y))
        y += 20
        y = self._div(px, y)

        # ── robot start ──
        y = self._sec(px, y, "ROBOT START")
        y = self._kv(px, y, "X", f"{self.start_x:.1f} in")
        y = self._kv(px, y, "Y", f"{self.start_y:.1f} in")
        y = self._kv(px, y, "Heading", f"{self.start_heading_deg:.1f}\u00b0")
        y += 4
        y = self._div(px, y)

        # ── waypoints list ──
        y = self._sec(px, y, f"WAYPOINTS ({len(self.waypoints)})")
        for i, wp in enumerate(self.waypoints):
            is_sel = i == self.selected_idx
            col = C_WP_SELECTED if is_sel else C_VALUE
            marker = "\u25b6 " if is_sel else "  "
            rx, ry, rt = self._field_to_robot_rel(wp.x, wp.y, wp.theta)
            line = f"{marker}{i + 1}: ({rx:.1f}, {ry:.1f}, {rt:.0f}\u00b0)"
            self.screen.blit(self.font_label.render(line, True, col), (px + 2, y))
            y += 16
            if y > WIN_H - 220:
                remaining = len(self.waypoints) - i - 1
                if remaining > 0:
                    self.screen.blit(
                        self.font_small.render(
                            f"  \u2026{remaining} more", True, C_LABEL
                        ),
                        (px + 4, y),
                    )
                    y += 16
                break
        y += 4
        y = self._div(px, y)

        # ── selected waypoint detail ──
        if self.selected_idx is not None and self.selected_idx < len(self.waypoints):
            wp = self.waypoints[self.selected_idx]
            y = self._sec(px, y, f"SELECTED  #{self.selected_idx + 1}")
            y = self._kv(px, y, "Field X", f"{wp.x:.1f} in")
            y = self._kv(px, y, "Field Y", f"{wp.y:.1f} in")
            y = self._kv(px, y, "Field \u03b8", f"{wp.theta:.1f}\u00b0")
            rx, ry, rt = self._field_to_robot_rel(wp.x, wp.y, wp.theta)
            y += 2
            y = self._kv(px, y, "Robot X", f"{rx:.1f} in")
            y = self._kv(px, y, "Robot Y", f"{ry:.1f} in")
            y = self._kv(px, y, "Robot \u03b8", f"{rt:.1f}\u00b0")
            y += 4
            y = self._div(px, y)

        # ── status ──
        if self._status_msg and self._status_frames > 0:
            self.screen.blit(
                self.font_label.render(self._status_msg, True, (100, 255, 100)),
                (px, y),
            )
            y += 20
            self._status_frames -= 1

        # ── controls help ──
        y = max(y, WIN_H - 170)
        y = self._div(px, y)
        y = self._sec(px, y, "CONTROLS")
        for h_text in (
            "Click field    add waypoint",
            "Drag point     move waypoint",
            "Drag handle    change theta",
            "Right-click    delete point",
            "N              rename path",
            "S              save / export",
            "C              clear all",
            "Del            delete selected",
            "Tab            cycle selection",
            "Esc            exit",
        ):
            self.screen.blit(self.font_help.render(h_text, True, C_LABEL), (px + 2, y))
            y += 13

        # ── footer ──
        fps = self.font_small.render(f"FPS: {self.clock.get_fps():.0f}", True, C_LABEL)
        self.screen.blit(fps, (px, WIN_H - 20))

    # panel helpers
    def _sec(self, x: int, y: int, title: str) -> int:
        self.screen.blit(self.font_section.render(title, True, C_SECTION), (x, y))
        return y + 20

    def _kv(self, x: int, y: int, key: str, val: str) -> int:
        self.screen.blit(self.font_label.render(key, True, C_LABEL), (x + 4, y))
        self.screen.blit(self.font_value.render(val, True, C_VALUE), (x + KV_OFFSET, y))
        return y + 17

    def _div(self, x: int, y: int) -> int:
        pygame.draw.line(self.screen, C_DIVIDER, (x, y), (x + PANEL_W - 30, y), 1)
        return y + 10

    # ════════════════════════════════════════════════════════════════════
    # Save / export
    # ════════════════════════════════════════════════════════════════════
    def _save_path(self) -> None:
        """Export waypoints as a ``CatmullRomPath`` C++ definition."""
        if len(self.waypoints) < 2:
            self._flash("Need at least 2 waypoints!", 180)
            return

        lines = [f"CatmullRomPath {self.path_name}({{"]
        for i, wp in enumerate(self.waypoints):
            rx, ry, rt = self._field_to_robot_rel(wp.x, wp.y, wp.theta)
            comma = "," if i < len(self.waypoints) - 1 else ""
            lines.append(f"    {{{rx:.1f}, {ry:.1f}, {rt:.0f}}}{comma}")
        lines.append("});")
        cpp_code = "\n".join(lines)

        out_dir = self.output_dir / "paths"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_file = out_dir / f"{self.path_name}.txt"
        out_file.write_text(cpp_code, encoding="utf-8")

        # Keep a faded copy on the field
        self.saved_paths.append((self.path_name, list(self.spline_field_pts)))

        self._flash(f"Saved \u2192 {out_file}")
        print(f"[path_editor] Saved '{self.path_name}' to {out_file}")
        print(cpp_code)

    # ════════════════════════════════════════════════════════════════════
    # Main loop
    # ════════════════════════════════════════════════════════════════════
    def run(self) -> None:
        """Enter the editor main loop (must be called from the main thread)."""
        while self.running:
            self._handle_events()

            # Clear overlay
            self.overlay.fill((0, 0, 0, 0))

            # Draw layers
            self._draw_field()
            self._draw_start_marker()
            self._draw_saved_paths()
            self._draw_spline()
            self._draw_waypoints()
            self.screen.blit(self.overlay, (0, 0))
            self._draw_mouse_coords()
            self._draw_panel()

            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()
