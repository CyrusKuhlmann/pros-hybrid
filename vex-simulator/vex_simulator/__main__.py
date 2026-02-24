"""Main entry point for VEX simulator.

Usage::

    python -m vex_simulator           # run the simulator
    python -m vex_simulator editor    # open the path editor
"""

from __future__ import annotations

import asyncio
import sys
import threading
from pathlib import Path

from vex_simulator.config import load_config
from vex_simulator.path_loader import parse_paths_from_directory
from vex_simulator.server import SimulatorServer
from vex_simulator.visualizer import Visualizer

# Root of the C++ project – all .cpp/.h files under here are scanned for paths
_PROS_SIM_ROOT = Path(__file__).resolve().parents[2] / "pros-sim"

# Output directory for saved paths (project root)
_PROJECT_ROOT = Path(__file__).resolve().parents[1]


def run_editor() -> None:
    """Launch the interactive path editor."""
    from vex_simulator.path_editor import PathEditor

    config = load_config()
    print(
        f"[editor] Start pose: ({config.start_x_in}, {config.start_y_in}), "
        f"heading {config.start_heading_deg}°"
    )

    editor = PathEditor(
        start_x=config.start_x_in,
        start_y=config.start_y_in,
        start_heading_deg=config.start_heading_deg,
        output_dir=_PROJECT_ROOT,
    )
    editor.run()


def run() -> None:
    """Run the simulator with the pygame visual display."""
    config = load_config()  # reads robot_config.toml from cwd
    print(
        f'Loaded config: {config.wheel_diameter_in}" wheels, '
        f'{config.track_width_in}" track, {config.motor_max_rpm} RPM, '
        f'{config.robot_size_in}" chassis'
    )

    # Load spline paths from all C++ sources in the project
    paths = parse_paths_from_directory(_PROS_SIM_ROOT)

    server = SimulatorServer(config=config)

    # Start the async TCP server in a daemon thread
    def _serve() -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(server.run())
        finally:
            loop.close()

    srv_thread = threading.Thread(target=_serve, daemon=True)
    srv_thread.start()

    # Run the visualiser on the main thread (pygame requirement)
    try:
        viz = Visualizer(server.robot, paths=paths)
        viz.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.running = False
        print("\nSimulator stopped.")
        sys.exit(0)


def main() -> None:
    """Dispatch to simulator or editor based on CLI arguments."""
    if len(sys.argv) > 1 and sys.argv[1] == "editor":
        run_editor()
    else:
        run()


if __name__ == "__main__":
    main()
