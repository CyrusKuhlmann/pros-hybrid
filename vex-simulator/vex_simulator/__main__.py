"""Main entry point for VEX simulator."""

from __future__ import annotations

import asyncio
import sys
import threading
from pathlib import Path

from vex_simulator.config import load_config
from vex_simulator.path_loader import parse_paths_from_cpp
from vex_simulator.server import SimulatorServer
from vex_simulator.visualizer import Visualizer

# Relative path from this package's directory to the C++ main.cpp
_PROS_SIM_MAIN = Path(__file__).resolve().parents[2] / "pros-sim" / "src" / "main.cpp"


def run() -> None:
    """Run the simulator with the pygame visual display."""
    config = load_config()  # reads robot_config.toml from cwd
    print(
        f'Loaded config: {config.wheel_diameter_in}" wheels, '
        f'{config.track_width_in}" track, {config.motor_max_rpm} RPM, '
        f'{config.robot_size_in}" chassis'
    )

    # Load spline paths from the C++ source
    paths = parse_paths_from_cpp(_PROS_SIM_MAIN)

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


if __name__ == "__main__":
    run()
