"""Main entry point for VEX simulator."""

from __future__ import annotations

import asyncio
import sys
import threading

from vex_simulator.server import SimulatorServer
from vex_simulator.visualizer import Visualizer


def run() -> None:
    """Run the simulator with the pygame visual display."""
    server = SimulatorServer()

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
        viz = Visualizer(server.robot)
        viz.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.running = False
        print("\nSimulator stopped.")
        sys.exit(0)


if __name__ == "__main__":
    run()
