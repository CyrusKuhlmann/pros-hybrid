"""Main entry point for VEX simulator."""

import asyncio
import sys

from vex_simulator.server import main


def run() -> None:
    """Run the simulator."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nSimulator stopped.")
        sys.exit(0)


if __name__ == "__main__":
    run()
