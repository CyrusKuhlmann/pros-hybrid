"""TCP server for client-simulator communication."""

from __future__ import annotations

import asyncio
import json
from typing import TYPE_CHECKING

from vex_simulator.config import RobotConfig, load_config
from vex_simulator.protocol import CommandMessage, MessageType
from vex_simulator.robot_state import RobotState

if TYPE_CHECKING:
    pass


class SimulatorServer:
    """TCP server that handles client connections and simulation loop."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9090,
        config: RobotConfig | None = None,
    ) -> None:
        """Initialize server."""
        self.host = host
        self.port = port
        self.robot = RobotState(config)
        self.clients: list[tuple[asyncio.StreamReader, asyncio.StreamWriter]] = []
        self.running = False

    async def handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        """Handle a client connection."""
        addr = writer.get_extra_info("peername")
        print(f"Client connected: {addr}")

        # Send handshake
        handshake = {"type": MessageType.HANDSHAKE.value, "version": "0.1.0"}
        writer.write((json.dumps(handshake) + "\n").encode())
        await writer.drain()

        # Add to client list
        self.clients.append((reader, writer))

        try:
            while self.running:
                # Read command with timeout
                try:
                    line = await asyncio.wait_for(reader.readline(), timeout=0.1)
                except TimeoutError:
                    continue

                if not line:
                    break

                try:
                    data = json.loads(line.decode().strip())
                    cmd = CommandMessage.from_dict(data)
                    self.robot.handle_command(cmd)
                except json.JSONDecodeError as e:
                    print(f"Invalid JSON from client: {e}")
                except (KeyError, ValueError) as e:
                    print(f"Invalid command from client: {e}")

        except ConnectionResetError:
            print(f"Client disconnected: {addr}")
        finally:
            self.clients.remove((reader, writer))
            writer.close()
            await writer.wait_closed()

    async def broadcast_state(self) -> None:
        """Broadcast state to all connected clients at 50Hz."""
        while self.running:
            # Update physics
            dt = 0.02  # 50Hz = 20ms
            self.robot.update(dt)

            # Get state
            state = self.robot.get_state_update()
            message = json.dumps(state.to_dict()) + "\n"

            # Send to all clients (non-blocking so a slow client
            # never stalls the physics loop)
            encoded = message.encode()
            for _, writer in list(self.clients):
                try:
                    writer.write(encoded)
                except (ConnectionResetError, BrokenPipeError, OSError):
                    pass  # Will be cleaned up in handle_client

            await asyncio.sleep(dt)

    async def run(self) -> None:
        """Start the server and simulation loop."""
        self.running = True

        server = await asyncio.start_server(self.handle_client, self.host, self.port)
        addr = (
            server.sockets[0].getsockname()
            if server.sockets
            else (self.host, self.port)
        )
        print(f"Simulator running on {addr[0]}:{addr[1]}")
        print("Waiting for client connections...")

        # Start state broadcast task
        broadcast_task = asyncio.create_task(self.broadcast_state())

        async with server:
            try:
                await server.serve_forever()
            except KeyboardInterrupt:
                print("\nShutting down...")
            finally:
                self.running = False
                broadcast_task.cancel()
                try:
                    await broadcast_task
                except asyncio.CancelledError:
                    pass


async def main() -> None:
    """Main entry point."""
    config = load_config()
    server = SimulatorServer(config=config)
    await server.run()


if __name__ == "__main__":
    asyncio.run(main())
