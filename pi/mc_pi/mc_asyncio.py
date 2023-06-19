import asyncio
import websockets
import json

# Get local machine name
DOCKER_SERVER_HOST = 'localhost'
CPU_SUB_SERVER_PORT = 9999


async def get_cpu_info(uri):
    async with websockets.connect(uri) as websocket:
        while True:
            try:
                message = await websocket.recv()
                print(f"MC: from CPU={message}")
            except websockets.ConnectionClosed:
                print("Connection with server closed")
                break

async def main():
    cpu_sub_uri = f"ws://{DOCKER_SERVER_HOST}:{CPU_SUB_SERVER_PORT}"

    print("connecting...")

    await asyncio.gather(
        get_cpu_info(cpu_sub_uri)
    )

if __name__ == "__main__":
    asyncio.run(main())
