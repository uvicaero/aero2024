import asyncio
from mavsdk import System

async def run():
    print("Test2")
    drone = System()
    print("Test3")
    await drone.connect(system_address="udp://:14550")
    #await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Example action: Arm the drone
    print("Arming the drone...")
    await drone.action.arm()
    print("Drone armed.")

if __name__ == "__main__":
    print("Test1")
    asyncio.run(run())
