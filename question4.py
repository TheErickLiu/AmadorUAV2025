import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (OffboardError, PositionNedYaw)  
from mavsdk.telemetry import Telemetry

async def check_health(drone):
    """
    Continuously checks the drone's health and telemetry status.
    Follows the safety checklist workflow.
    """
    while True:
        async for health in drone.telemetry.health():
            if not health.is_gyrometer_calibration_ok:
                print("Error: health.is_gyrometer_calibration_ok false\n")
                break
            if not health.is_accelerometer_calibration_ok:
                print("Error: health.is_accelerometer_calibration_ok false\n")
                break
            if not health.is_magnetometer_calibration_ok:
                print("Error: health.is_magnetometer_calibration_ok false\n")
                break
            if not health.is_local_position_ok:
                print("Error: health.is_local_position_ok false\n")
                break
            if not health.is_global_position_ok:
                print("Error: health.is_global_position_ok false\n")
                break
            if not health.is_home_position_ok:
                print("Error: health.is_home_position_ok false\n")
                break

            # Check health every 3 seconds
            print("Waiting for 3 seconds recheck ...\n")
            await asyncio.sleep(3)
            
        # Check if Telemetry is Lost
        try:
            async for status_text in drone.telemetry.status_text():
                print(f"telemetry.status_text: {status_text}")
                # Telemetry recovered
                break  
        except asyncio.TimeoutError:
            print("Error: Telemetry lost, waiting for 30 seconds to recheck ...\n")
            await asyncio.sleep(30)

            # Check if telemetry is still lost
            try:
                async for status_text in drone.telemetry.status_text():
                    print(f"telemetry.status_text: {status_text}")
                    # Telemetry recovered
                    break 
            except asyncio.TimeoutError:
                print("Error: Telemetry lost, switching to RTL mode and waiting for 3 minutes to recheck\n")
                await drone.action.return_to_launch()
                await asyncio.sleep(180)

                # Last check for telemetry
                try:
                    async for status_text in drone.telemetry.status_text():
                        print(f"telemetry.status_text: {status_text}")
                        # Telemetry recovered
                        break 
                except asyncio.TimeoutError:
                    print("Error: Telemetry lost, killing all motor operations ...")
                    await drone.action.kill()
                    return

async def run():
    """
    Runs the main logic, including the safety checks,
    mission execution, offboard control, payload drop, and landing.
    """
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to be connected ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone!\n")
            break

    # Health check task
    check_health_task = asyncio.create_task(check_health(drone))
  
    # Get initial position of the drone. (47.398, 8.546) with Gazebo's default.sdf.	
    async for position in drone.telemetry.position():
        initial_latitude = position.latitude_deg
        initial_longitude = position.longitude_deg
        initial_altitude = position.absolute_altitude_m
        print(f"Initial poistion: {initial_latitude}, {initial_longitude}, {initial_altitude}\n")
        break

    # Mission plan is mostly from MAVSDK-Python/examples/mission.py.
    # Use Gazebo's initial position as the last waypoint.
    mission_plan = MissionPlan(
        [
            MissionItem(47.398039859999997, 8.5455725400000002, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE),
            MissionItem(47.398036222362471, 8.5450146439425509, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE),
            MissionItem(47.397825620791885, 8.5450092830019059, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE),
            MissionItem(initial_latitude, initial_longitude, 25, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE),
        ]
    )
    await drone.mission.upload_mission(mission_plan)

    print("Arming ...\n")
    await drone.action.arm()

    print("Starting Mission ...\n")
    await drone.mission.start_mission()

    # Monitor mission progress
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")  
        if mission_progress.current == mission_progress.total:
            print("Mission complete!\n")
            break
    
    print("Sleeping 15 seconds before switching to offboard ...\n")
    await asyncio.sleep(15)


    # Set initial relative setpoint to the current position before switching to offboard mode
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard ...\n")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}, disarming ...\n")
        await drone.action.disarm()
        return  
    
    # Travel to a NED position
    await drone.offboard.set_position_ned(PositionNedYaw(10.0, 10.0, 2.0, 0)) # 
    await asyncio.sleep(15)

    # Drop the payload by opening and closing actuator 1.
    print("Opening actuator 1 ...\n")
    await drone.action.set_actuator(1, 0.9)
    # Adjust based on speed of the actuator and time taken to release the payload.
    await asyncio.sleep(3)
    print("Closing actuator 1 ...\n")
    await drone.action.set_actuator(1, 0.1)

    print("Return to the initial location ...\n")
    await drone.offboard.set_position_ned(PositionNedYaw(-10.0, -10.0, -1.0, 45.0))
    await asyncio.sleep(15)

    print("Landing ...\n")
    await drone.action.land()

    check_health_task.cancel()


if __name__ == "__main__":
    # MAVSDK is asynchronous in nature, e.g. drone.connect. 
    # asyncio.run() creates an event loop internally, runs
    # the coroutine and closes the loop at the end.
    asyncio.run(run())