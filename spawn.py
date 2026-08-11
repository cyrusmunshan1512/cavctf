"""
spawn.py

Runs CARLA in synchronous mode with this script owning the clock.
Spawns one vehicle, steps the simulation by hand, prints its kinematics.
"""

import carla
import math
import time

def main():
    # Actors I create. The finally block destroys everything in here;
    # anything not in here leaks onto the map.
    actor_list = []

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Saved before touching anything. Sync mode is a SERVER setting and
    # persists after this script exits -- if I leave it on, the server sits
    # frozen waiting for a tick and the next run hangs on connect.
    original_settings = world.get_settings()

    # set_autopilot() routes the vehicle through the Traffic Manager, so TM
    # needs the same sync treatment as the world.
    traffic_manager = client.get_trafficmanager(8000)

    try:
        # get_settings() returns a copy; apply_settings() is what commits it.
        settings = world.get_settings()
        settings.synchronous_mode = True
        # 20 Hz. Fixed so each step is a known amount of SIM time regardless
        # of how long it takes in wall clock -- that's what makes runs
        # reproducible. Also divides evenly into the 10 Hz BSM rate.
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Second switch. Both or neither, or TM steps out of sync with the world.
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(234)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

        spawn_points = world.get_map().get_spawn_points()
        print(f'{len(spawn_points)} spawn points available')
        spawn_point = spawn_points[50]

        # try_spawn_actor returns None if the point is occupied, rather than
        # raising. Abort instead of continuing with a None handle.
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
            print(f'Spawn blocked at {spawn_point.location}. Aborting.')
            return

        actor_list.append(vehicle)
        vehicle.set_autopilot(True)

        # A fresh actor isn't fully simulated until a step has run. Without
        # this the first reads come back as zeros.
        world.tick()

        spectator = world.get_spectator()
        camera_transform = carla.Transform(
            carla.Location(x=spawn_point.location.x,
                           y=spawn_point.location.y,
                           z=spawn_point.location.z + 1.0),
            carla.Rotation(pitch=-15.0,
                           yaw=spawn_point.rotation.yaw,
                           roll=0.0))
        spectator.set_transform(camera_transform)

        # 400 ticks = 20 s of sim time. Print every 20th = once per sim second.
        for i in range(400):
            loop_start = time.perf_counter() 

            world.tick()

            if i % 20 == 0:
                location = vehicle.get_location()
                print(f'Location: {location}')

                snapshot = world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                print(f'Sim Time: {sim_time:.2f} s')

                # get_velocity() is a vector, not a speed. Collapse to a
                # magnitude in m/s.
                v = vehicle.get_velocity()
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                print(f'Speed: {speed:.2f} m/s')

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, 0.05 - elapsed))

    finally:
        # Hand the clock back BEFORE destroying anything -- destroy commands
        # may not be processed on a frozen server.
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':
    main()