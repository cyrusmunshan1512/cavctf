"""
spawn.py

Runs CARLA in synchronous mode with this script owning the clock.
Spawns a victim and a lead car 40 m ahead in the SAME lane, steps the
simulation by hand, prints the victim's kinematics.
"""

import carla
import math
import time


def find_valid_pair(world, carla_map, spawn_points, victim_bp, lead_bp, gap):
    """Try spawn points until one yields a victim + lead pair in the same lane.
    Returns (victim, lead) or (None, None)."""

    for index, victim_transform in enumerate(spawn_points):

        # Snap this spawn point onto the lane centerline, then walk forward
        # along that same lane to find where the lead car goes.
        victim_wp = carla_map.get_waypoint(victim_transform.location)

        lead_wp_list = victim_wp.next(gap)
        if not lead_wp_list:
            # Lane ends before the gap. Not fatal -- try the next point.
            continue

        lead_transform = lead_wp_list[0].transform
        # Waypoints sit ON the road surface. CARLA's own spawn points sit
        # ~0.6 m up; without the lift the spawn collides with the road.
        lead_transform.location.z += 0.6

        victim = world.try_spawn_actor(victim_bp, victim_transform)
        if victim is None:
            continue

        lead = world.try_spawn_actor(lead_bp, lead_transform)
        if lead is None:
            # The victim spawned but its partner didn't. Destroy it before
            # moving on, or every failed attempt leaves a car on the map
            # that blocks later attempts.
            victim.destroy()
            continue

        print(f'Using spawn point {index}, gap {gap} m')
        return victim, lead

    return None, None


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
        victim_bp = blueprint_library.find('vehicle.tesla.model3')
        lead_bp = blueprint_library.find('vehicle.audi.tt')

        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()
        print(f'{len(spawn_points)} spawn points available')

        victim, lead = find_valid_pair(world, carla_map, spawn_points,
                                       victim_bp, lead_bp, 40.0)
        if victim is None:
            print('No spawn point has a clear 40 m lane ahead. Aborting.')
            return

        actor_list.append(victim)
        actor_list.append(lead)

        # No autopilot on either car. Autopilot means the Traffic Manager
        # drives, and it has its own collision avoidance -- it would brake
        # for the lead car on its own and I could never show that MY attack
        # caused the braking.

        # A fresh actor isn't fully simulated until a step has run. Without
        # this the first reads come back as zeros.
        world.tick()

        gap = victim.get_location().distance(lead.get_location())
        print(f'Gap between cars: {gap:.1f} m')

        # Top-down over the midpoint so both cars are in frame.
        victim_loc = victim.get_location()
        lead_loc = lead.get_location()
        spectator = world.get_spectator()
        camera_transform = carla.Transform(
            carla.Location(x=(victim_loc.x + lead_loc.x) / 2,
                           y=(victim_loc.y + lead_loc.y) / 2,
                           z=40.0),
            carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0))
        spectator.set_transform(camera_transform)

        # 400 ticks = 20 s of sim time. Print every 20th = once per sim second.
        for i in range(400):
            loop_start = time.perf_counter()

            world.tick()

            if i % 20 == 0:
                location = victim.get_location()
                print(f'Location: {location}')

                snapshot = world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                print(f'Sim Time: {sim_time:.2f} s')

                # get_velocity() is a vector, not a speed. Collapse to a
                # magnitude in m/s.
                v = victim.get_velocity()
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                print(f'Speed: {speed:.2f} m/s')

            # Pace the loop so one tick takes 0.05 s of REAL time too.
            # Sim time is unaffected -- this only changes how fast I watch it.
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