for i, sp in enumerate(spawn_points):
    v = world.try_spawn_actor(vehicle_bp, sp)
    if v is not None:
        print(f'index {i} OK')
        v.destroy()
    if i > 20:
        break