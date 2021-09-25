import re
import carla
import random


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def get_different_spawn_point(world, player):
    spawn_points = world.map.get_spawn_points()
    random.shuffle(spawn_points)

    if spawn_points[0].location != player.get_location():
        return spawn_points, spawn_points[0]
    else:
        return spawn_points, spawn_points[1]
