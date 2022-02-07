# Modified work Copyright (c) 2021 Anita Hu.
# Original work Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Original source: https://github.com/carla-simulator/carla/blob/0.9.11/PythonAPI/examples/automatic_control.py

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
