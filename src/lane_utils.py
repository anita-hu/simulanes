import carla
import numpy as np
from enum import Enum


class Side(Enum):
    LEFT = 0
    RIGHT = 1
    BOTH = 2

class LaneExtractor(object):
    def __init__(self, world):
        self.vehicle = world.player
        self.map = world.map
        self.world = world.world
        self.waypoint = None
        self.max_waypoint_dist = 40
        self.iteration_limit = 500
        self.max_lane_length = 100
    
    def get_lane_points(self, waypoint, distance):
        lane = []
        curr_waypoint = waypoint
        for i in range(int(self.max_waypoint_dist/abs(distance))):
            if distance > 0:
                new_waypoint = curr_waypoint.next(distance)[0]
            else:
                new_waypoint = curr_waypoint.previous(abs(distance))[0]
            lane.append(new_waypoint)
            curr_waypoint = new_waypoint
        return lane

    def get_adjacent_lanes(self, side):
        if side == Side.LEFT:
            cur_waypoint = self.waypoint.get_left_lane()
        else:
            cur_waypoint = self.waypoint.get_right_lane()
        prev_lane_change = self.waypoint.lane_change
        crossed = False
        lanes = []
        i = 0
        while (cur_waypoint is not None and cur_waypoint.lane_type == carla.LaneType.Driving):
            lane_dict = {'waypoint': cur_waypoint}
            if (side == Side.LEFT and prev_lane_change == carla.LaneChange.Right) or (side == Side.RIGHT and prev_lane_change == carla.LaneChange.Left) or prev_lane_change == carla.LaneChange.NONE:
                crossed = True
            lane_dict['crossed'] = crossed
            lanes.append(lane_dict)
            prev_lane_change = cur_waypoint.lane_change 
            if (side == Side.LEFT and crossed) or (side == Side.RIGHT and not crossed):
                cur_waypoint = cur_waypoint.get_right_lane()
            else:
                cur_waypoint = cur_waypoint.get_left_lane()
            i += 1
            if i > self.iteration_limit:
                print("Loop iteration limit reached")
                break

        # Extend lanes and get marking positions on correct side
        for lane_dict in lanes:
            # self.world.debug.draw_point(lane_dict['waypoint'].transform.location, size=0.2, color=carla.Color(r=0, g=0, b=255), life_time=0.1)
            # Extend each lane until a junction is reached
            if lane_dict['crossed']:
                lane_points = self.get_lane_points(lane_dict['waypoint'], -1.0)
            else:
                lane_points = self.get_lane_points(lane_dict['waypoint'], 1.0)
            
            self.get_marking_positions(lane_points, lane_dict['crossed'], side)

    def get_marking_positions(self, lane_points, crossed, side):
        for point in lane_points:
            print(point.left_lane_marking.type, point.right_lane_marking.type)
            print(point.left_lane_marking.color, point.right_lane_marking.color)
            # self.world.debug.draw_point(point.transform.location, size=0.1, color=carla.Color(r=0, g=0, b=255), life_time=0.1)
            if (side == Side.LEFT and point.left_lane_marking.type == carla.libcarla.LaneMarkingType.NONE) or (side == Side.RIGHT and point.right_lane_marking.type == carla.libcarla.LaneMarkingType.NONE):
                continue
            # Display forward vector
            forward_vector = point.transform.get_forward_vector()
            if (side == Side.LEFT and crossed) or (side == Side.RIGHT and not crossed):
                # Rotate 90 deg CCW
                rotated_vector = carla.Vector3D(-forward_vector.y, forward_vector.x, 0)
            else:
                # Rotate 90 deg CW
                rotated_vector = carla.Vector3D(forward_vector.y, -forward_vector.x, 0)
            
            # Scale rotated vector to have a length equal to lane width / 2
            angle = np.arctan2(rotated_vector.y, rotated_vector.x)
            target_length = point.lane_width / 2
            new_vector = carla.Vector3D(target_length * np.cos(angle), target_length * np.sin(angle), 0)

            start_point = point.transform.location
            end_point = start_point + new_vector
            self.world.debug.draw_point(end_point, size=0.1, color=carla.Color(r=0, g=255, b=0), life_time=0.1)

    def update(self):
        ego_location = self.vehicle.get_location()
        self.waypoint = self.map.get_waypoint(ego_location, project_to_road=True)

        self.get_adjacent_lanes(Side.LEFT)
        self.get_adjacent_lanes(Side.RIGHT)

        # Extend ego lane and get marking positions on both sides
        ego_lane_points = self.get_lane_points(self.waypoint, 1.0)
        
        # position of the current lane
        self.get_marking_positions(ego_lane_points, False, Side.LEFT)
        self.get_marking_positions(ego_lane_points, False, Side.RIGHT)

