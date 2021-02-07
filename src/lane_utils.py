import carla
import numpy as np
import pygame
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
        self.lanes = []
        self.max_waypoint_dist = 50
        self.iteration_limit = 500
        self.max_lane_length = 100
        self.world2vehicle = None
        self.vehicle2camera = np.array(world.camera_manager.sensor_transform.get_inverse_matrix())
        self.projection_matrix = world.camera_manager.projection_matrix
        self.image_dim = world.camera_manager.hud.dim
    
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
            # Extend each lane to a set distance away from the vehicle
            if lane_dict['crossed']:
                lane_points = self.get_lane_points(lane_dict['waypoint'], -1.0)
            else:
                lane_points = self.get_lane_points(lane_dict['waypoint'], 1.0)
            
            self.get_marking_positions(lane_points, lane_dict['crossed'], side)
            
    def transform_to_camera(self, world_points):
        # Points in map coords of shape (3, num_points).
        world_points = np.array(world_points).T
        
        # Add an extra 1.0 at the end of each 3d point so it becomes of
        # shape (4, num_points) and it can be multiplied by a (4, 4) matrix.
        world_points = np.r_[world_points, [np.ones(world_points.shape[1])]]
        
        # Transform the points from map coords to vehicle coords.
        vehicle_points = np.dot(self.world2vehicle, world_points)
        
        # Transform the points from vehicle coords to camera coords.
        sensor_points = np.dot(self.vehicle2camera, vehicle_points)
        
        # New we change from camera coords to pixel coordinates
        # ^ z                       . z
        # |                        /
        # |              to:      +-------> x
        # | . x                   |
        # |/                      |
        # +-------> y             v y

        # This can be achieved by swapping:
        # (x, y ,z) -> (y, -z, x)
        point_in_camera_coords = np.array([
            sensor_points[1],
            sensor_points[2] * -1,
            sensor_points[0]])
            
        # Finally we can use our K matrix to do the actual 3D -> 2D.
        points_2d = np.dot(self.projection_matrix, point_in_camera_coords)

        # Remember to normalize the x, y values by the 3rd value.
        points_2d = np.array([
            points_2d[0, :] / points_2d[2, :],
            points_2d[1, :] / points_2d[2, :],
            points_2d[2, :]])

        # At this point, points_2d[0, :] contains all the x and points_2d[1, :]
        # contains all the y values of our points. In order to properly
        # visualize everything on a screen, the points that are out of the screen
        # must be discarted, the same with points behind the camera projection plane.
        points_2d = points_2d.T
        points_in_canvas_mask = \
            (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < self.image_dim[0]) & \
            (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < self.image_dim[1]) & \
            (points_2d[:, 2] > 0.0)
            
        points_2d = points_2d[points_in_canvas_mask]
        points_2d = points_2d[:, :2].astype(np.int)

        return points_2d
        
    def visualize_lanes(self, display, color=(255, 0, 0)):
        for lane in self.lanes:
            for point in lane:
                pygame.draw.circle(display, color, point, 3)

    def get_marking_positions(self, lane_points, crossed, side):
        lane = []
        for point in lane_points:
            lane_marking_type = point.left_lane_marking.type if side == side.LEFT else point.right_lane_marking.type
            lane_marking_color = point.left_lane_marking.color if side == side.LEFT else point.right_lane_marking.color
            # self.world.debug.draw_point(point.transform.location, size=0.1, color=carla.Color(r=0, g=0, b=255), life_time=0.1)
            if lane_marking_type == carla.libcarla.LaneMarkingType.NONE:
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
            self.world.debug.draw_point(end_point, size=0.1, color=carla.Color(r=0, g=255, b=0, a=255), life_time=0.1)
            lane.append([end_point.x, end_point.y, end_point.z])
        
        if lane:
            camera_coord_lane = self.transform_to_camera(lane)
            self.lanes.append(camera_coord_lane)

    def update(self):
        self.lanes = []
        self.world2vehicle = self.vehicle.get_transform().get_inverse_matrix()
        
        ego_location = self.vehicle.get_location()
        self.waypoint = self.map.get_waypoint(ego_location, project_to_road=True)

        self.get_adjacent_lanes(Side.LEFT)
        self.get_adjacent_lanes(Side.RIGHT)

        # Extend ego lane and get marking positions on both sides
        ego_lane_points = self.get_lane_points(self.waypoint, 1.0)
        
        # position of the current lane
        self.get_marking_positions(ego_lane_points, False, Side.LEFT)
        self.get_marking_positions(ego_lane_points, False, Side.RIGHT)

