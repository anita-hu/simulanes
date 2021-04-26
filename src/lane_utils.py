import carla
import numpy as np
import pygame
import json
from scipy.interpolate import CubicSpline
import lane_config
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
        self.camera = world.camera_manager
        self.segmentation = world.segmentation_manager
        self.waypoint = None
        self.lanes = []
        self.max_waypoint_dist = 50
        self.max_adjacent_lanes = 2
        self.max_lane_length = 100
        self.min_lane_points = 2
        self.lane_min_y_diff = 2
        self.save_image = True
        self.world2vehicle = None
        self.vehicle2camera = np.array(self.camera.sensor_transform.get_inverse_matrix())
        self.projection_matrix = self.camera.projection_matrix
        self.image_dim = self.camera.hud.dim
    
    def get_lane_points(self, waypoint, distance):
        lane = []
        curr_waypoint = waypoint
        for i in range(int(self.max_waypoint_dist/abs(distance))):
            if distance > 0:
                new_waypoint = curr_waypoint.next(distance)
            else:
                new_waypoint = curr_waypoint.previous(abs(distance))
            if new_waypoint:
                lane.append(new_waypoint[0])
                curr_waypoint = new_waypoint[0]
            else:
                break
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
            if i >= self.max_adjacent_lanes:
                break
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
                
        while cur_waypoint is not None:
            if i >= self.max_adjacent_lanes:
                break
            lanes.append({'waypoint': cur_waypoint, 'crossed': crossed})
            if (side == Side.LEFT) or (side == Side.RIGHT):
                cur_waypoint = cur_waypoint.get_right_lane()
            else:
                cur_waypoint = cur_waypoint.get_left_lane()
            i += 1
            
        # Extend lanes and get marking positions on correct side
        for lane_dict in lanes:
            # Extend each lane to a set distance away from the vehicle
            if lane_dict['crossed']:
                lane_points = self.get_lane_points(lane_dict['waypoint'], -1.0)
            else:
                lane_points = self.get_lane_points(lane_dict['waypoint'], 1.0)
            
            self.get_marking_positions(lane_points, lane_dict['crossed'], side)
            
    def project_to_camera(self, world_points):
        if not world_points:
            return world_points
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
        
    def visualize_lanes(self, display):        
        hue_step = 360 // len(lane_config.lane_classes)
        for lane_dict in self.lanes:
            lane_xs = lane_dict['xs']
            lane_ys = lane_config.row_anchors
            lane = np.stack([lane_xs, lane_ys], axis=1)
            lane_type = lane_config.lane_classes[lane_dict['class']]
            color = pygame.Color(0, 0, 0)
            color.hsla = (hue_step*lane_dict['class'], 100, 50, 50)
            pygame.draw.circle(display, color, lane[0], 3)
            for i in range(1, len(lane)):
                if lane_xs[i] != -2:
                    pygame.draw.circle(display, color, lane[i], 3)
                    if lane_xs[i-1] != -2:
                        pygame.draw.line(display, color, lane[i-1], lane[i], 2)

    def format_lane(self, lane, lane_type, lane_color):
        lane_class = 0
        
        if len(lane) < 2:
            return lane, lane_class, None
            
        filtered = [lane[0]]
        for point in lane[1:]:
            y_diff = filtered[-1][1] - point[1]
            if y_diff > self.lane_min_y_diff:
                filtered.append(point)

        if len(filtered) < 2:
            return [lane[0]], lane_class, None

        # Interpolate points to predefined anchors for labels
        points = np.array(filtered)
        points = points[points[:, 1].argsort()]
        min_y, max_y = points[0][1], points[-1][1]
        cs = CubicSpline(points[:, 1], points[:, 0])
        ys = np.array(lane_config.row_anchors)
        lane_xs = cs(ys).astype(int)
        lane_xs[ys < min_y] = -2
        lane_xs[ys > max_y] = -2

        # Interpolate points for lane verification
        ys = np.arange(int(min_y), int(max_y)+1)
        xs = cs(ys).astype(int)
        verification_points = np.vstack((ys[np.newaxis, :], xs[np.newaxis, :]))

        if lane_type == carla.LaneMarkingType.Broken:
            lane_class = 1
        elif lane_type == carla.LaneMarkingType.Solid:
            lane_class = 3
        elif lane_type == carla.LaneMarkingType.BrokenBroken:
            lane_class = 5
        elif lane_type == carla.LaneMarkingType.SolidSolid:
            lane_class = 7
        elif lane_type == carla.LaneMarkingType.SolidBroken:
            lane_class = 9
        elif lane_type == carla.LaneMarkingType.BrokenSolid:
            lane_class = 11
        elif lane_type == carla.LaneMarkingType.BottsDots:
            lane_class = 13
        elif lane_type == carla.LaneMarkingType.Curb:
            lane_class = 14
        
        if lane_color == carla.LaneMarkingColor.Yellow:
            lane_class += 1
                
        return lane_xs.tolist(), lane_class, verification_points

    def get_marking_positions(self, lane_points, crossed, side):
        lane = []
        prev_lane_marking_type = lane_marking_type = carla.LaneMarkingType.NONE
        prev_lane_marking_color = lane_marking_color = carla.LaneMarkingColor.Standard
        for point in lane_points:
            # Display forward vector
            forward_vector = point.transform.get_forward_vector()
            prev_lane_marking_type = lane_marking_type
            prev_lane_marking_color = lane_marking_color
            if (side == Side.LEFT and crossed) or (side == Side.RIGHT and not crossed):
                # Rotate 90 deg CCW
                rotated_vector = carla.Vector3D(-forward_vector.y, forward_vector.x, 0)
                lane_marking_type = point.right_lane_marking.type
                lane_marking_color = point.right_lane_marking.color
            else:
                # Rotate 90 deg CW
                rotated_vector = carla.Vector3D(forward_vector.y, -forward_vector.x, 0)
                lane_marking_type = point.left_lane_marking.type
                lane_marking_color = point.left_lane_marking.color
                
            # check if waypoint is in juction
            if lane_marking_type == carla.LaneMarkingType.NONE:
                if len(lane) == 0:
                    # vehicle in juction, will provide lanes after the juction
                    continue
                else:
                    # vehicle approaching juction, do not provide lanes after juction
                    lane_marking_type = prev_lane_marking_type
                    lane_marking_color = prev_lane_marking_color
                    break
            
            # Scale rotated vector to have a length equal to lane width / 2
            angle = np.arctan2(rotated_vector.y, rotated_vector.x)
            target_length = point.lane_width / 2
            new_vector = carla.Vector3D(target_length * np.cos(angle), target_length * np.sin(angle), 0)

            start_point = point.transform.location
            end_point = start_point + new_vector
            # self.world.debug.draw_point(end_point, size=0.1, color=carla.Color(r=0, g=255, b=0, a=255), life_time=0.1)
            lane.append([end_point.x, end_point.y, end_point.z])
        
        camera_coord_lane = self.project_to_camera(lane)
        lane_xs, lane_class, verification_points = self.format_lane(camera_coord_lane, lane_marking_type, lane_marking_color)
        
        if len(lane_xs) >= self.min_lane_points:
            self.lanes.append({'xs': lane_xs, 'class': lane_class, 'ver_points': verification_points})
        
    def remove_occluded_lanes(self):
        # Get binary mask for road (128, 64, 128), lane (157, 234, 50), sidewalk (244, 35, 232)
        road_mask = np.all(self.segmentation.numpy_image == (128, 64, 128), axis=-1)
        lane_mask = np.all(self.segmentation.numpy_image == (157, 234, 50), axis=-1)
        sidewalk_mask = np.all(self.segmentation.numpy_image == (244, 35, 232), axis=-1)
        combined_mask = road_mask | lane_mask | sidewalk_mask
        
        non_occluded_lanes = []
        occluded = 0

        for lane in self.lanes:
            mask_vals = combined_mask[lane['ver_points'][0], lane['ver_points'][1]]
            occlusion = 1 - np.count_nonzero(mask_vals) / mask_vals.shape[0]

            if occlusion > 0.9:
                occluded += 1
            else:
                non_occluded_lanes.append(lane)

        if occluded > 1:
            self.lanes = non_occluded_lanes
            print(f"Found {occluded} occluded lanes")

    def save_lanes(self, image_path):
        label_dict = {'lanes': [], 'classes': []}
        label_dict['h_sample'] = lane_config.row_anchors
        
        for lane_dict in self.lanes:
            label_dict['lanes'].append(lane_dict['xs'])
            label_dict['classes'].append(lane_dict['class'])
         
        with open(image_path.replace('jpg', 'json'), 'w') as label_file:
            json.dump(label_dict, label_file)
        
    def update(self):
        self.lanes = []
        self.world2vehicle = self.vehicle.get_transform().get_inverse_matrix()
        
        ego_location = self.vehicle.get_location()
        self.waypoint = self.map.get_waypoint(ego_location, project_to_road=True)

        self.get_adjacent_lanes(Side.LEFT)
        self.get_adjacent_lanes(Side.RIGHT)

        # Extend ego lane and get marking positions on both sides
        ego_lane_points = self.get_lane_points(self.waypoint, 1.0)
        
        # Position of the current lane
        self.get_marking_positions(ego_lane_points, False, Side.LEFT)
        self.get_marking_positions(ego_lane_points, False, Side.RIGHT)
        
        # Remove fully occluded lanes
        self.remove_occluded_lanes()
        
        if not self.save_image:
            return

        # Save image and lane data
        if not self.waypoint.is_junction and self.camera.latest_image.frame % int(self.camera.hud.server_fps) == 0:
            image_path = self.camera.save_frame()
            if image_path is not None:
                self.save_lanes(image_path)

