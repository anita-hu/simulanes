import carla
import numpy as np
import pygame
import json
from scipy.interpolate import CubicSpline
import lane_config
from enum import Enum

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.collections import PatchCollection

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
        self.lane_ver_threshold = 0.7
        self.waypoint_dist = 2.0
        self.max_waypoint_range = 5000.0 # in cm?
        self.world2vehicle = None
        self.vehicle2camera = np.array(self.camera.sensor_transform.get_inverse_matrix())
        self.projection_matrix = self.camera.projection_matrix
        self.image_dim = self.camera.hud.dim
        self.lane_marking_types = [
            carla.LaneMarkingType.Broken,
            carla.LaneMarkingType.Solid,
            carla.LaneMarkingType.SolidSolid,
            carla.LaneMarkingType.SolidBroken,
            carla.LaneMarkingType.BrokenSolid,
            carla.LaneMarkingType.BrokenBroken,
            carla.LaneMarkingType.BottsDots,
            carla.LaneMarkingType.Curb
        ]

        # Get fov and position of camera
        sensors = self.camera.sensors
        rgb_cam_sensor = list(filter(lambda s: s[0] == 'sensor.camera.rgb', sensors))[0]
        rgb_cam_bp = rgb_cam_sensor[-1]
        self.rgb_cam_fov = rgb_cam_bp.get_attribute("fov").as_float()
        self.rgb_cam_tf = self.camera.sensor_transform

        # Get all waypoints in map
        map_waypoints = self.map.generate_waypoints(self.waypoint_dist)
        # print(self.map_waypoints[0].road_id) # 259
        # print(self.map_waypoints[0].lane_id) # 7
        self.waypoints_pos = [[], [], []]
        self.waypoints_road_id = []
        self.waypoints_lane_id = []
        for wp in map_waypoints:
            # Remove junction waypoints
            if wp.is_junction:
                continue
            # Remove waypoints with irrelevant lane markings
            if wp.right_lane_marking.type not in self.lane_marking_types and wp.left_lane_marking.type not in self.lane_marking_types:
                continue
            loc = wp.transform.location
            self.waypoints_pos[0].append(loc.x)
            self.waypoints_pos[1].append(loc.y)
            self.waypoints_pos[2].append(loc.z)
            self.waypoints_road_id.append(wp.road_id)
            self.waypoints_lane_id.append(wp.lane_id)
        self.waypoints_pos = np.array(self.waypoints_pos) # (3, N)
        self.waypoints_road_id = np.array(self.waypoints_road_id) # (N,)
        self.waypoints_lane_id = np.array(self.waypoints_lane_id) # (N,)
        # plt.figure(figsize=(12, 8), dpi=400)
        # plt.scatter(self.waypoints_pos[0], self.waypoints_pos[1], s=1, c=ids, cmap='gist_rainbow')
        # plt.savefig("/home/carla/PythonAPI/simulanes/debug_plots/road_id_0_60.png")
        # print("plot saved")

        

    def project_to_camera(self, world_points):
        """
        Project all world points to camera coordinates.

        world_points: (3, N) numpy array of points
        """
        if world_points is None:
            return world_points
        
        # Add an extra 1.0 at the end of each 3d point so it becomes of
        # shape (4, num_points) and it can be multiplied by a (4, 4) matrix.
        world_points = np.r_[world_points, [np.ones(world_points.shape[1])]]
        
        # Transform the points from map coords to vehicle coords.
        vehicle_points = np.dot(self.world2vehicle, world_points)
        
        # Transform the points from vehicle coords to camera coords.
        sensor_points = np.dot(self.vehicle2camera, vehicle_points)

        # Get mask for all points within the max range threshold
        points_in_range_mask = (sensor_points[0]**2 + sensor_points[1]**2 + sensor_points[2]**2 < self.max_waypoint_range)
        
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

        # Combine masks for points in canvas and points within max range
        cam_points_in_range_mask = (points_in_canvas_mask) & (points_in_range_mask)
            
        points_2d = points_2d[cam_points_in_range_mask]
        points_2d = points_2d[:, :2].astype(np.int)

        return points_2d, cam_points_in_range_mask

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
        lane_marking_type = carla.LaneMarkingType.NONE
        lane_marking_color = carla.LaneMarkingColor.Standard
        for point in lane_points:
            lane_marking_type = point.left_lane_marking.type if side == side.LEFT else point.right_lane_marking.type
            lane_marking_color = point.left_lane_marking.color if side == side.LEFT else point.right_lane_marking.color
            
            # check if waypoint is in juction
            if lane_marking_type == carla.libcarla.LaneMarkingType.NONE:
                if len(lane) == 0:
                    # vehicle in juction, will provide lanes after the juction
                    continue
                else:
                    # vehicle approaching juction, do not provide lanes after juction
                    lane_marking_type = lane_points[0].left_lane_marking.type if side == side.LEFT else lane_points[0].right_lane_marking.type
                    lane_marking_color = lane_points[0].left_lane_marking.color if side == side.LEFT else lane_points[0].right_lane_marking.color
                    break 
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
            # self.world.debug.draw_point(end_point, size=0.1, color=carla.Color(r=0, g=255, b=0, a=255), life_time=0.1)
            lane.append([end_point.x, end_point.y, end_point.z])
        
        camera_coord_lane = self.project_to_camera(lane)
        lane_xs, lane_class, verification_points = self.format_lane(camera_coord_lane, lane_marking_type, lane_marking_color)
        
        if len(lane_xs) >= self.min_lane_points:
            self.lanes.append({'xs': lane_xs, 'class': lane_class, 'ver_points': verification_points})

    def verify_lanes(self):
        """
        Verifies position of all solid lanes acquired from the waypoints (sometimes the waypoints don't match
        the lanes from the camera feed) by checking it against the segmentation mask.
        """
        # Get binary mask for road (128, 64, 128)
        road_mask = np.all(self.segmentation.numpy_image == (128, 64, 128), axis=-1)

        for lane in self.lanes:
            # Only check solid lanes
            if lane['class'] != 3:
                continue

            # Get mask values at each verification point
            mask_vals = road_mask[lane['ver_points'][0], lane['ver_points'][1]]

            # Get ratio of False points to total points
            ratio = 1 - np.count_nonzero(mask_vals) / mask_vals.shape[0]

            # Don't save this example if a lane is off
            if ratio < self.lane_ver_threshold:
                print(f"Lane found with ratio {ratio} < {self.lane_ver_threshold}. Skipping this frame.")
                return False

        return True
            
    def save_lanes(self, image_path):
        label_dict = {'lanes': [], 'classes': []}
        label_dict['h_sample'] = lane_config.row_anchors
        
        for lane_dict in self.lanes:
            label_dict['lanes'].append(lane_dict['xs'])
            label_dict['classes'].append(lane_dict['class'])
         
        with open(image_path.replace('jpg', 'json'), 'w') as label_file:
            json.dump(label_dict, label_file)

    def visualize_lanes(self, display):
        unique_ids = np.unique(np.hstack((self.cam_road_ids[:, np.newaxis], self.cam_lane_ids[:, np.newaxis])), axis=0)
        hues = np.linspace(0, 360, unique_ids.shape[0])[:, np.newaxis]
        id_hues = {}
        for i in range(unique_ids.shape[0]):
            id_hues[tuple(unique_ids[i])] = hues[i]
        
        for n in range(self.cam_pts.shape[0]):
            ID = (self.cam_road_ids[n], self.cam_lane_ids[n])
            hue = id_hues[ID]

            color = pygame.Color(0, 0, 0)
            color.hsla = (hue, 95, 55, 100)

            pygame.draw.circle(display, color, list(self.cam_pts[n]), 3)


    def update(self):
        # Convert all points to camera coords and remove ones outside of camera view
        self.world2vehicle = np.array(self.vehicle.get_transform().get_inverse_matrix())
        self.cam_pts, cam_pts_mask = self.project_to_camera(self.waypoints_pos)

        # Get road and lane IDs for all camera points
        self.cam_road_ids = self.waypoints_road_id[cam_pts_mask] # (M,)
        self.cam_lane_ids = self.waypoints_lane_id[cam_pts_mask] # (M,)    
