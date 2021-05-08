import sys


def get_gt_lane_count(town_name, road_id):
    town = getattr(sys.modules[__name__], town_name)
    for count, road_ids in town.lane_count.items():
        if road_id in road_ids:
            return count
    raise ValueError(f"Ground truth lane count for {road_id} not found in {town_name}")


class Town05:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [19, 20, 48],
        7: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
            32, 33, 39, 40, 41, 42, 43, 44, 45, 46, 47, 49, 50, 51, 52],
        10: [12, 34, 35, 36, 37, 38]
    }
