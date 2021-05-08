import sys

available_towns = ['Town01', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07', 'Town10']


def get_town_info(town_name):
    if town_name == 'Town10HD':
        town_name = 'Town10'
    return getattr(sys.modules[__name__], town_name)


def get_gt_lane_count(town_name, road_id):
    town = get_town_info(town_name)
    for count, road_ids in town.lane_count.items():
        if road_id in road_ids:
            return count
    raise ValueError(f"Ground truth lane count for {road_id} not found in {town_name}")


def is_bad_road_id(town_name, road_id):
    town = get_town_info(town_name)
    if road_id in town.bad_road_ids:
        return True
    return False


class Town01:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        3: [i for i in range(26)]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO


'''
Town 02 is skipped due to lane misalignment issues throughout the map
'''


class Town03:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [59, 61, 62],
        7: [17, 18, 19, 20, 21, 22, 23, 41, 42, 43, 73, 74, 75],
        8: [0, 1, 2, 3, 4, 5, 6, 7, 8, 65, 66, 67, 68, 69],
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 24, 25, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 44, 46,
                    47, 48, 49, 50, 51, 52, 60, 65, 75, 76, 77, 78, 79, 80]


class Town04:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        # TODO
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO


class Town05:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [19, 20, 48],
        7: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
            32, 33, 39, 40, 41, 42, 43, 44, 45, 46, 47, 49, 50, 51, 52],
        10: [12, 34, 35, 36, 37, 38]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO


class Town06:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        # TODO
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO


class Town07:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        # TODO
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO


class Town10:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        # TODO
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = []  # TODO
