import sys

available_town_info = ['Town01', 'Town03', 'Town04', 'Town05', 'Town07', 'Town10', 'Town10HD']


def get_town_info(town_name):
    if town_name == 'Town10HD':
        town_name = 'Town10'
    return getattr(sys.modules[__name__], town_name)


def get_gt_lane_count(town_name, road_id):
    town = get_town_info(town_name)
    for count, road_ids in town.lane_count.items():
        if road_id in road_ids:
            return count
    return -1


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
    bad_road_ids = []


'''
Town 02 is skipped due to lane misalignment issues throughout the map
'''


class Town03:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [59, 62],
        7: [17, 18, 19, 20, 21, 22, 23, 41, 42, 43, 74, 75],
        8: [0, 1, 2, 3, 4, 5, 6, 7, 8, 65, 66, 67, 68, 69],
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 24, 25, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 44, 46,
                    47, 48, 49, 50, 51, 52, 60, 61, 65, 66, 73, 75, 76, 77, 78, 79, 80]


class Town04:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        2: [31, 33, 34, 44],
        3: [5, 14, 22, 23, 25],
        4: [1, 7, 8, 9, 18, 19, 20, 24, 32],
        5: [10, 11, 15, 16, 17, 26, 27, 28, 29, 30],
        10: [6, 35, 36, 38, 39, 40, 41, 45, 46, 47, 48, 49, 50]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [0, 2, 3, 4, 12, 13, 37, 42, 43, 51, 52]
    # missing: 21


class Town05:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [19, 20, 48],
        7: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
            32, 33, 39, 40, 41, 42, 43, 44, 45, 46, 47, 49, 50, 51, 52],
        10: [12, 34, 35, 36, 37, 38]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [7, 8, 19, 20, 22, 23, 48]


'''
Town 06 is skipped due to missing left curb throughout the map and lane class issues
'''


class Town07:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        0: [0, 1, 36, 37, 46, 47],
        2: [6, 9, 10, 31, 32, 40, 45, 49, 50],
        3: [7, 11, 20, 21, 34, 39, 41, 42, 43, 44, 52, 57, 58, 59, 60, 61, 62],
        4: [3, 12, 13, 14, 17, 23, 24, 25, 29, 38, 55, 56],
        5: [15]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [4, 5, 8, 16, 18, 25, 26, 27, 28, 33, 35, 51, 53]
    # missing: 2, 19, 22, 30, 48, 54


class Town10:
    # lane count (including curbs from both sides of the road): road ids
    lane_count = {
        5: [12],
        7: [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 13, 14, 15, 16, 17, 22],
        8: [18, 19, 20, 21]
    }
    # road ids where there are errors in the lanes i.e misalignment or missing lane
    bad_road_ids = [4, 9, 11, 18, 19, 20, 21]
