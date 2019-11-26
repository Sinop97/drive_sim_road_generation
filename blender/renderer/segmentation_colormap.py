from commonroad.renderer.groundplane import ROADMARKING_TYPE_TO_VISUAL
from commonroad.renderer.traffic_sign import SIGN_MESHES

LANE_MARKING_SEGMENTATION_COLOR = (128, 0, 0)
BLOCKED_AREA_SEGMENTATION_COLOR = (0, 128, 0)
# pedestrian island currently not segmented, could do in the future
# PEDESTRIAN_ISLAND_COLOR = (0, 0, 128)
DRIVABLE_AREA_SEGMENTATION_COLOR = (0, 255, 0)
STOPLINE_SEGMENTATION_COLOR = (0, 255, 255)
STOPLINE_DASHED_SEGMENTATION_COLOR = (255, 255, 0)
ZEBRA_COLOR = (128, 128, 0)

BACKGROUND_COLOR = (0, 0, 0)

EGO_VEHICLE_COLOR = (100, 100, 100)

OBSTACLE_COLOR = (0, 0, 255)

RAMP_COLOR = (0, 255, 0)

TRAFFIC_MARKING_SEGMENTATION_COLORS = {marking: (0, 255, 255) for marking in ROADMARKING_TYPE_TO_VISUAL.keys()}

SIGN_BASE_COLOR = (200, 100, 0)

SIGN_TO_COLOR = {sign: (100, 0, 100) for sign in SIGN_MESHES.keys()}

INTERSECTION_COLOR = (64, 128, 255)


def convert_to_one_range(color):
    return (color[0]/255, color[1]/255, color[2]/255)

