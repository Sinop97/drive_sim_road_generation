#!/usr/bin/env python3
import sys
import os
import shutil
import json
# because blender does not want to have it otherwise -__-
sys.path.append(os.getcwd())
sys.path += ['/usr/local/lib/python3.5/dist-packages', '/usr/lib/python3/dist-packages',
             '/usr/lib/python3.5/dist-packages']
from blender.renderer.blender import generate_blend

OUTPUT_DIR = 'blender-output-new'  # output directory
FORCE_OUTPUT = True  # overwrite output if True
ADD_VEHICLE = True  # render ego-vehicle in the frames
INPUT_FILE = 'driving-scenario.xml'  # input CommonRoad file
GAZEBO_WORLDS_PATH = '../drive_gazebo_worlds'  # location of the drive_gazebo_worlds package
GAZEBO_SIM_PATH = '../drive_gazebo_sim'  # location of the drive_gazebo_sim package


# blender does not less us parse arguments
config = {'render_interval_distance': 0.05,
          'groundplane_shader_type': 'ShaderNodeBsdfGlossy',
          'env_config': 'subway_entrance',
          'texture_padding_ratio': 1.0,
          # choose camera to render from, available: realsense, top
          # available: RGB (cycles render image), semseg_color (semantic segmentation colormap)
          # instances (id map of traffic signs (including poles)), lanes (DRIVABLE lane segmentation, only left/right)
          'render_passes': ['rgb', 'semseg_color', 'instances', 'lanes'],
          # resolution provided separately for each camera
          'frame_range': (16, -1),
          # use a .png to render the vehicle -> has to be re-generated for each camera position
          'use_vehicle_mask': True,
          'cameras': [{'name': 'top',
                       'position_offset': {'x': -0.126113, 'y': 0, 'z': 0.231409},
                       # rotation of the cameras around the Y axis (lateral car axis) in degrees
                       'rotation': 31,
                       'image_resolution': (2048, 1536),
                       # used if camera_mask is set to True
                       'segmentation_mask': 'top_segmentation_mask.png',
                       'sensor_width': 7.18,  # 1/1.8 inch on IDS camera
                       'sensor_height': 5.32,  # 1/1.8 inch on IDS camera
                       'focal_length': 1.7,  # 1.7 mm on Theia
                       },
                      # http://robotsforroboticists.com/wordpress/wp-content/uploads/2019/09/realsense-sep-2019.pdf
                      {'name': 'realsense',
                       'position_offset': {'x': -0.220317, 'y': 0.0325, 'z': 0.11},
                       # rotation of the cameras around the Y axis (lateral car axis) in degrees
                       'rotation': 0,
                       'image_resolution': (1280, 960),
                       # used if camera_mask is set to True
                       'segmentation_mask': 'realsense_segmentation_mask.png',
                       'sensor_width': 6.4,  # 1/2 inch, see above for source
                       'sensor_height': 4.8,  # 1/2 inch, see above for source
                       # 'focal_length': 1.93,  -> documentation above incorrect, use horizontal FOV
                       'horizontal_fov': 69.4  # unit: degrees
                       }
                      ]
          }


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if os.listdir(OUTPUT_DIR) != [] and not FORCE_OUTPUT:
        print("Output directory is not empty.")
        print("Use --force")
        sys.exit(1)

    shutil.copy2(INPUT_FILE, OUTPUT_DIR)

    with open(os.path.join(OUTPUT_DIR, 'config.json'), 'w', encoding='utf-8') as config_file:
        json.dump(config, config_file, ensure_ascii=False, indent=4)

    with open(INPUT_FILE) as input_file:
        xml = input_file.read()

    generate_blend(xml, OUTPUT_DIR, ADD_VEHICLE, OUTPUT_DIR, GAZEBO_WORLDS_PATH, GAZEBO_SIM_PATH, config)
