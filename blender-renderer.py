#!/usr/bin/env python3
import sys
import os
import shutil
# because blender does not want to have it otherwise -__-
sys.path.append(os.getcwd())

from blender.renderer.blender import generate_blend
import sys, os

OUTPUT_DIR = 'blender-output'  # output directory
FORCE_OUTPUT = True  # overwrite output if True
ADD_VEHICLE = True  # render ego-vehicle in the frames
INPUT_FILE = 'driving-scenario.xml'  # input CommonRoad file
GAZEBO_WORLDS_PATH = '../drive_gazebo_worlds'  # location of the drive_gazebo_worlds package
GAZEBO_SIM_PATH = '../drive_gazebo_sim'  # location of the drive_gazebo_sim package

if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if os.listdir(OUTPUT_DIR) != [] and not FORCE_OUTPUT:
        print("Output directory is not empty.")
        print("Use --force")
        sys.exit(1)

    shutil.copy2(INPUT_FILE, OUTPUT_DIR)

    with open(INPUT_FILE) as input_file:
        xml = input_file.read()

    generate_blend(xml, OUTPUT_DIR, ADD_VEHICLE, OUTPUT_DIR, GAZEBO_WORLDS_PATH, GAZEBO_SIM_PATH)
