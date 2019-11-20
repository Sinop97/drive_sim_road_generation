#!/usr/bin/env python3
import sys
import os
sys.path.append(os.getcwd())

from blender.renderer.blender import generate_blend
import argparse, sys, os

OUTPUT_DIR = 'blender-output'
FORCE_OUTPUT = True
ADD_VEHICLE = True
INPUT_FILE = 'driving-scenario.xml'

if __name__ == "__main__":
    # parser = argparse.ArgumentParser(
    #     description="Generate .blend files from CommonRoad XML")
    # parser.add_argument("input", nargs="?", type=argparse.FileType("r"),
    #     default=sys.stdin)
    # parser.add_argument("--output", "-o", required=True)
    # parser.add_argument("--force", "-f", action="store_true")
    # parser.add_argument("--add_vehicle", "-av", action="store_true")
    # args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if os.listdir(OUTPUT_DIR) != [] and not FORCE_OUTPUT:
        print("Output directory is not empty.")
        print("Use --force")
        sys.exit(1)

    with open(INPUT_FILE) as input_file:
        xml = input_file.read()

    generate_blend(xml, OUTPUT_DIR, ADD_VEHICLE)
