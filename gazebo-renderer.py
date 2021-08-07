#!/usr/bin/env python3
from commonroad.renderer import sdf
import argparse, sys, os
import shutil   

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate Gazebo SDF files from CommonRoad XML")
    parser.add_argument("input", nargs="?", type=argparse.FileType("r"),
        default=sys.stdin)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--force", "-f", action="store_true")
    parser.add_argument("--add_vehicle", "-av", action="store_true")
    parser.add_argument("--add_color", "-ac", action="store_true")
    parser.add_argument("--concatenate_tiles", "-ct", action="store_true")
    parser.add_argument("--remove", "-r", action="store_true")
    parser.add_argument("--distance", "-d", type=float, required=True)
    args = parser.parse_args()
    
    os.makedirs(args.output, exist_ok=True)
    if args.remove and os.listdir(args.output) != []:
        output_list = os.listdir(args.output)
        for i in range(len(output_list)):
            shutil.rmtree(os.getcwd() + "\\" + args.output + "\\" + output_list[i], ignore_errors=True)

    if os.listdir(args.output) != [] and not args.force:
        print("Output directory is not empty.")
        print("Use --force")
        sys.exit(1)

    with args.input as input_file:
        xml = input_file.read()
    
    sdf.generate_sdf(xml, args.output, args.add_vehicle, args.add_color, args.concatenate_tiles, args.distance)
