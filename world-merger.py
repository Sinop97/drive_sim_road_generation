from os import path, makedirs
import argparse, os, sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Merge the raw and colored SDF file")
    parser.add_argument("input", type=str)
    args = parser.parse_args()

    target_dir = os.getcwd()
    space = " " 

    with open(path.join(target_dir, args.input, "raw", "worlds", "world.sdf"), "r") as file:
        raw = file.readlines()
        del raw[len(raw) - 1]

    with open(path.join(target_dir, args.input, "colored", "worlds", "world.sdf"), "r") as file:
        colored = file.readlines()
        del colored[0]

    merged = raw + ['    \n'] + colored

    if not path.exists(path.join(target_dir, args.input, "merged")):
        makedirs(path.join(target_dir, args.input, "merged"))

    with open(path.join(target_dir, args.input, "merged", "world.sdf"), "w+") as file:
        file.write(space.join(merged)) 
