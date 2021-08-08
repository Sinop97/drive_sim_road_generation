# Road generation to use in a simulation for semantic segmentation
## General

This project extends the original [TUM](https://github.com/tum-phoenix/drive_sim_road_generation) project for our use case and implements new different objects in the Road Generator. It is a part of the master "Applied Computer Science" at Esslingen University of Applied Sciences and handles the theme "semantic segmentation for an autonomous model vehicle".

## Installation

First: Install **Python 3** and **Gazebo 8**.

http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

```sh
sudo apt-get install python3 gazebo8 libgazebo8-dev

pip install -r requirements.txt
```

Compile the keyframe plugin for gazebo:

```
cd keyframe_plugin
mkdir build
cd build
cmake ../
make

export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
```

Additionally, install the font located in the commonroad/renderer/fonts subfolder, which is needed to draw speed limit numbers.

## Usage
### Generating the road
Generate a scenario in CommonRoad XML from Template XML:
```
./road-generator.py presets/driving.xml --add_color
```
Arguments:

--add_color or -ac: color the world to create labels in the next step (optional)

### Rendering the road
Render CommonRoad XML for Gazbeo:
```
./gazebo-renderer.py driving-scenario.xml -o world -d 40 --add_color --concatenate_tiles
```
Arguments:

--distance or -d: y-distance to world's origin  

--add_color or -ac: color the world to create labels in the next step (optional)

--concatenate_tiles or -ct: concatenate the tiles to close the gaps between the tiles (optional)


Road generation and rendering can also be done in a single step:
```
./road-generator.py presets/driving.xml | ./gazebo-renderer.py -o world
```

View in Gazbeo:
(Make sure the plugin's build folder is set as environment variable `GAZEBO_PLUGIN_PATH`)
```
cd world
gazebo world.sdf
```

### Merging the worlds
Merge the raw world with the colored world:

```
./world-merger.py world
```
