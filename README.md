# Randomized Road Generation and 3D Visualization

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
Generate a random scenario in CommonRoad XML from Template XML:

```
./road-generator.py presets/driving.xml -o driving-scenario.xml
```

Attention: for rendering of the advanced cup track, you will need an installed "Din 1451 Std" font.

Render CommonRoad XML for Gazebo:

```
./gazebo-renderer.py driving-scenario.xml -o world
```

View in Gazbeo:
(Make sure the plugin's build folder is set as environment variable `GAZEBO_PLUGIN_PATH`)

```
cd world
gazebo world.sdf
```

Road generation and rendering can also be done in a single step:

```
./road-generator.py presets/driving.xml | ./gazebo-renderer.py -o world
```

## Blender Renderer

The Blender renderer automatically converts the intermediate CommonRoad representation and renders keyframe images (currently RGB color and Semantic Segmentation) driving along them.

Configuration is set via variables in the file ```blender-renderer.py``` as Blender unfortunately does not allow the passing of arguments during script execution.

Before running, the following packages have to cloned as well:
- drive_gazebo_worlds
- drive_gazebo_sim

Afterwards, the user has to adjust the ```GAZEBO_WORLDS_PATH``` and ```GAZEBO_SIM_PATH``` variables in the file ```blender-renderer.py``` to point to the root folders of the packages, respectively. 

The variable ```INPUT_PATH``` has to point to the intermediate CommonRoad file generated as described above by ```road-generator.py ```.

Additional configuration options are available:
- ```OUTPUT_DIR``` : output directory (default: ```./blender-output```)
- ```FORCE_OUTPUT``` : overwrite existing files if output folder exists (default: ```True```)
- ```ADD_VEHICLE``` : render ego-vehicle (default: ```True```)

Finally, the renderer can the be run using:
```
bash ./generate_blender_env.sh
```

We have tested it using Blender 2.79. Images are rendered in the resolution of 1280 x 800 (corresponding to the Intel RealSense D435 camera). Output images are uncompressed .png (this might require a large amount of free memory for larger sequences). The RGB images are found in ```OUTPUT_DIR/rgb``` Semantic Segmentation images inside ```OUTPUT_DIR/semseg_color```.  The label mapping for the Semantic Segmentation images can be found in ```./blender/renderer/segmentation_colormap.py```.

Two separate scenes are created within the Blender file, one for the RGB image and one for the Semantic Segmentation ground truth.

The RGB image is rendered using the Cycles Renderer (warning, this might be quite computationally expensive, especially if done on the CPU). It uses a HDRI skydome image in order to introduce a reasonably realistic ambient lighting and background. The background is configured via dicts found in ```blender/renderer/env_configs.py```. 
The user has to specify the HDRI image filepath (we have rendered using 2K images) as well as the default orientation and scale of the skybox dome (this can be calibrated empirically using the Cycles background view in the viewport and the background node editor).

The Semantic Segmentation ground truth images are rendered using the internal Blender renderer. Mipmaps are disabled on the ground plane materials in order to prevent blurry artifcats from appearing on faraway roads. 