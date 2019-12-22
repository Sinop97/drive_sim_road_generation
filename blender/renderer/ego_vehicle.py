import bpy
import os
import math
from blender.renderer.segmentation_colormap import EGO_VEHICLE_COLOR
from blender.renderer.utils import duplicate_add_segmentation


def draw(gazebo_sim_path, scene_rgb, scene_seg, config):
    model_file = 'high_shell_export.dae'
    model_path = os.path.join(gazebo_sim_path, 'meshes', model_file)
    bpy.ops.wm.collada_import(filepath=model_path)

    ego_vehicle_name = 'ego_vehicle'
    bpy.context.active_object.name = ego_vehicle_name
    ego_vehicle = bpy.data.objects[ego_vehicle_name]
    ego_vehicle.data.use_auto_smooth = True
    bpy.context.scene.cursor_location = (0, 0, 0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    ego_vehicle.rotation_euler = (0, 0, math.pi)
    scene_rgb.objects.link(ego_vehicle)

    if not config['use_vehicle_mask']:
        duplicate_add_segmentation('seg-' + ego_vehicle_name, EGO_VEHICLE_COLOR, scene_seg)
    return ego_vehicle_name
