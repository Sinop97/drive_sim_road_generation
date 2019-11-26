import bpy
import os
from blender.renderer.segmentation_colormap import RAMP_COLOR
from blender.renderer.utils import duplicate_add_segmentation


def draw_ramp(ramp, mesh_path, scene_rgb, scene_seg):
    bpy.ops.wm.collada_import(filepath=os.path.join(mesh_path, 'Ramp.dae'))
    ramp_name = 'Ramp/{}'.format(ramp.id)
    bpy.context.active_object.name = ramp_name
    obj = bpy.data.objects[ramp_name]

    obj.location = (ramp.centerPoint.x, ramp.centerPoint.y, 0)
    obj.rotation_euler = [0, 0, ramp.orientation]
    obj.scale[0] = 0.001
    obj.scale[1] = 0.001
    obj.scale[2] = 0.001
    scene_rgb.objects.link(obj)

    duplicate_add_segmentation("Seg-Ramp-{}".format(ramp.id), RAMP_COLOR, scene_seg)

