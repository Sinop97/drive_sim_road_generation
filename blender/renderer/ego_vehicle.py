import bpy
import os
import math


def draw():
    model_file = 'car-cc2017.dae'
    model_path = os.path.join('commonroad', 'renderer', 'models', model_file)
    bpy.ops.wm.collada_import(filepath=model_path)

    bpy.ops.transform.rotate(value=-math.pi/2, axis=(1, 0, 0), constraint_axis=(True, False, False),
                             constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                             proportional_edit_falloff='SMOOTH', proportional_size=1, release_confirm=True,
                             use_accurate=False)