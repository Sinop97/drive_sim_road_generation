import bpy
import os


def draw_ramp(ramp, mesh_path):
    bpy.ops.wm.collada_import(filepath=os.path.join(mesh_path, 'Ramp.dae'))
    ramp_name = 'Ramp/{}'.format(ramp.id)
    bpy.context.active_object.name = ramp_name
    obj = bpy.data.objects[ramp_name]

    obj.location = (ramp.centerPoint.x, ramp.centerPoint.y, 0)
    obj.rotation_euler = [0, 0, ramp.orientation]
    obj.scale[0] = 0.001
    obj.scale[1] = 0.001
    obj.scale[2] = 0.001
    print('Added ramp')