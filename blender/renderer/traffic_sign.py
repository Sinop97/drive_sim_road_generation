import bpy
from commonroad.renderer.traffic_sign import SIGN_MESHES
import os


def draw(sign, sign_mesh_path):
    bpy.ops.wm.collada_import(filepath=os.path.join(sign_mesh_path,
                                                    SIGN_MESHES[sign.type]['mesh'] + '.dae'))
    bpy.ops.object.join()
    sign_name = 'Sign/{}_{}'.format(sign.type, sign.id)
    bpy.context.active_object.name = sign_name
    obj = bpy.data.objects[sign_name]

    obj.location = (sign.centerPoint.x, sign.centerPoint.y, 0)
    obj.rotation_euler = [0, 0, sign.orientation]
    obj.scale[0] = 0.001
    obj.scale[1] = 0.001
    obj.scale[2] = 0.001
    print('Added traffic sign')
