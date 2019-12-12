import bpy
from commonroad.renderer.traffic_sign import SIGN_MESHES
import os
from blender.renderer.segmentation_colormap import SIGN_BASE_COLOR, SIGN_TO_COLOR
from blender.renderer.segmentation_colormap import convert_to_one_range
from blender.renderer.utils import add_segmentation_mat, convert_mat_to_cycles
import numpy as np


def draw(sign, sign_mesh_path, scene_rgb, scene_seg, sign_idx=0):
    bpy.ops.wm.collada_import(filepath=os.path.join(sign_mesh_path,
                                                    SIGN_MESHES[sign.type]['mesh'] + '.dae'))
    sign_mesh_names = []
    print('Selected objects: {}'.format(bpy.context.selected_objects))
    for obj in bpy.context.selected_objects:
        obj.name = obj.name.replace('Body', 'Seg-Sign/{}_{}'.format(sign.type, sign.id))
        # dae importer auto-renames the UV map, need to have uniform name to be able to merge
        for uv_layer in obj.data.uv_layers:
            uv_layer.name = 'UVMap'
        for m_slot in obj.material_slots:
            m_slot.material = convert_mat_to_cycles(m_slot.material)
        sign_mesh_names.append(obj.name)
    sign_mesh_names = sorted(sign_mesh_names)
    bpy.ops.object.duplicate()
    bpy.ops.object.join()

    sign_name = 'Sign/{}_{}'.format(sign.type, sign.id)
    bpy.context.active_object.name = sign_name
    obj = bpy.data.objects[sign_name]

    obj.location = (sign.centerPoint.x, sign.centerPoint.y, 0)
    obj.rotation_euler = [0, 0, sign.orientation]
    obj.scale[0] = 0.001
    obj.scale[1] = 0.001
    obj.scale[2] = 0.001
    obj.data.use_auto_smooth = True
    scene_rgb.objects.link(obj)

    for idx, name in enumerate(sign_mesh_names[:-1]):
        add_segmentation_mat(SIGN_BASE_COLOR, "Seg-Mat/Sign-Base/{}_{}_{}".format(sign.type, sign.id, idx),
                             bpy.data.objects[name])

    # for segmentation: label the forward-facing faces of label mesh only
    add_segmentation_mat(SIGN_TO_COLOR[sign.type],
                         "Seg-Mat/Seg-Sign-Base/{}_{}_{}".format(sign.type, sign.id, len(sign_mesh_names)-1),
                         bpy.data.objects[sign_mesh_names[-1]])
    sign_mesh = bpy.data.objects[sign_mesh_names[-1]].data
    # other local coordinate system for "Vorbeifahrt rechts" sign
    if sign.type == 'stvo-222':
        forward_normal_dir = (0.0, -1.0, 0.0)
    else:
        forward_normal_dir = (1.0, 0.0, 0.0)
    for face in sign_mesh.polygons:
        if np.linalg.norm(np.array(tuple(face.normal)) - np.array(forward_normal_dir)) < 0.1:
            face.material_index = 0
        else:
            face.material_index = 1

    mat = bpy.data.materials.new(name="Seg-Mat/Seg-Sign-Base/{}_{}_{}-backside_color".format(sign.type, sign.id,
                                                                                       len(sign_mesh_names)-1))
    mat.diffuse_color = convert_to_one_range(convert_to_one_range(SIGN_BASE_COLOR))
    mat.use_shadeless = True
    obj.data.materials.append(mat)

    bpy.ops.object.select_all(action='DESELECT')

    bpy.context.scene.objects.active = bpy.data.objects[sign_mesh_names[0]]
    for sign_mesh_name in sign_mesh_names:
        bpy.data.objects[sign_mesh_name].select = True
    bpy.ops.object.join()

    sign_name = 'Seg-Sign/{}_{}'.format(sign.type, sign.id)
    ctx = bpy.context.copy()
    ctx['active_object'].name = sign_name
    obj = bpy.data.objects[sign_name]

    obj.location = (sign.centerPoint.x, sign.centerPoint.y, 0)
    obj.rotation_euler = [0, 0, sign.orientation]
    obj.scale[0] = 0.001
    obj.scale[1] = 0.001
    obj.scale[2] = 0.001
    # pass index for instance IDs
    obj.data.use_auto_smooth = True
    print('Sign {} setting pass index to {} and name to {}'.format(sign.type, sign_idx, sign_name))
    obj.pass_index = sign_idx
    bpy.data.materials[obj.material_slots[-1].material.name].pass_index = sign_idx
    scene_seg.objects.link(obj)
