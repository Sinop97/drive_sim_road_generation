import bpy
import os
from blender.renderer.segmentation_colormap import convert_to_one_range


def duplicate_add_segmentation(obj_name, mat_color, scene_seg):
    bpy.ops.object.duplicate()
    bpy.context.active_object.name = obj_name
    seg_obj = bpy.data.objects[obj_name]
    add_segmentation_mat(mat_color, 'Seg-Mat/'+obj_name, seg_obj)
    scene_seg.objects.link(seg_obj)


def add_segmentation_mat(mat_color, mat_name, obj):
    mat = bpy.data.materials.new(name=mat_name)
    mat.diffuse_color = convert_to_one_range(mat_color)
    mat.use_shadeless = True
    for i in range(len(obj.material_slots)):
        bpy.ops.object.material_slot_remove({'object': obj})
    obj.data.materials.append(mat)


def generate_material_cycles(material_name, filename, shader_type='ShaderNodeBsdfDiffuse'):
    bpy.context.scene.render.engine = 'CYCLES'
    mat = bpy.data.materials.new(material_name)
    mat.use_nodes = True
    matnodes = mat.node_tree.nodes

    tex = matnodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images[os.path.basename(filename)]

    if shader_type != "ShaderNodeBsdfDiffuse":
        shader_node = matnodes.new(shader_type)
    else:
        shader_node = matnodes['Diffuse BSDF']

    # link color
    mat.node_tree.links.new(tex.outputs['Color'], shader_node.inputs['Color'])
    mat.node_tree.links.new(shader_node.outputs['BSDF'],
                            matnodes['Material Output'].inputs['Surface'])
    return mat


def convert_mat_to_cycles(mat):
    # other texture slots are dummy placeholders in most cases
    texture = mat.texture_slots[0].texture

    bpy.context.scene.render.engine = 'CYCLES'
    mat.use_nodes = True
    matnodes = mat.node_tree.nodes

    tex = matnodes.new('ShaderNodeTexImage')
    tex.image = texture.image
    mat.node_tree.links.new(tex.outputs['Color'], matnodes['Diffuse BSDF'].inputs['Color'])
    mat.node_tree.links.new(matnodes['Diffuse BSDF'].outputs['BSDF'],
                            matnodes['Material Output'].inputs['Surface'])
    return mat


def generate_material_internal_segmentation(material_name, filename):
    bpy.data.textures.new(filename, type='IMAGE')
    bpy.data.textures[filename].image = bpy.data.images[os.path.basename(filename)]
    bpy.data.textures[filename].extension = 'CLIP'
    bpy.data.textures[filename].use_interpolation = False
    bpy.data.textures[filename].filter_size = 0.10
    bpy.data.materials.new(material_name)
    mat = bpy.data.materials[material_name]
    mat.use_shadeless = True
    slot = mat.texture_slots.add()
    slot.texture = bpy.data.textures[filename]
    slot.texture_coords = 'ORCO'
    return mat