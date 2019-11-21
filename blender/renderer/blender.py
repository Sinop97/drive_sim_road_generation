from blender.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad import schema
from os import path, makedirs
import bpy
import os


def generate_blend(xml_content, target_dir, add_vehicle, output_dir, gazebo_world_path):
    # delete box in scene originally
    doc = schema.CreateFromDocument(xml_content)

    scene = bpy.data.scenes.new("Scene")
    # bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    groundplane.draw(doc, target_dir)
    if add_vehicle:
        ego_vehicle.draw() # render keyframes at the end by moving the object and calling bpy
    for obst in doc.obstacle:
        if obst.type != "blockedArea":
            obstacle.draw(obst)
    mesh_basepath = os.path.join(gazebo_world_path, 'meshes')
    for sign in doc.trafficSign:
        traffic_sign.draw(sign, mesh_basepath)
    for ramp in doc.ramp:
        special_objects.draw_ramp(ramp, mesh_basepath)

    # bpy.ops.wm.save_mainfile(os.path.join(output_dir, 'render_scene.blend'), compress=False)
