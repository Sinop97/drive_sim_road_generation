import bpy
from blender.renderer.segmentation_colormap import OBSTACLE_COLOR
from blender.renderer.utils import add_segmentation_mat


def obstacle_model(name, center_x, center_y, length, width, height, orientation, scene, color=None):
    bpy.ops.mesh.primitive_cube_add()
    bpy.context.active_object.name = name
    obj = bpy.data.objects[name]
    obj.location = (center_x, center_y, height/4)
    obj.scale[0] = length/2
    obj.scale[1] = width/2
    obj.scale[2] = height/2
    obj.rotation_euler = [0, 0, orientation]
    if color:
        add_segmentation_mat(color, "Seg-Mat/Obstacle_{}".format(name), obj)
    scene.objects.link(obj)


def draw(obst, scene_rgb, scene_segmentation):
    for idx, rect in enumerate(obst.shape.rectangle):
        obstacle_model("Obstacle/{0}/{1}".format(obst.id, idx),
                       rect.centerPoint.x, rect.centerPoint.y, rect.length,
                       rect.width, 0.2, - rect.orientation, scene_rgb)

        obstacle_model("Seg-Obstacle/{0}/{1}".format(obst.id, idx),
                       rect.centerPoint.x, rect.centerPoint.y, rect.length,
                       rect.width, 0.2, - rect.orientation, scene_segmentation, color=OBSTACLE_COLOR)