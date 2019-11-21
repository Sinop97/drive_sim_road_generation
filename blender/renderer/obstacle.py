import bpy


def obstacle_model(name, center_x, center_y, length, width, height, orientation):
    bpy.ops.mesh.primitive_cube_add()
    bpy.context.active_object.name = name
    obj = bpy.data.objects[name]
    obj.location = (center_x, center_y, 0)
    obj.scale[0] = length/2
    obj.scale[1] = width/2
    obj.scale[2] = height/2
    obj.rotation_euler = [0, 0, orientation]


def draw(obst):
    for idx, rect in enumerate(obst.shape.rectangle):
        print('Added obstacle')
        obstacle_model("Obstacle/{0}/{1}".format(obst.id, idx),
                       rect.centerPoint.x, rect.centerPoint.y, rect.length,
                       rect.width, 0.2, - rect.orientation)