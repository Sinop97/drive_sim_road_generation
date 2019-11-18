import bpy

def draw(obst):
    raise NotImplementedError()
    bpy.data.images.load(texture_file)

    bpy.ops.mesh.primitive_plane_add(location=(x, y, 0))
    bpy.context.active_object.name = segment_name
    obj = bpy.data.objects[segment_name]
    obj.scale[0] = segment_scale
    obj.scale[1] = segment_scale

    bpy.data.textures.new(segment_name, type='IMAGE')
    bpy.data.textures[segment_name].image = bpy.data.images[os.path.basename(texture_file)]
    bpy.data.materials.new(segment_name)
    mat = bpy.data.materials[segment_name]
    slot = mat.texture_slots.add()
    slot.texture = bpy.data.textures[segment_name]
    slot.use_map_normal = True
    slot.texture_coords = 'ORCO'

    obj.data.materials.append(mat)

    print("Added obstacle {}".format(segment_name))