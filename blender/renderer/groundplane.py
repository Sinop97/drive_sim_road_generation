import cairo
# have to use pycairo instead of cairocffi as Rsvg bindings don't work with the latter
#import cairocffi as cairo
import math
from commonroad import utils
from os import path
import os
import hashlib
from tqdm import tqdm
import numpy as np
from enum import Enum
from collections import namedtuple
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg
from commonroad.renderer.groundplane import draw_stop_line
from commonroad.renderer.groundplane import draw_zebra_crossing
from commonroad.renderer.groundplane import draw_all_boundaries
from commonroad.renderer.groundplane import draw_obstacle
from commonroad.renderer.groundplane import draw_island_junction
from commonroad.renderer.groundplane import draw_road_marking

import bpy

PIXEL_PER_UNIT = 500
TILE_SIZE = 2048
PADDING = 3


def add_ground_segment(texture_file, x, y, segment_scale, segment_name):
    bpy.data.images.load(texture_file)

    bpy.ops.mesh.primitive_plane_add(location=(x, y, 0))
    bpy.context.active_object.name = segment_name
    obj = bpy.data.objects[segment_name]
    obj.scale[0] = segment_scale
    obj.scale[1] = segment_scale

    bpy.data.textures.new(segment_name, type='IMAGE')
    #print([image.name for image in bpy.data.images])
    bpy.data.textures[segment_name].image = bpy.data.images[os.path.basename(texture_file)]
    bpy.data.materials.new(segment_name)
    mat = bpy.data.materials[segment_name]
    slot = mat.texture_slots.add()
    slot.texture = bpy.data.textures[segment_name]
    slot.use_map_normal = True
    slot.texture_coords = 'ORCO'

    obj.data.materials.append(mat)

    print("Added segment {}".format(segment_name))


def draw(doc, target_dir):
    bounding_box = utils.get_bounding_box(doc)
    bounding_box.x_min -= PADDING
    bounding_box.y_min -= PADDING
    bounding_box.x_max += PADDING
    bounding_box.y_max += PADDING

    width = math.ceil((bounding_box.x_max - bounding_box.x_min) * PIXEL_PER_UNIT)
    height = math.ceil((bounding_box.y_max - bounding_box.y_min) * PIXEL_PER_UNIT)

    width_num = int(math.ceil(width / TILE_SIZE))
    height_num = int(math.ceil(height / TILE_SIZE))

    os.makedirs(path.join(target_dir, "materials", "textures"), exist_ok=True)

    for (x, y) in tqdm([(x,y) for x in range(width_num) for y in range(height_num)]):
        surface = cairo.ImageSurface(cairo.FORMAT_RGB24, TILE_SIZE, TILE_SIZE)
        ctx = cairo.Context(surface)

        # fill black
        ctx.set_source_rgb(0, 0, 0)
        ctx.rectangle(0, 0, TILE_SIZE, TILE_SIZE)
        ctx.fill()

        # Inverse y-axis
        ctx.translate(0, TILE_SIZE / 2)
        ctx.scale(1, -1)
        ctx.translate(0, -TILE_SIZE / 2)

        ctx.scale(PIXEL_PER_UNIT, PIXEL_PER_UNIT)
        ctx.translate(-bounding_box.x_min, -bounding_box.y_min)
        ctx.translate(- x * TILE_SIZE / PIXEL_PER_UNIT, - y * TILE_SIZE / PIXEL_PER_UNIT)

        ctx.set_source_rgb(1, 1, 1)
        for lanelet in doc.lanelet:
            draw_stop_line(ctx, lanelet)
            if lanelet.type == "zebraCrossing":
                draw_zebra_crossing(ctx, lanelet)

        draw_all_boundaries(ctx, doc.lanelet, "leftBoundary")
        draw_all_boundaries(ctx, doc.lanelet, "rightBoundary")

        for obstacle in doc.obstacle:
            draw_obstacle(ctx, obstacle)

        for island_junction in doc.islandJunction:
            draw_island_junction(ctx, island_junction)

        for road_marking in doc.roadMarking:
            draw_road_marking(ctx, road_marking)

        # sha_256 = hashlib.sha()
        # sha_256.update(surface.get_data())
        # hash = sha_256.hexdigest()

        texture_file = "tile-{}-{}.png".format(x, y)
        texture_path = path.join(target_dir, "materials", "textures", texture_file)
        surface.write_to_png(texture_path)

        add_ground_segment(
            texture_path,
            bounding_box.x_min + (x + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            bounding_box.y_min + (y + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            TILE_SIZE / PIXEL_PER_UNIT,
            "Tile_{0}_{1}".format(x, y)
        )
