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
from commonroad.renderer.groundplane import ROADMARKING_TYPE_TO_VISUAL
from commonroad.renderer.groundplane import boundary_to_equi_distant
from commonroad.renderer.groundplane import expand_boundary
from commonroad.renderer.groundplane import get_lanelet_by_id
from commonroad.renderer.groundplane import draw_rectangle
from PIL import Image, ImageOps

from blender.renderer.utils import generate_material_cycles, generate_material_internal_segmentation

import bpy
import bmesh
from mathutils import Vector
from blender.renderer.segmentation_colormap import BACKGROUND_COLOR, DRIVABLE_AREA_SEGMENTATION_COLOR,\
    LANE_MARKING_SEGMENTATION_COLOR, STOPLINE_DASHED_SEGMENTATION_COLOR, STOPLINE_SEGMENTATION_COLOR, \
    ZEBRA_COLOR, BLOCKED_AREA_SEGMENTATION_COLOR, TRAFFIC_MARKING_SEGMENTATION_COLORS, INTERSECTION_COLOR
from blender.renderer.segmentation_colormap import convert_to_one_range

PIXEL_PER_UNIT = 500
TILE_SIZE = 2048
PADDING = 3


def add_ground_segment(texture_file, x, y, segment_scale, segment_name, scene, config, segmap=False, scale_factor=0.99):
    bpy.data.images.load(texture_file)

    bpy.ops.mesh.primitive_plane_add(location=(x, y, 0))
    bpy.context.active_object.name = segment_name
    obj = bpy.data.objects[segment_name]
    obj.scale[0] = segment_scale/2 
    obj.scale[1] = segment_scale/2
    obj.select = True

    lm = obj.data.uv_textures.get("UV")
    if not lm:
        lm = obj.data.uv_textures.new("UV")
    lm.active = True
    bpy.ops.object.editmode_toggle()
    bpy.ops.uv.unwrap()

    # scale down uv map to prevent bleed -> not necessary if texture is not mirrored
    # me = obj.data
    # bm = bmesh.from_edit_mesh(me)
    # uv_layer = bm.loops.layers.uv.verify()
    # bm.faces.layers.tex.verify()  # currently blender needs both layers.
    # for f in bm.faces:
    #     for l in f.loops:
    #         l[uv_layer].uv *= scale_factor
    #         l[uv_layer].uv += Vector(((1 - scale_factor)/2, (1 - scale_factor)/2))
    # bmesh.update_edit_mesh(me)
    #
    bpy.ops.object.editmode_toggle()

    if segmap:
        mat = generate_material_internal_segmentation(segment_name, texture_file)
        mat.texture_slots[0].texture_coords = 'UV'
        mat.texture_slots[0].texture.use_mipmap = False
    else:
        bpy.context.scene.objects.active = obj
        bpy.ops.mesh.uv_texture_add()
        mat = generate_material_cycles(segment_name, texture_file, shader_type=config['groundplane_shader_type'])

    obj.data.materials.append(mat)

    scene.objects.link(obj)


def draw_stop_lines_segmentation(ctx, lanelet):
    ctx.save()
    p1 = lanelet.leftBoundary.point[-1]
    p2 = lanelet.rightBoundary.point[-1]

    if lanelet.stopLine:
        ctx.set_source_rgb(*convert_to_one_range(STOPLINE_SEGMENTATION_COLOR))
        lineWidth = 0.04
        segmentLength = 0.08
        segmentGap = 0.06
        if lanelet.stopLine == "dashed":
            ctx.set_source_rgb(*convert_to_one_range(STOPLINE_DASHED_SEGMENTATION_COLOR))
        else:
            ctx.set_dash([])
            ctx.set_line_cap(cairo.LINE_CAP_BUTT)
        ctx.set_line_width(lineWidth)
        ctx.move_to(p1.x, p1.y)
        ctx.line_to(p2.x, p2.y)
        ctx.stroke()
    ctx.restore()


def draw_zebra_crossing_segmentation(ctx, lanelet):
    ctx.save()
    ctx.set_source_rgb(*convert_to_one_range(ZEBRA_COLOR))
    ctx.move_to(lanelet.leftBoundary.point[0].x, lanelet.leftBoundary.point[0].y)
    ctx.line_to(lanelet.leftBoundary.point[1].x, lanelet.leftBoundary.point[1].y)
    ctx.line_to(lanelet.rightBoundary.point[1].x, lanelet.rightBoundary.point[1].y)
    ctx.line_to(lanelet.rightBoundary.point[0].x, lanelet.rightBoundary.point[0].y)
    ctx.close_path()
    ctx.fill()
    ctx.stroke()
    ctx.restore()


# a separate function that tracks the lane segments following each other and gets an index of the first one would
# be needed to do full drivable / adjacent segmentation (no ego-lane information here)
def draw_drivable_segmentation(ctx, lanelet_list):
    all_ids = [lanelet.id for lanelet in lanelet_list]
    while len(all_ids) > 0:
        current_id = all_ids[0]
        suc = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), 'rightBoundary', "successor",
                              ignore_boundary=True)
        pred = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), 'rightBoundary',
                               "predecessor", ignore_boundary=True)
        ids_in_run = pred[::-1] + [current_id] + suc

        for id in ids_in_run:
            all_ids.remove(id)

        lanelets = list(map(lambda x: get_lanelet_by_id(lanelet_list, x), ids_in_run))

        ctx.save()
        ctx.set_source_rgb(*convert_to_one_range(DRIVABLE_AREA_SEGMENTATION_COLOR))

        ctx.move_to(lanelets[0].rightBoundary.point[0].x,
                    lanelets[0].rightBoundary.point[0].y)

        for lanelet in lanelets:
            for p in lanelet.rightBoundary.point:
                ctx.line_to(p.x, p.y)

        for lanelet in reversed(lanelets):
            for p in reversed(lanelet.leftBoundary.point):
                ctx.line_to(p.x, p.y)
        ctx.fill()
        ctx.stroke()
        ctx.restore()


# necessary because otherwise we have bleeding artifacts between the two road lanes
def draw_midline_segmentation(ctx, lanelet_list, boundary_name):
    all_ids = [lanelet.id for lanelet in lanelet_list
        if getattr(lanelet, boundary_name).lineMarking is not None]
    while len(all_ids) > 0:
        current_id = all_ids[0]
        suc = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), boundary_name, "successor")
        pred = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), boundary_name, "predecessor")
        ids_in_run = pred[::-1] + [current_id] + suc

        for id in ids_in_run:
            all_ids.remove(id)

        lanelets = list(map(lambda x: get_lanelet_by_id(lanelet_list, x), ids_in_run))

        ctx.save()
        ctx.set_source_rgb(*convert_to_one_range(DRIVABLE_AREA_SEGMENTATION_COLOR))
        ctx.set_line_width (0.02)

        ctx.move_to(getattr(lanelets[0], boundary_name).point[0].x,
            getattr(lanelets[0], boundary_name).point[0].y)

        for lanelet in lanelets:
            for p in getattr(lanelet, boundary_name).point:
                ctx.line_to(p.x, p.y)
        ctx.stroke()
        ctx.restore()


def draw_blocked_area_segmentation(ctx, rectangle):
    ctx.save()
    ctx.set_source_rgb(*convert_to_one_range(BLOCKED_AREA_SEGMENTATION_COLOR))
    ctx.translate(rectangle.centerPoint.x, rectangle.centerPoint.y)
    ctx.rotate(-rectangle.orientation)

    ctx.set_line_width (0.02)
    sheering = rectangle.width / 2
    ctx.move_to(- rectangle.length / 2, - rectangle.width / 2)
    ctx.line_to(rectangle.length / 2, - rectangle.width / 2)
    ctx.line_to(rectangle.length / 2 - sheering, rectangle.width / 2)
    ctx.line_to(- rectangle.length / 2 + sheering, rectangle.width / 2)
    ctx.close_path()
    ctx.fill()
    ctx.stroke()
    ctx.restore()


def draw_obstacle_segmentation(ctx, obstacle):
    if obstacle.type == "blockedArea":
        for rect in obstacle.shape.rectangle:
            draw_blocked_area_segmentation(ctx, rect)
    elif obstacle.type == "segmentationIntersection":
        for rect in obstacle.shape.rectangle:
            draw_rectangle(ctx, rect, color=INTERSECTION_COLOR)


def draw_road_marking_segmentation(ctx, marking):
    ctx.set_source_rgb(*convert_to_one_range(TRAFFIC_MARKING_SEGMENTATION_COLORS[marking.type]))
    marking_visual = ROADMARKING_TYPE_TO_VISUAL[marking.type]
    if marking_visual.marker_text:
        ctx.save()
        ctx.set_dash([])
        font = "DIN 1451 Std"
        font_size = 0.4
        text = '30'
        font_args = [cairo.FONT_SLANT_NORMAL]
        ctx.translate(marking.centerPoint.x,  # - 0.145*math.cos(marking.orientation),
                      marking.centerPoint.y)  # - 0.145*math.sin(marking.orientation))
        ctx.rotate(marking.orientation)
        # mirror text
        ctx.transform(cairo.Matrix(1.0, 0, 0, -1, 0, 0))
        ctx.translate(-0.145, 0.29)
        ctx.select_font_face(font, *font_args)
        ctx.set_font_size(font_size)
        ctx.text_path(marking_visual.marker_text)
        ctx.set_line_width(0.01)
        (x_bearing, y_bearing, text_width, text_height,
         x_advance, y_advance) = ctx.text_extents(text)
        ctx.fill_preserve()
        ctx.stroke()
        ctx.restore()
    if marking_visual.crossed:
        ctx.save()
        ctx.move_to(marking.centerPoint.x + 0.145 * math.cos(marking.orientation),
                    marking.centerPoint.y + 0.145 * math.sin(marking.orientation))
        ctx.line_to(marking.centerPoint.x + 0.145 * math.cos(marking.orientation)
                    - text_height * math.cos(marking.orientation) + text_width * math.sin(marking.orientation),
                    marking.centerPoint.y + 0.145 * math.sin(marking.orientation)
                    - text_height * math.sin(marking.orientation) - text_width * math.cos(marking.orientation))
        ctx.move_to(marking.centerPoint.x + (0.145 - text_height) * math.cos(marking.orientation),
                    marking.centerPoint.y + (0.145 - text_height) * math.sin(marking.orientation))
        ctx.line_to(marking.centerPoint.x + 0.145 * math.cos(marking.orientation)
                    + text_width * math.sin(marking.orientation),
                    marking.centerPoint.y + 0.145 * math.sin(marking.orientation)
                    - text_width * math.cos(marking.orientation))
        ctx.set_line_width(0.05)
        ctx.stroke()
        ctx.restore()

    if marking_visual.marker_image:
        # draw to temporary surface first to be able to change color
        ctx.save()
        img = cairo.ImageSurface(cairo.FORMAT_ARGB32, 72, 500) # todo: will have to change size dynamically if we add other markers in the future, for now all have this
        ctx_temp = cairo.Context(img)
        handle = Rsvg.Handle()
        svg = handle.new_from_file(marking_visual.marker_image.value)
        svg.render_cairo(ctx_temp)

        ctx.translate(marking.centerPoint.x, marking.centerPoint.y)
        ctx.rotate(marking.orientation)
        ctx.scale(0.001, 0.001)
        ctx.mask_surface(img)
        ctx.stroke()
        ctx.restore()


def pad_png(path, ratio):
    im = Image.open(path)
    old_size = im.size  # old_size[0] is in (width, height) format

    new_size = tuple([int(x * ratio) + 3 for x in old_size])

    im = im.resize(new_size, Image.ANTIALIAS)
    new_im = Image.new("RGB", old_size)
    new_im.paste(im, ((old_size[0] - new_size[0]) // 2, (old_size[1] - new_size[1]) // 2))
    new_im.save(path)


def draw(doc, target_dir, scene_rgb, scene_segmentation, obstacles, config):
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
        # pad_png(texture_path, config['texture_padding_ratio'])

        add_ground_segment(
            texture_path,
            bounding_box.x_min + (x + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            bounding_box.y_min + (y + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            TILE_SIZE / PIXEL_PER_UNIT,
            "Tile_{0}_{1}".format(x, y),
            scene_rgb,
            config,
            scale_factor=config['texture_padding_ratio']
        )

        # draw segmentation map
        surface = cairo.ImageSurface(cairo.FORMAT_RGB24, TILE_SIZE, TILE_SIZE)
        ctx = cairo.Context(surface)

        ctx.set_source_rgb(*BACKGROUND_COLOR)
        ctx.rectangle(0, 0, TILE_SIZE, TILE_SIZE)
        ctx.fill()

        # Inverse y-axis
        ctx.translate(0, TILE_SIZE / 2)
        ctx.scale(1, -1)
        ctx.translate(0, -TILE_SIZE / 2)

        ctx.scale(PIXEL_PER_UNIT, PIXEL_PER_UNIT)
        ctx.translate(-bounding_box.x_min, -bounding_box.y_min)
        ctx.translate(- x * TILE_SIZE / PIXEL_PER_UNIT, - y * TILE_SIZE / PIXEL_PER_UNIT)

        draw_drivable_segmentation(ctx, doc.lanelet)
        draw_midline_segmentation(ctx, doc.lanelet, "leftBoundary")
        for obstacle in doc.obstacle:
            if obstacle.type == "segmentationIntersection":
                for rect in obstacle.shape.rectangle:
                    draw_rectangle(ctx, rect, color=INTERSECTION_COLOR)
        for lanelet in doc.lanelet:
            draw_stop_lines_segmentation(ctx, lanelet)
            if lanelet.type == "zebraCrossing":
                draw_zebra_crossing_segmentation(ctx, lanelet)

        draw_all_boundaries(ctx, doc.lanelet, "leftBoundary",
                            convert_to_one_range(LANE_MARKING_SEGMENTATION_COLOR))
        draw_all_boundaries(ctx, doc.lanelet, "rightBoundary",
                            convert_to_one_range(LANE_MARKING_SEGMENTATION_COLOR))

        for obstacle in doc.obstacle:
            if obstacle.type == "blockedArea":
                for rect in obstacle.shape.rectangle:
                    draw_blocked_area_segmentation(ctx, rect)

        # do not segment island junctions for now
        # for island_junction in doc.islandJunction:
        #     draw_island_junction_segmentation(ctx, island_junction)

        for road_marking in doc.roadMarking:
            draw_road_marking_segmentation(ctx, road_marking)

        # sha_256 = hashlib.sha()
        # sha_256.update(surface.get_data())
        # hash = sha_256.hexdigest()

        texture_file = "segmentation-tile-{}-{}.png".format(x, y)
        texture_path = path.join(target_dir, "materials", "textures", texture_file)
        surface.write_to_png(texture_path)
        # pad_png(texture_path, config['texture_padding_ratio'])

        add_ground_segment(
            texture_path,
            bounding_box.x_min + (x + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            bounding_box.y_min + (y + 0.5) * TILE_SIZE / PIXEL_PER_UNIT,
            TILE_SIZE / PIXEL_PER_UNIT,
            "Seg-Tile_{0}_{1}".format(x, y),
            scene_segmentation,
            config,
            segmap=True,
            scale_factor=config['texture_padding_ratio']
        )
