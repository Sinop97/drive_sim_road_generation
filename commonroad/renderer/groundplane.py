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
#import gi
#gi.require_version('Rsvg', '2.0')
#from gi.repository import Rsvg
from commonroad.renderer import concatenate
from PIL import Image

PIXEL_PER_UNIT = 500
TILE_SIZE = 2048
PADDING = 3


class MarkerImage(Enum):
    TURN_LEFT = 'commonroad/renderer/street_markings/Fahrbahnmarkierung_Pfeil_L.svg'
    TURN_RIGHT = 'commonroad/renderer/street_markings/Fahrbahnmarkierung_Pfeil_R.svg'
    ZONE_START_10 = 'commonroad/renderer/street_markings/10_zone_beginn.svg'

StreetMarking = namedtuple('StreetMarking', ['marker_image', 'marker_text', 'crossed'])


ROADMARKING_TYPE_TO_VISUAL = {
    #"10_zone_beginn": StreetMarking(marker_image=None, marker_text='10', crossed=False),
    "10_zone_beginn": StreetMarking(marker_image=MarkerImage.ZONE_START_10, marker_text=None, crossed=False),
    "20_zone_beginn": StreetMarking(marker_image=None, marker_text='20', crossed=False),
    "stvo-274.1": StreetMarking(marker_image=None, marker_text='30', crossed=False),
    "40_zone_beginn": StreetMarking(marker_image=None, marker_text='40', crossed=False),
    "50_zone_beginn": StreetMarking(marker_image=None, marker_text='50', crossed=False),
    "60_zone_beginn": StreetMarking(marker_image=None, marker_text='60', crossed=False),
    "70_zone_beginn": StreetMarking(marker_image=None, marker_text='70', crossed=False),
    "80_zone_beginn": StreetMarking(marker_image=None, marker_text='80', crossed=False),
    "90_zone_beginn": StreetMarking(marker_image=None, marker_text='90', crossed=False),
    "ende_10_zone": StreetMarking(marker_image=None, marker_text='10', crossed=True),
    "ende_20_zone": StreetMarking(marker_image=None, marker_text='20', crossed=True),
    "stvo-274.2": StreetMarking(marker_image=None, marker_text='30', crossed=True),
    "ende_40_zone": StreetMarking(marker_image=None, marker_text='40', crossed=True),
    "ende_50_zone": StreetMarking(marker_image=None, marker_text='50', crossed=True),
    "ende_60_zone": StreetMarking(marker_image=None, marker_text='60', crossed=True),
    "ende_70_zone": StreetMarking(marker_image=None, marker_text='70', crossed=True),
    "ende_80_zone": StreetMarking(marker_image=None, marker_text='80', crossed=True),
    "ende_90_zone": StreetMarking(marker_image=None, marker_text='90', crossed=True),
    "turn_left": StreetMarking(marker_image=MarkerImage.TURN_LEFT, marker_text=None, crossed=False),
    "turn_right": StreetMarking(marker_image=MarkerImage.TURN_RIGHT, marker_text=None, crossed=False),
}


def draw_boundary(ctx, boundary):
    if boundary.lineMarking is None:
        return
    ctx.set_line_width (0.02)
    if boundary.lineMarking == "dashed":
        ctx.set_dash([0.2, 0.2])
    else:
        ctx.set_dash([])

    ctx.move_to(boundary.point[0].x, boundary.point[0].y)
    for p in boundary.point[1:]:
        ctx.line_to(p.x, p.y)
    ctx.stroke()

def draw_stop_line(ctx, lanelet, add_color):
    ctx.save()
    
    if lanelet.stopLine:
        p1 = lanelet.stopLine[0]
        p2 = lanelet.stopLine[1]

        if add_color:
            lineWidth = 0.04 # 0.045

            ctx.set_dash([])
            ctx.set_line_cap(cairo.LINE_CAP_BUTT)

            if lanelet.stopLineAttributes is not None:
                ctx.set_source_rgb(lanelet.stopLineAttributes.color.red, lanelet.stopLineAttributes.color.green, lanelet.stopLineAttributes.color.blue)
            else:
                ctx.set_source_rgb((250.0/255.0), 0.0, 0.0)
                            
        else:
            lineWidth = 0.04

            if lanelet.stopLineAttributes is not None:
                lineWidth = lanelet.stopLineAttributes.lineWidth
                segmentLength = lanelet.stopLineAttributes.segmentLength
                segmentGap = lanelet.stopLineAttributes.segmentGap
                ctx.set_dash([segmentLength, segmentGap])
            else:
                ctx.set_dash([])
                ctx.set_line_cap(cairo.LINE_CAP_BUTT)

        ctx.set_line_width(lineWidth)
    
        ctx.move_to(p1.x, p1.y)
        ctx.line_to(p2.x, p2.y) 
                             
        ctx.stroke()
    ctx.restore()

def draw_rectangle(ctx, rectangle):
    ctx.save()
    ctx.translate(rectangle.centerPoint.x, rectangle.centerPoint.y)
    ctx.rotate(-rectangle.orientation)
    ctx.rectangle(- rectangle.length / 2, - rectangle.width / 2,
        rectangle.length, rectangle.width)
    ctx.fill()
    ctx.restore()

def draw_circle(ctx, circle):
    ctx.arc(circle.centerPoint.x, circle.centerPoint.y, circle.radius, 0, 2*math.pi)
    ctx.fill()

def draw_polygon(ctx, polygon):
    ctx.move_to(polygon.point[0].x, polygon.point[1].y)
    for point in polygon.point[1:]:
        ctx.line_to(point.x, point.y)
    ctx.fill()

def draw_shape(ctx, shape):
    for rect in shape.rectangle:
        draw_rectangle(ctx, rect)
    for circ in shape.circle:
        draw_circle(ctx, circ)
    for poly in shape.polygon:
        draw_polygon(ctx, poly)

def draw_parking_lot(ctx, shape):
    for rect in shape.rectangle:
        ctx.save()
        ctx.translate(rect.centerPoint.x, rect.centerPoint.y)
        ctx.rotate(-rect.orientation)

        ctx.set_source_rgb((16.0/255.0), 0.0, (96.0/255.0))
        ctx.rectangle(-(rect.length / 2.0), -(rect.width / 2.0), rect.length, rect.width - 0.01)
        #ctx.rectangle(-(rect.length / 2.0), -(rect.width / 2.0) - 0.01, rect.length, rect.width - 0.0025)
        ctx.fill() 

        ctx.restore()

def draw_parking_spot(ctx, shape, add_color):
    for rect in shape.rectangle:
        ctx.save()
        ctx.translate(rect.centerPoint.x, rect.centerPoint.y)
        ctx.rotate(-rect.orientation)

        if add_color:
            ctx.set_source_rgb(0.0, (238.0/255.0), (218.0/255.0))
            ctx.rectangle(-(rect.length / 2.0), -(rect.width / 2.0) + 0.01, rect.length, rect.width - 0.01)
            ctx.fill() 
        else:
            ctx.set_line_width (0.02)
            ctx.set_source_rgb(1, 1, 1) 

            length = rect.length / 2.0
            width = rect.width / 2.0

            ctx.move_to(- length, - width)
            ctx.line_to(- length, width + 0.01)
            ctx.move_to(length, - width)
            ctx.line_to(length, width + 0.01) 
            ctx.stroke() 
        ctx.restore()

def draw_no_parking_zone(ctx, shape):
    for rect in shape.rectangle:
        ctx.save()
        ctx.set_line_width (0.02)
        ctx.set_source_rgb(1, 1, 1)

        ctx.translate(rect.centerPoint.x, rect.centerPoint.y)
        ctx.rotate(-rect.orientation)

        length = rect.length / 2.0
        width = rect.width / 2.0

        if rect.length == 0.35:
            offset = 0.0
        else:
            offset = 0.01

        ctx.move_to(- length + offset, - width)
        ctx.line_to(- length + offset, width + 0.01)
        ctx.move_to(length - offset, - width)
        ctx.line_to(length - offset, width + 0.01)  

        ctx.move_to(- length + offset, width)
        ctx.line_to(length - offset, - width)
        ctx.move_to(length - offset, width)
        ctx.line_to(- length + offset, - width)

        ctx.stroke()
        ctx.restore()

def draw_island_junction(ctx, island):
    ctx.save()
    ctx.set_dash([])
    ctx.set_line_width (0.02)
    for i in range(0, len(island.point), 2):
        ctx.move_to(island.point[i].x, island.point[i].y)
        ctx.line_to(island.point[i+1].x, island.point[i+1].y)
    ctx.stroke()
    ctx.restore()

def draw_road_marking(ctx, marking):
    marking_visual = ROADMARKING_TYPE_TO_VISUAL[marking.type]
    if marking_visual.marker_text:
        ctx.save()
        ctx.set_dash([])
        font = "DIN 1451 Std"
        font_size = 0.4
        text = '30'
        font_args = [cairo.FONT_SLANT_NORMAL]
        ctx.translate(marking.centerPoint.x, #- 0.145*math.cos(marking.orientation),
                      marking.centerPoint.y) #- 0.145*math.sin(marking.orientation))
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
        ctx.save()
        # handle = Rsvg.Handle()
        # svg = handle.new_from_file(marking_visual.marker_image.value)
        # ctx.translate(marking.centerPoint.x, marking.centerPoint.y)
        # ctx.rotate(marking.orientation)
        # ctx.scale(0.001, 0.001)
        # svg.render_cairo(ctx)
        ctx.restore()


def draw_stripes_rect(ctx, rectangle, add_color):
    ctx.save()
    ctx.translate(rectangle.centerPoint.x, rectangle.centerPoint.y)
    ctx.rotate(-rectangle.orientation)

    ctx.set_line_width (0.02)
    ctx.set_source_rgb(1, 1, 1)
    
    #draw trapez
    sheering = rectangle.width / 2
    ctx.move_to(- rectangle.length / 2, - rectangle.width / 2)
    ctx.line_to(- rectangle.length / 2 + sheering, rectangle.width / 2)
    ctx.line_to(rectangle.length / 2 - sheering, rectangle.width / 2)
    ctx.line_to(rectangle.length / 2, - rectangle.width / 2)
    
    if add_color:#fill with black background color
        #add a small rectangle to black the whole blocked zone       
        ctx.move_to(- rectangle.length / 2, - rectangle.width / 2)
        ctx.line_to(rectangle.length / 2, - rectangle.width / 2)
        ctx.line_to(rectangle.length / 2, - rectangle.width / 2 - 0.025) #0.002
        ctx.line_to(-rectangle.length / 2, - rectangle.width / 2 - 0.025)
        ctx.line_to(-rectangle.length / 2, - rectangle.width / 2)
        
        ctx.close_path()
        ctx.clip_preserve()

        ctx.set_source_rgb(0.0, 0.0, 0.0)
        ctx.fill()
    else:#draw inner lines
        ctx.close_path()
        ctx.clip_preserve() 
                   
        start_x = - rectangle.length / 2 - rectangle.width
        end_x = rectangle.length / 2
        y_bottom = - rectangle.width / 2
        y_top = rectangle.width / 2
        ctx.set_line_width (0.02)
        for x in np.arange(start_x, end_x, 0.08):
            ctx.move_to(x, y_bottom)
            ctx.line_to(x + rectangle.width, y_top)  
        ctx.stroke()  
        

    ctx.restore()

def draw_zebra_crossing(ctx, lanelet, add_color):
    left = boundary_to_equi_distant(lanelet.leftBoundary, 0.04, 0.02)
    right = boundary_to_equi_distant(lanelet.rightBoundary, 0.04, 0.02)
    rightBoundary = lanelet.rightBoundary.point[0]
    leftBoundary = lanelet.leftBoundary.point[0]

    p1 = lanelet.lane.point[0]
    p2 = lanelet.lane.point[-1]

    lane = lanelet.lane
    flag = True
    ctx.save()

    if add_color:  
        dx = rightBoundary.x - leftBoundary.x
        dy = rightBoundary.y - leftBoundary.y

        ctx.set_line_width (math.sqrt(dx * dx + dy * dy)) 
        ctx.set_source_rgb((245.0/255.0), 0.0, (143.0/255.0))
        ctx.move_to(p1.x, p1.y)
        ctx.line_to(p2.x, p2.y)
        ctx.stroke()
    else:
        for (l, r) in zip(left, right):
            if flag:
                ctx.move_to(l[0], l[1])
                ctx.line_to(r[0], r[1])
                flag = False
            else:
                ctx.line_to(r[0], r[1])
                ctx.line_to(l[0], l[1])
                ctx.close_path()
                ctx.fill()
                flag = True
    ctx.restore()

def draw_starting_line(ctx, startingLine, add_color):
    a = 0.016

    for rect in startingLine.shape.rectangle:
        ctx.save()
        flag = True
        once = True

        ctx.translate(rect.centerPoint.x, rect.centerPoint.y)
        ctx.rotate(-rect.orientation)

        if add_color:
            ctx.set_line_width (0.048)
            ctx.set_source_rgb((165.0/255.0), (220.0/255.0), 0.0)
            ctx.move_to(0.0, -0.01)
            ctx.line_to(0.0, rect.length + 0.01)
            ctx.stroke()
        else:
            y = 0.01
            dy = a

            ctx.set_source_rgb(0, 0, 0)
            ctx.rectangle(-a - (a / 2.0) - 0.01, 0.01, rect.width + 0.02, rect.length - 0.02)
            ctx.fill()

            while y < rect.length:
                if (y + a) > rect.length:
                    dy = rect.length - y - 0.01

                if flag:
                    ctx.set_source_rgb(1, 1, 1)
                    ctx.rectangle(-a - (a / 2.0), y, a, dy)
                    ctx.rectangle((a / 2.0), y, a, dy)
                    ctx.fill()
                    y += a
                    flag = False
                else:
                    ctx.set_source_rgb(1, 1, 1)
                    ctx.rectangle(-(a / 2.0), y, a, dy)
                    ctx.fill()
                    y += a
                    flag = True
                
        ctx.restore()

def fill_intersection_center(ctx, center, add_color):
    if add_color:
        ctx.save()

        for rect in center.shape.rectangle:
            ctx.translate(rect.centerPoint.x, rect.centerPoint.y)
            ctx.rotate(-rect.orientation)
            ctx.rectangle(- rect.length / 2.0, - rect.width / 2.0, rect.length, rect.width)
            ctx.set_source_rgb(center.color.red, center.color.green, center.color.blue) 
            ctx.fill()
                
        ctx.restore()

def distance_points(p1, p2):
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return math.sqrt(dx*dx + dy*dy)

def boundary_length(boundary):
    length = 0
    for (p1, p2) in zip(boundary.point, boundary.point[1:]):
        length += distance_points(p1, p2)
    return length

def boundary_point_lengths(boundary):
    result = [0]
    len = 0
    for (p1, p2) in zip(boundary.point, boundary.point[1:]):
        len += distance_points(p1, p2)
        result.append(len)
    return result

def boundary_to_equi_distant(boundary, step_width, offset):
    lengths = boundary_point_lengths(boundary)
    x = list(map(lambda p: p.x, boundary.point))
    y = list(map(lambda p: p.y, boundary.point))
    eval_marks = np.arange(offset, lengths[-1], step_width)
    xinterp = np.interp(eval_marks, lengths, x)
    yinterp = np.interp(eval_marks, lengths, y)
    return map(lambda i: (i[0],i[1]), zip(xinterp, yinterp))

def draw_obstacle(ctx, obstacle, add_color):
    if (obstacle.type == "noParkingZone") and not add_color:
        draw_no_parking_zone(ctx, obstacle.shape)

    if (obstacle.type == "parkingLot") and add_color:
        draw_parking_lot(ctx, obstacle.shape)

    if obstacle.type == "parkingSpot":
        draw_parking_spot(ctx, obstacle.shape, add_color)

    if (obstacle.type == "perpendicularParkingObstacle") and not add_color:
        draw_parking_spot(ctx, obstacle.shape, False)

    if obstacle.type == "blockedArea":
        for rect in obstacle.shape.rectangle:
            draw_stripes_rect(ctx, rect, add_color)
    # uncomment if you want white boxes under the obstacles
    #else:
        #draw_shape(ctx, obstacle.shape)

def draw_all_boundaries(ctx, lanelet_list, boundary_name, target_dir, add_color, lane_color):
    all_ids = []
    for lanelet in lanelet_list:
        if getattr(lanelet, boundary_name).lineMarking is not None:
            if getattr(lanelet, boundary_name).color is not None:
                red = getattr(lanelet, boundary_name).color.red
                green = getattr(lanelet, boundary_name).color.green
                blue = getattr(lanelet, boundary_name).color.blue

            if lane_color == "orange":  
                all_ids.append(lanelet.id)
            elif lane_color == "green":  
                if (red == 51.0) and (green == (153.0/255.0)) and (blue == 0.0):  
                    all_ids.append(lanelet.id)            
            elif lane_color == "other":  
                if not ((red == (247.0/255.0)) and (green == (103.0/255.0)) and (blue == 0.0)) and not ((red == 51.0) and (green == (153.0/255.0)) and (blue == 0.0)):  
                    all_ids.append(lanelet.id)            
            else:
                all_ids.append(lanelet.id)

    state = 0
    while len(all_ids) > 0:
        points = []   
        current_id = all_ids[0]

        suc = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), boundary_name, "successor")        
        pred = expand_boundary(lanelet_list, get_lanelet_by_id(lanelet_list, current_id), boundary_name, "predecessor")

        ids_in_run = pred[::-1] + [current_id] + suc
                
        for id in ids_in_run:
            try:
                all_ids.remove(id)
            except:
                ids_in_run.remove(id)

        lanelets = list(map(lambda x: get_lanelet_by_id(lanelet_list, x), ids_in_run))
  
        ctx.save()
        current_type = get_lanelet_by_id(lanelet_list, current_id).type

        if getattr(lanelets[0], boundary_name).color is not None:
            red = getattr(lanelets[0], boundary_name).color.red
            green = getattr(lanelets[0], boundary_name).color.green
            blue = getattr(lanelets[0], boundary_name).color.blue
        else:
            red = 0.0
            green = 0.0
            blue = 0.0

        if add_color:
            if lane_color == "orange":
                ctx.set_source_rgb((247.0/255.0), (103.0/255.0), 0.0)
            else:
                ctx.set_source_rgb(red, green, blue)

            if (boundary_name == "lane"):
                ctx.set_line_width (0.4) 

            elif boundary_name == "rightBoundary":                       
                ctx.set_line_width(0.02) 

            elif boundary_name == "leftBoundary":
                line_marking = getattr(lanelets[0], boundary_name).lineMarking
                
                if (line_marking == "solid") or (line_marking == "dashed"):
                    ctx.set_line_width(0.02) 
                elif (line_marking == "double-solid") or (line_marking == "dashed-solid") or (line_marking == "solid-dashed"):
                    ctx.set_line_width(0.06)
            
        else:
            if boundary_name != "lane":
                lane_type = getattr(lanelets[0], "type")    
                line_marking = getattr(lanelets[0], boundary_name).lineMarking    
                ctx.set_source_rgb(1.0, 1.0, 1.0)
                ctx.set_line_width(0.02)

                if lane_type == "right_lane":
                    if (line_marking == "dashed") or (line_marking == "dashed-solid"):
                        ctx.set_dash([0.2, 0.2])
                    elif (line_marking == "solid") or (line_marking == "double-solid") or (line_marking == "solid-dashed"):
                        ctx.set_dash([])        
                else:
                    if (line_marking == "dashed") or (line_marking == "solid-dashed"):
                        ctx.set_dash([0.2, 0.2])
                    elif (line_marking == "solid") or (line_marking == "double-solid") or (line_marking == "dashed-solid"):
                        ctx.set_dash([])

            else:
                ctx.set_source_rgb(0.0, 0.0, 0.0)
                ctx.set_line_width(0.02)            

        ctx.move_to(getattr(lanelets[0], boundary_name).point[0].x,
            getattr(lanelets[0], boundary_name).point[0].y)

        for lanelet in lanelets:
            tmp = []

            for p in getattr(lanelet, boundary_name).point:
                ctx.line_to(p.x, p.y)

                if add_color and (boundary_name == "lane") and (lane_color != "orange"):
                    tmp.append([p.x, p.y])

            if add_color and (boundary_name == "lane") and (lane_color != "orange"):
                if len(tmp) == 2:
                    # x_end - x_start
                    dx = tmp[1][0] - tmp[0][0]
                    # y_end - y_start  
                    dy = tmp[1][1] - tmp[0][1]
                    d = math.sqrt(dx * dx + dy * dy)

                    dx = dx / d
                    dy = dy / d

                    t = 0.1
                    last = int(round(d / t, 0))

                    points.append([tmp[0][0], tmp[0][1]])
                    d = points[len(points) - 1]

                    for i in range(1, last):
                        points.append([points[len(points) - 1][0] + t * dx, points[len(points) - 1][1] + t * dy])
                else:
                    for i in range(0, len(tmp), 4):
                        points.append([tmp[i][0], tmp[i][1]])

        if add_color and (boundary_name == "lane") and (lane_color != "orange"):
            with open(path.join(target_dir, "trajectory", "trajectory_" + str(state) + ".txt"), "w+") as file:
                state += 1
                for i in range(len(points)):
                    file.write(str(points[i][0]) + "," + str(points[i][1]) + "\n")

        ctx.stroke()
        ctx.restore()

def draw_parking_diagonals(ctx, parkingLot, add_color):
    ctx.save()
    ctx.set_line_width(0.02)
    ctx.set_source_rgb(1, 1, 1)

    if len(parkingLot.startDiagonal) > 0:
        ctx.move_to(parkingLot.startDiagonal[0].x, parkingLot.startDiagonal[0].y)
        ctx.line_to(parkingLot.startDiagonal[1].x, parkingLot.startDiagonal[1].y) 

    if len(parkingLot.endDiagonal) > 0:
        ctx.move_to(parkingLot.endDiagonal[0].x, parkingLot.endDiagonal[0].y)
        ctx.line_to(parkingLot.endDiagonal[1].x, parkingLot.endDiagonal[1].y) 

    ctx.stroke()
    ctx.restore()

def get_lanelet_by_id(lanelet_list, id):
    for lanelet in lanelet_list:
        if lanelet.id == id:
            return lanelet
    return None

def expand_boundary(lanelet_list, lanelet, boundary_name, direction):
    ids = []
    original_line_type = getattr(lanelet, boundary_name).lineMarking
    found = True
    while found:
        found = False
        if getattr(lanelet, direction) is not None:
            for next in getattr(lanelet, direction).lanelet:
                next_lanelet = get_lanelet_by_id(lanelet_list, next.ref)
                if getattr(next_lanelet, boundary_name).lineMarking == original_line_type:
                    lanelet = next_lanelet
                    ids.append(lanelet.id)
                    found = True
                    break
    return ids

def draw(doc, target_dir, filename, add_color, concatenate_tiles, distance):
    bounding_box = utils.get_bounding_box(doc)
    bounding_box.x_min -= PADDING
    bounding_box.y_min -= PADDING
    bounding_box.x_max += PADDING
    bounding_box.y_max += PADDING

    width = math.ceil((bounding_box.x_max - bounding_box.x_min) * PIXEL_PER_UNIT)
    height = math.ceil((bounding_box.y_max - bounding_box.y_min) * PIXEL_PER_UNIT)

    width_num = math.ceil(width / TILE_SIZE)
    height_num = math.ceil(height / TILE_SIZE)

    os.makedirs(path.join(target_dir, filename, "materials", "textures"), exist_ok=True)
    os.makedirs(path.join(target_dir, filename, "materials", "scripts"), exist_ok=True)

    models = ""
    
    texture_names = []
    material_names = []
    format = ""
    filtering = "none"
    extra = ""
    offset = width_num

    if not add_color:
        format = "PF_L8"
        filtering = "anisotropic"
        extra = "max_anisotropy 16"
        offset = 0

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
        
        for center in doc.intersectionCenter:
            fill_intersection_center(ctx, center, add_color) 

        for prakingLot in doc.parkingLot:
            draw_parking_diagonals(ctx, prakingLot, add_color)  

        # for traffic in doc.trafficSign:
        #     ctx.save()

        #     ctx.translate(traffic.centerPoint.x, traffic.centerPoint.y)
        #     ctx.rectangle(-0.05, -0.05, 0.1, 0.1)
        #     ctx.set_source_rgb(1.0, 1.0, 1.0) 
        #     ctx.fill()     
                
        #     ctx.restore()  

        if add_color:
            draw_all_boundaries(ctx, doc.lanelet, "rightBoundary", target_dir, add_color, "orange")
            draw_all_boundaries(ctx, doc.lanelet, "lane", target_dir, add_color, "orange")
            draw_all_boundaries(ctx, doc.lanelet, "rightBoundary", target_dir, add_color, "green")
            draw_all_boundaries(ctx, doc.lanelet, "lane", target_dir, add_color, "green")
            draw_all_boundaries(ctx, doc.lanelet, "rightBoundary", target_dir, add_color, "other")
            draw_all_boundaries(ctx, doc.lanelet, "lane", target_dir, add_color, "other")
        else:
            draw_all_boundaries(ctx, doc.lanelet, "rightBoundary", target_dir, add_color, None) 
            draw_all_boundaries(ctx, doc.lanelet, "lane", target_dir, add_color, None)  

        draw_all_boundaries(ctx, doc.lanelet, "leftBoundary", target_dir, add_color, None)    

        for lanelet in doc.lanelet:
            draw_stop_line(ctx, lanelet, add_color)
            if lanelet.type == "zebraCrossing":
                draw_zebra_crossing(ctx, lanelet, add_color)

        for startingLine in doc.startingLine:
            draw_starting_line(ctx, startingLine, add_color)

        for obstacle in doc.obstacle:
            draw_obstacle(ctx, obstacle, add_color)

        for island_junction in doc.islandJunction:
            draw_island_junction(ctx, island_junction)

        for road_marking in doc.roadMarking:
            draw_road_marking(ctx, road_marking)

        sha_256 = hashlib.sha256()
        sha_256.update(surface.get_data())
        hash = sha_256.hexdigest()

        texture_file = "tile-{0}.png".format(hash)
        material_file = "tile-{0}.material".format(hash)

        x_pos = bounding_box.x_min + (x + 0.5) * TILE_SIZE / PIXEL_PER_UNIT
        y_pos = (bounding_box.y_min + (y + 0.5) * TILE_SIZE / PIXEL_PER_UNIT) + distance

        texture_names.append(texture_file) 
        material_names.append(material_file) 

        if (x == 0) and (y == 0):
            start = [x_pos, y_pos]       
        
        if (x == (width_num - 1)) and (y == (height_num - 1)):
            end = [x_pos, y_pos]

        # die Verzeichnisse raw und colored erstellen
        surface.write_to_png(
            path.join(target_dir, filename, "materials", "textures", texture_file))

        with open(path.join(target_dir, filename, "materials", "scripts", material_file), "w") as file:
            file.write(ground_plane_material("Tile/" + hash, texture_file, format, filtering, extra))

        models += ground_plane_model(
            x_pos,
            y_pos, 
            TILE_SIZE / PIXEL_PER_UNIT,
            TILE_SIZE / PIXEL_PER_UNIT,
            "Tile/{0}-{1}".format(x + offset, y),
            "Tile/" + hash)

    # secait00
    if concatenate_tiles:
        dst = concatenate.concate_tiles(target_dir, filename, texture_names, width_num, height_num)
        
        if add_color:
            hash = str(2).zfill(64)
        else:
            hash = str(1).zfill(64)

        texture_file = "tile-{0}.png".format(hash)
        material_file = "tile-{0}.material".format(hash)

        dst.save(path.join(os.getcwd(), target_dir, filename, "materials", "textures", texture_file))

        with open(path.join(target_dir, filename, "materials", "scripts",  material_file), "w") as file:
            file.write(ground_plane_material("Tile/" + hash, texture_file, format, filtering, extra)) 

        x_pos = start[0] + ((end[0] - start[0]) / 2.0)
        y_pos = start[1] + ((end[1] - start[1]) / 2.0)
        x = 0
        y = 0

        models = ground_plane_model(
            x_pos,
            y_pos, 
            (TILE_SIZE / PIXEL_PER_UNIT) * width_num,
            (TILE_SIZE / PIXEL_PER_UNIT) * height_num,
            "Tile/{0}-{1}".format(x + offset, y),
            "Tile/" + hash)
    #

    return models

def ground_plane_material(name, file, format, filtering, extra):
    return """
    material {name}
    {{
        technique
        {{
            pass
            {{
                texture_unit
                {{
                    texture {file} {format}
                    filtering {filtering}
                    {extra}
                }}
            }}
        }}
    }}
    """.format(name=name, file=file, format=format, filtering=filtering, extra=extra)

def ground_plane_model(x, y, tile_size_w, tile_size_h, name, material): 
    return """
    <model name='{name}'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{tile_size_w} {tile_size_h}</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{tile_size_w} {tile_size_h}</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://materials/scripts</uri>
              <uri>file://materials/textures</uri>
              <name>{material}</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>{x} {y} 0 0 -0 0</pose>
    </model>
    """.format(x=x, y=y, tile_size_w=tile_size_w, tile_size_h=tile_size_h, name=name, material=material)
