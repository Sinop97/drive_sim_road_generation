from blender.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad import schema
from os import path, makedirs


def generate_blend(xml_content, target_dir, add_vehicle):
    doc = schema.CreateFromDocument(xml_content)

    groundplane.draw(doc, target_dir)
    if add_vehicle:
        ego_vehicle.draw(target_dir, doc.lanelet)
    for obst in doc.obstacle:
        if obst.type != "blockedArea":
            obstacle.draw(obst)
    for sign in doc.trafficSign:
        traffic_sign.draw(sign, target_dir)
    for ramp in doc.ramp:
        special_objects.draw_ramp(ramp.centerPoint.x, ramp.centerPoint.y, ramp.orientation, ramp.id)

    if not path.exists(path.join(target_dir, "worlds")):
        makedirs(path.join(target_dir, "worlds"))
