from blender.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad import schema
from os import path, makedirs


def generate_sdf(xml_content, target_dir, add_vehicle):
    doc = schema.CreateFromDocument(xml_content)

    content = groundplane.draw(doc, target_dir)
    if add_vehicle:
        content += ego_vehicle.draw(target_dir, doc.lanelet)
    for obst in doc.obstacle:
        if obst.type != "blockedArea":
            content += obstacle.draw(obst)
    for sign in doc.trafficSign:
        content += traffic_sign.draw(sign, target_dir)
    for ramp in doc.ramp:
        content += special_objects.draw_ramp(ramp.centerPoint.x, ramp.centerPoint.y, ramp.orientation, ramp.id)

    if not path.exists(path.join(target_dir, "worlds")):
        makedirs(path.join(target_dir, "worlds"))

    with open(path.join(target_dir, "worlds", "world.sdf"), "w+") as file:
        file.write("<sdf version='1.6'><world name='default'>")
        file.write(sun_light())
        file.write(spot_light(pose=[0, 0, 0.2, 0, 0, 0], idx=0))
        file.write(sky())
        file.write(content)
        file.write("</world></sdf>")