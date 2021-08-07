from commonroad.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad.generator import road_generation
from commonroad import schema
from os import path, makedirs

def generate_sdf(xml_content, target_dir, add_vehicle, add_color, concatenate_tiles, distance):
    doc = schema.CreateFromDocument(xml_content)

    if add_color:
        filename = "colored"
    else:
        filename = "raw"

    if not path.exists(path.join(target_dir, filename)):
        makedirs(path.join(target_dir, filename))

    if not path.exists(path.join(target_dir, "trajectory")):
        makedirs(path.join(target_dir, "trajectory"))

    content = groundplane.draw(doc, target_dir, filename, add_color, concatenate_tiles, distance)
    if add_vehicle:
        content += ego_vehicle.draw(target_dir, doc.lanelet)
    for obst in doc.obstacle:
        if (obst.type != "blockedArea") and (obst.type != "parkingLot") and (obst.type != "parkingSpot") and (obst.type != "noParkingZone"):
            content += obstacle.draw(obst, add_color, distance)
    for sign in doc.trafficSign:
        content += traffic_sign.draw(sign, target_dir, add_color, distance)
    for ramp in doc.ramp:
        content += special_objects.draw_ramp(ramp.centerPoint.x, ramp.centerPoint.y, ramp.orientation, ramp.id)

    if not path.exists(path.join(target_dir, filename, "worlds")):
        makedirs(path.join(target_dir, filename, "worlds"))

    with open(path.join(target_dir, filename, "worlds", "world.sdf"), "w+") as file:
        file.write("<sdf version='1.6'><world name='default'>")

        name = 'sun_light'

        if add_color:
            name = 'sun_light_colored'

        file.write(sun_light(name, int(distance)))
        file.write(content)
        file.write("</world></sdf>")

def sun_light(name, y):
    return """
    <light name={name} type='directional'>
      <pose frame=''>0 {y} 10 0 -0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.1 0.1 -0.9</direction>
      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
    </light>
    """.format(name=name, y=y)
