from commonroad.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad.generator import road_generation
from commonroad import schema
from os import path, makedirs

def generate_sdf(xml_content, target_dir, add_vehicle):
    doc = schema.CreateFromDocument(xml_content)

    content = groundplane.draw(doc, target_dir)
    if add_vehicle:
        print('Ego vehicle ', add_vehicle)
        print('Drawing ego vehicle')
        content += ego_vehicle.draw(target_dir, doc.lanelet)
    for obst in doc.obstacle:
        if obst.type != "blockedArea":
            content += obstacle.draw(obst)
    for sign in doc.trafficSign:
        content += traffic_sign.draw(sign, target_dir)
        print(type(sign))
    for ramp in doc.ramp:
        print('Ramp', ramp, 'in world', dir(ramp))
        sign_start = schema.trafficSign
        sign_start.id = ramp.id+'_108-10'
        sign_start.type = 'stvo-108-10'
        sign_start.centerPoint.x = ramp.centerPoint.x
        sign_start.centerPoint.y = ramp.centerPoint.y + c
        content += traffic_sign.draw(sign_start, target_dir)
        content += special_objects.draw_ramp(ramp.centerPoint.x, ramp.centerPoint.y, ramp.orientation, ramp.id)
        sign_start = schema.trafficSign
        sign_start.id = ramp.id+'_108-10'
        sign_start.type = 'stvo-108-10'
        content += traffic_sign.draw(sign, target_dir)

    if not path.exists(path.join(target_dir, "worlds")):
        makedirs(path.join(target_dir, "worlds"))

    with open(path.join(target_dir, "worlds", "world.sdf"), "w+") as file:
        file.write("<sdf version='1.6'><world name='default'>")
        file.write(sun_light())
        file.write(content)
        file.write("</world></sdf>")

def sun_light():
    return """
    <light name='sun_light' type='directional'>
      <pose frame=''>0 0 10 0 -0 0</pose>
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
    """
