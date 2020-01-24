from blender.renderer import groundplane, obstacle, traffic_sign, ego_vehicle, special_objects
# we assume that the road width config set here is the same used during the generation
from commonroad import schema
from os import path, makedirs
import bpy
import os
from commonroad.renderer.ego_vehicle import get_start_lanelet, get_next_lanelet, middle_of_lanelet, middle_of_lanelet_equidistant
import math
from tqdm import tqdm
from blender.renderer import env_configs
import random
import cv2


def get_keyframes(lanelets, config):
    current_lanelet = get_start_lanelet(lanelets)
    t = 0
    last_point = None
    keyframes = []
    while current_lanelet is not None:
        middle = middle_of_lanelet_equidistant(current_lanelet, step_length=config['render_interval_distance'])
        for p in middle:
            if last_point is not None:
                orientation = math.atan2(p[1] - last_point[1], p[0] - last_point[0])
            else:
                orientation = 0
            keyframes.append({'t': t, 'x': p[0], 'y': p[1], 'z': 0, 'orientation': orientation})
            if last_point is not None:
                dx = last_point[0] - p[0]
                dy = last_point[1] - p[1]
                t += math.sqrt(dx * dx + dy * dy)
            last_point = p
        current_lanelet = get_next_lanelet(lanelets, current_lanelet)
    return keyframes


def setup_env(scene_rgb, scene_seg, scene_lanes, env_config):
    scene_seg.render.engine = 'BLENDER_RENDER'
    scene_lanes.render.engine = 'BLENDER_RENDER'

    scene_rgb.render.engine = 'CYCLES'
    scene_rgb.cycles.device = 'GPU'
    scene_rgb.cycles.samples = 350
    scene_rgb.cycles.preview_samples = 64
    scene_rgb.render.layers[0].cycles.use_denoising = True
    world = bpy.data.worlds['World']
    world.use_nodes = True
    nt = world.node_tree
    bg_node = nt.nodes.new(type="ShaderNodeTexEnvironment")
    bg_node.image = bpy.data.images.load(env_config['image_path'])
    scene_rgb.world = world

    # find location of Background node and position Grad node to the left
    backNode = nt.nodes['Background']
    bg_node.location.x = backNode.location.x - 300
    bg_node.location.y = backNode.location.y

    mapping_node = nt.nodes.new(type="ShaderNodeMapping")
    mapping_node.rotation = env_config['rotation']
    mapping_node.scale = env_config['scale']
    mapping_node.translation = env_config['translation']
    nt.links.new(mapping_node.outputs['Vector'], bg_node.inputs['Vector'])

    coord_node = nt.nodes.new(type="ShaderNodeTexCoord")
    nt.links.new(coord_node.outputs['Generated'], mapping_node.inputs['Vector'])

    gradColOut = bg_node.outputs['Color']
    backColIn = backNode.inputs['Color']
    nt.links.new(gradColOut, backColIn)

    # add instance id output
    scene_seg.render.layers["RenderLayer"].use_pass_combined = False
    scene_seg.render.layers["RenderLayer"].use_pass_z = False
    scene_seg.render.layers["RenderLayer"].use_pass_diffuse = False
    scene_seg.render.layers["RenderLayer"].use_pass_color = True
    scene_seg.render.layers["RenderLayer"].use_pass_object_index =   False
    scene_seg.render.layers["RenderLayer"].use_pass_material_index = True
    scene_seg.use_nodes = True
    seg_tree = scene_seg.node_tree
    instance_out_node = seg_tree.nodes.new(type="CompositorNodeOutputFile")
    instance_out_node.name = 'InstanceOutput'
    instance_out_node.format.compression = 0
    instance_out_node.file_slots[0].path = 'Image.exr'
    instance_out_node.format.file_format = 'OPEN_EXR'
    seg_tree.links.new(seg_tree.nodes['Render Layers'].outputs['Color'], seg_tree.nodes['Composite'].inputs['Image'])
    seg_tree.links.new(seg_tree.nodes['Render Layers'].outputs['IndexMA'], instance_out_node.inputs['Image'])


def render_keyframes(lanelets, output_path, scene_rgb, scene_seg, scene_lanes, config, add_vehicle=True,
                     car_name='ego_vehicle'):
    if config['use_vehicle_mask']:
        vehicle_masks = []

    for camera_config in config['cameras']:
        bpy.ops.object.camera_add(view_align=True, location=(0, 0, 0), rotation=(0, 0, 0))
        cam_name = camera_config['name']
        bpy.data.cameras[bpy.context.active_object.name].name = cam_name
        bpy.context.active_object.name = cam_name
        bpy.data.cameras[cam_name].clip_start = 0.05  # to prevent clipping of vehicle body
        bpy.data.cameras[cam_name].sensor_fit = 'HORIZONTAL'
        bpy.data.cameras[cam_name].sensor_width = camera_config['sensor_width']
        bpy.data.cameras[cam_name].sensor_height = camera_config['sensor_height']
        if 'focal_length' in camera_config:
            bpy.data.cameras[cam_name].lens = camera_config['focal_length']
        elif 'horizontal_fov' in camera_config:
            bpy.data.cameras[cam_name].lens_unit = 'FOV'
            bpy.data.objects[cam_name].data.angle_x = camera_config['horizontal_fov'] * math.pi/180
        else:
            print('No lens config provided for camera \'{}\', using defaults'.format(cam_name))

        camera_path = os.path.join(output_path, cam_name)

        if not os.path.isdir(os.path.join(camera_path, 'rgb')):
            os.makedirs(os.path.join(camera_path, 'rgb'))

        if not os.path.isdir(os.path.join(camera_path, 'semseg_color')):
            os.makedirs(os.path.join(camera_path, 'semseg_color'))

        if not os.path.isdir(os.path.join(camera_path, 'traffic_sign_id')):
            os.makedirs(os.path.join(camera_path, 'traffic_sign_id'))

        if not os.path.isdir(os.path.join(camera_path, 'lane_segmentation')):
            os.makedirs(os.path.join(camera_path, 'lane_segmentation'))

        if config['use_vehicle_mask']:
            vehicle_masks.append(cv2.imread(os.path.join('blender', 'segmentation_masks',
                                                         camera_config['segmentation_mask']),
                                            cv2.IMREAD_UNCHANGED))

    keyframes = get_keyframes(lanelets, config)

    seg_tree = scene_seg.node_tree
    seg_tree.nodes['Render Layers'].scene = scene_seg

    instance_out_node = seg_tree.nodes['InstanceOutput']
    seg_tree.links.new(seg_tree.nodes['Render Layers'].outputs['IndexMA'], instance_out_node.inputs['Image'])

    scene_rgb.render.use_file_extension = False
    scene_seg.render.use_file_extension = False
    scene_seg.display_settings.display_device = 'None'
    scene_lanes.display_settings.display_device = 'None'

    bpy.context.user_preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'

    for idx, keyframe in enumerate(tqdm(keyframes[config['frame_range'][0]: config['frame_range'][1]])):
        name_idx = idx + config['frame_range'][0]
        pos_x = keyframe['x']
        pos_y = keyframe['y']
        orientation = keyframe['orientation']
        if config['add_random_jitter']:
            pos_x = keyframe['x'] + random.uniform(-config['random_jitter']['x'], config['random_jitter']['x'])
            pos_y = keyframe['y'] + random.uniform(-config['random_jitter']['y'], config['random_jitter']['y'])
            orientation = keyframe['orientation'] + random.uniform(-config['random_jitter']['angle'],
                                                                   config['random_jitter']['angle']) * math.pi / 180

        if add_vehicle:
            car = bpy.data.objects[car_name]
            car.location = (pos_x,
                            pos_y,
                            0)
            car.rotation_euler = [0, 0, orientation + math.pi]

            if not config['use_vehicle_mask']:
                seg_car = bpy.data.objects['seg-' + car_name]
                seg_car.location = (pos_x,
                                    pos_y,
                                    0)
                seg_car.rotation_euler = [0, 0, orientation + math.pi]

        for cam_idx, camera_config in enumerate(config['cameras']):
            cam_name = camera_config['name']
            print('Rendering camera: {}'.format(cam_name))
            camera = bpy.data.objects[cam_name]
            scene_rgb.camera = camera
            scene_seg.camera = camera
            scene_lanes.camera = camera

            camera_path = os.path.join(output_path, cam_name)
            resolution = camera_config['image_resolution']
            scene_rgb.render.resolution_x = resolution[0]
            scene_rgb.render.resolution_y = resolution[1]
            scene_rgb.render.image_settings.compression = 0
            scene_rgb.render.resolution_percentage = 100

            scene_seg.render.resolution_x = resolution[0]
            scene_seg.render.resolution_y = resolution[1]
            scene_seg.render.image_settings.compression = 0
            scene_seg.render.resolution_percentage = 100

            scene_lanes.render.resolution_x = resolution[0]
            scene_lanes.render.resolution_y = resolution[1]
            scene_lanes.render.image_settings.compression = 0
            scene_lanes.render.resolution_percentage = 100

            camera_offset = camera_config['position_offset']
            camera_orientation_y = camera_config['rotation'] * math.pi/180.0

            bpy.context.screen.scene = scene_rgb
            scene_rgb.render.layers["RenderLayer"].use_pass_combined = True
            scene_rgb.render.layers["RenderLayer"].use_pass_z = True
            scene_rgb.render.layers["RenderLayer"].use_pass_diffuse = False

            camera.location = (pos_x + camera_offset['x'] * math.cos(orientation) - camera_offset['y'] * math.sin(orientation),
                               pos_y + camera_offset['x'] * math.sin(orientation) + camera_offset['y'] * math.cos(orientation),
                               camera_offset['z'])
            camera.rotation_euler = [-math.pi/2 - camera_orientation_y, math.pi, orientation + math.pi/2]
            if 'rgb' in config['render_passes']:
                scene_rgb.render.filepath = os.path.join(camera_path, 'rgb', 'Image{:04d}.png'.format(name_idx+1))
                bpy.ops.render.render(write_still=True)

            instance_out_node = seg_tree.nodes['InstanceOutput']
            instance_out_node.base_path = os.path.join(camera_path, 'traffic_sign_id')

            # activate diffuse pass only for scene
            if 'semseg_color' in config['render_passes'] or 'instances' in config['render_passes']:
                bpy.context.screen.scene = scene_seg
                scene_seg.render.filepath = os.path.join(camera_path, 'semseg_color', 'Image{:04d}.png'.format(
                    name_idx+1))
                bpy.ops.render.render(write_still=True)
                # blender seems to insist of plastering the frame number at the end of the file. fix manually
                os.rename(os.path.join(camera_path, 'traffic_sign_id', 'Image.exr0001'),
                          os.path.join(camera_path, 'traffic_sign_id', 'Image{:04d}.exr'.format(name_idx+1)))

                # draw vehicle mask
                if config['use_vehicle_mask']:
                    vehicle_mask = vehicle_masks[cam_idx]
                    rendered_image = cv2.imread(scene_seg.render.filepath)
                    rendered_image[vehicle_mask[..., -1] != 0] = vehicle_mask[vehicle_mask[..., -1] != 0][..., :-1]
                    cv2.imwrite(scene_seg.render.filepath, rendered_image)

            if 'lanes' in config['render_passes']:
                bpy.context.screen.scene = scene_lanes
                scene_lanes.render.filepath = os.path.join(camera_path, 'lane_segmentation',
                                                           'Image{:04d}.png'.format(name_idx+1))
                bpy.ops.render.render(write_still=True)


def generate_blend(xml_content, target_dir, add_vehicle, output_dir, gazebo_world_path, gazebo_sim_path, config):
    # delete box in scene originally
    doc = schema.CreateFromDocument(xml_content)

    bpy.ops.object.delete(use_global=False)
    scene_rgb = bpy.data.scenes.new("RGB")
    scene_segmentation = bpy.data.scenes.new("Semantic Segmentation")
    scene_lanes = bpy.data.scenes.new("Lane Segmentation")

    scenes = {'rgb': scene_rgb, 'segmentation': scene_segmentation, 'lane_detection': scene_lanes}

    groundplane.draw(doc, target_dir, scenes, obstacle, config)
    if add_vehicle:
        # render keyframes at the end by moving the object and calling bpy
        ego_vehicle.draw(gazebo_sim_path, scene_rgb, scene_segmentation, config)
    for obst in doc.obstacle:
        if obst.type != "blockedArea" and obst.type != "segmentationIntersection":
            obstacle.draw(obst, scene_rgb, scene_segmentation)
    mesh_basepath = os.path.join(gazebo_world_path, 'meshes')
    for sign_idx, sign in enumerate(doc.trafficSign):
        traffic_sign.draw(sign, mesh_basepath, scene_rgb, scene_segmentation, sign_idx=sign_idx+1)
    for ramp in doc.ramp:
        special_objects.draw_ramp(ramp, mesh_basepath, scene_rgb, scene_segmentation)

    setup_env(scene_rgb, scene_segmentation, scene_lanes, getattr(env_configs, config['env_config']))
    render_keyframes(doc.lanelet, output_dir, scene_rgb, scene_segmentation, scene_lanes,
                     config, add_vehicle=add_vehicle)
    # bpy.ops.wm.save_mainfile(os.path.join(output_dir, 'render_scene.blend'), compress=False)
