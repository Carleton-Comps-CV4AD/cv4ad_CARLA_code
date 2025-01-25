import numpy as np
import cv2
from ego_vehicle import Camera

def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

def configure_matrices(camera: Camera):
    camera_bp = camera.camera_blueprint
    carla_camera = camera.camera
    # Get the world to camera matrix
    world_to_camera = np.array(camera.transform_at_last_image.get_inverse_matrix())

    # Get the attributes from the camera
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    # Calculate the camera projection matrix to project from 3D -> 2D
    K = build_projection_matrix(image_w, image_h, fov)
    K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

    return world_to_camera, K, K_b

def ___get_bb_img(world, ego, img, camera: Camera):
    world_to_camera, K, K_b = configure_matrices(camera)
    img = np.reshape(np.copy(img.raw_data), (img.height, img.width, 4))

    for npc in world.get_actors().filter('*vehicle*'):

        # Filter out the ego vehicle
        if npc.id != ego.id:

            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(ego.get_transform().location)

            # Filter for the vehicles within 50m
            if dist < 100:

            # Calculate the dot product between the forward vector
            # of the vehicle and the vector between the vehicle
            # and the other vehicle. We threshold this dot product
            # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = ego.get_transform().get_forward_vector()
                ray = npc.get_transform().location - ego.get_transform().location

                if forward_vec.dot(ray) > 0:
                    p1 = get_image_point(bb.location, K, world_to_camera)
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    x_max = -10000
                    x_min = 10000
                    y_max = -10000
                    y_min = 10000

                    for vert in verts:
                        p = get_image_point(vert, K, world_to_camera)
                        # Find the rightmost vertex
                        if p[0] > x_max:
                            x_max = p[0]
                        # Find the leftmost vertex
                        if p[0] < x_min:
                            x_min = p[0]
                        # Find the highest vertex
                        if p[1] > y_max:
                            y_max = p[1]
                        # Find the lowest  vertex
                        if p[1] < y_min:
                            y_min = p[1]

                    cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
    return img

def get_bb_img(world, ego, img, camera: Camera):
    world_to_camera, K, K_b = configure_matrices(camera)
    img = np.reshape(np.copy(img.raw_data), (img.height, img.width, 4))
    ego_transform = None
    for id, _, transform in camera.world_vehicles_locations_at_last_image:
        if id == ego.id:
            ego_transform = transform
            break
    assert ego_transform is not None

    for id, bb, transform in camera.world_vehicles_locations_at_last_image:

        # Filter out the ego vehicle
        if id != ego.id:

            dist = transform.location.distance(ego_transform.location)

            # Filter for the vehicles within 50m
            if dist < 100:

            # Calculate the dot product between the forward vector
            # of the vehicle and the vector between the vehicle
            # and the other vehicle. We threshold this dot product
            # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = ego_transform.get_forward_vector()
                ray = transform.location - ego_transform.location

                if forward_vec.dot(ray) > 0:
                    p1 = get_image_point(bb.location, K, world_to_camera)
                    verts = [v for v in bb.get_world_vertices(transform)]
                    x_max = -10000
                    x_min = 10000
                    y_max = -10000
                    y_min = 10000

                    for vert in verts:
                        p = get_image_point(vert, K, world_to_camera)
                        # Find the rightmost vertex
                        if p[0] > x_max:
                            x_max = p[0]
                        # Find the leftmost vertex
                        if p[0] < x_min:
                            x_min = p[0]
                        # Find the highest vertex
                        if p[1] > y_max:
                            y_max = p[1]
                        # Find the lowest  vertex
                        if p[1] < y_min:
                            y_min = p[1]

                    cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
    return img