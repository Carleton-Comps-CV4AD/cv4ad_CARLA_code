#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Lidar projection on RGB camera example
"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from queue import Queue
from queue import Empty

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    from PIL import Image
except ImportError:
    raise RuntimeError('cannot import PIL, make sure "Pillow" package is installed')

def get_distances(p_cloud):
    coordinates = np.array(p_cloud[:, :3])
    distances = np.linalg.norm(coordinates, axis=1)
    return distances

def project(image_data, lidar_data, camera: carla.Actor, camera_bp: carla.ActorBlueprint, lidar: carla.Actor, 
            camera_transform: carla.Transform, lidar_transform: carla.Transform):
    # Build the K projection matrix:
    # K = [[Fx,  0, image_w/2],
    #      [ 0, Fy, image_h/2],
    #      [ 0,  0,         1]]
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()
    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

    # In this case Fx and Fy are the same since the pixel aspect
    # ratio is 1
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0

    # Get the raw BGRA buffer and convert it to an array of RGB of
    # shape (image_data.height, image_data.width, 3).
    im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
    im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
    im_array = im_array[:, :, :3][:, :, ::-1]

    # Get the lidar data and convert it to a numpy array.
    p_cloud_size = len(lidar_data)
    p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
    p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

    # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
    # focus on the 3D points.
    intensity = np.array(p_cloud[:, 3])
    distance = get_distances(p_cloud)

    assert intensity.shape == distance.shape

    # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
    local_lidar_points = np.array(p_cloud[:, :3]).T

    # Add an extra 1.0 at the end of each 3d point so it becomes of
    # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
    local_lidar_points = np.r_[
        local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

    # This (4, 4) matrix transforms the points from lidar space to world space.
    # lidar_2_world = lidar.get_transform().get_matrix()
    lidar_2_world = camera.get_transform().get_matrix()
    # lidar_2_world = lidar_transform.get_matrix()

    # Transform the points from lidar space to world space.
    world_points = np.dot(lidar_2_world, local_lidar_points)

    # This (4, 4) matrix transforms the points from world to sensor coordinates.
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    # world_2_camera = np.array(camera_transform.get_inverse_matrix())

    # Transform the points from world space to camera space.
    sensor_points = np.dot(world_2_camera, world_points)

    # New we must change from UE4's coordinate system to an "standard"
    # camera coordinate system (the same used by OpenCV):

    # ^ z                       . z
    # |                        /
    # |              to:      +-------> x
    # | . x                   |
    # |/                      |
    # +-------> y             v y

    # This can be achieved by multiplying by the following matrix:
    # [[ 0,  1,  0 ],
    #  [ 0,  0, -1 ],
    #  [ 1,  0,  0 ]]

    # Or, in this case, is the same as swapping:
    # (x, y ,z) -> (y, -z, x)
    point_in_camera_coords = np.array([
        sensor_points[1],
        sensor_points[2] * -1,
        sensor_points[0]])

    # Finally we can use our K matrix to do the actual 3D -> 2D.
    points_2d = np.dot(K, point_in_camera_coords)

    # Remember to normalize the x, y values by the 3rd value.
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]])

    # At this point, points_2d[0, :] contains all the x and points_2d[1, :]
    # contains all the y values of our points. In order to properly
    # visualize everything on a screen, the points that are out of the screen
    # must be discarted, the same with points behind the camera projection plane.
    points_2d = points_2d.T

    intensity = intensity.T
    distance = distance.T
    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
        (points_2d[:, 2] > 0.0)
    points_2d = points_2d[points_in_canvas_mask]
    intensity = intensity[points_in_canvas_mask]
    distance = distance[points_in_canvas_mask]

    # Extract the screen coords (uv) as integers.
    u_coord = points_2d[:, 0].astype(np.int)
    v_coord = points_2d[:, 1].astype(np.int)

    # Since at the time of the creation of this script, the intensity function
    # is returning high values, these are adjusted to be nicely visualized.
    intensity = 4 * intensity - 3


    lidar_data_array = np.stack((v_coord, u_coord, intensity, distance), axis=0)

    return lidar_data_array
