import asyncio
import glob
import math
import os
import sys
import threading
import time

import numpy as np
from matplotlib import pyplot as plt
import torchvision.transforms as transforms
from PIL import Image
from ultralytics import YOLO

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def process_image(image):
    image[0] = carla.Color(255, 255, 255, 0)
    print(image[0])


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # destroy existing vehicles and walkers
    for vehicle in world.get_actors().filter('vehicle.*'):
        vehicle.destroy()
    for walker in world.get_actors().filter('walker.pedestrian*'):
        walker.destroy()

    # choose blueprint of cars
    blueprint_library = world.get_blueprint_library()
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
    car_bp = car_blueprints[0]

    # spectator location
    spectator_location = carla.Location(x=-48.87, y=118.53, z=0.00)
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(spectator_location + carla.Location(z=20), carla.Rotation(pitch=-90)))

    # camera sensor
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '640')
    camera_bp.set_attribute('fov', '90')

    # spawn vehicles
    vehicle_location = carla.Location(x=-48.87, y=40.53, z=-0.00)
    vehicle_rotation = carla.Rotation(pitch=0.00, yaw=90.00, roll=0.00)
    vehicle_spawn_point = carla.Transform(location=vehicle_location, rotation=vehicle_rotation)
    vehicle = world.spawn_actor(car_bp, vehicle_spawn_point)
    print("Vehicle enabled")
    camera_point = carla.Location(x=2.5, z=0.7)
    camera_transform = carla.Transform(camera_point)
    camera_sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    vehicle.set_autopilot(True)
    camera_sensor.listen(lambda image: process_image(image))


    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        if camera_sensor.is_listening:
            camera_sensor.destroy()
        vehicle.destroy()

if __name__ == '__main__':
    main()