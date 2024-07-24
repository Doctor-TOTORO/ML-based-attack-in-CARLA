import time

import carla
import random
import cv2
import torchvision.transforms as transforms
from PIL import Image
from simba import simba_single
from ultralytics import YOLO

import numpy as np

def process_image(image):
    return image



def process_image1(image):
    return


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # destroy existing vehicles and walkers
    for vehicle in world.get_actors().filter('vehicle.*'):
        vehicle.destroy()
    for walker in world.get_actors().filter('walker.pedestrian*'):
        walker.destroy()

    # choose blueprint of cars
    blueprint_library = world.get_blueprint_library()
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if
                      int(bp.get_attribute('number_of_wheels')) == 4]
    car_bp = car_blueprints[0]

    # spawn vehicles
    vehicle_location = carla.Location(x=-48.87, y=40.53, z=-0.00)
    vehicle1_location = carla.Location(x=-43.87, y=38.53, z=-0.00)
    vehicle_rotation = carla.Rotation(pitch=0.00, yaw=90.00, roll=0.00)
    vehicle1_rotation = carla.Rotation(pitch=0.00, yaw=0.00, roll=0.00)
    vehicle_spawn_point = carla.Transform(location=vehicle_location, rotation=vehicle_rotation)
    vehicle1_spawn_point = carla.Transform(location=vehicle1_location, rotation=vehicle1_rotation)
    vehicle1 = world.spawn_actor(car_bp, vehicle1_spawn_point)
    vehicle = world.spawn_actor(car_bp, vehicle_spawn_point)
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '640')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera1 = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle1)
    camera1.listen(lambda image: process_image1(image))
    camera.listen(lambda image: process_image(image))

    # spectator location
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(vehicle_location + carla.Location(z=20), carla.Rotation(pitch=-90.00)))
    vehicle.set_autopilot(True)


    try:
        while True:
            spectator.set_transform(
                carla.Transform(vehicle.get_transform().location + carla.Location(z=20), carla.Rotation(pitch=-90.00)))
            world.wait_for_tick()
            time.sleep(0.01)
    except KeyboardInterrupt:
        vehicle.destroy()
        camera.destroy()
        camera1.destroy()

if __name__ == '__main__':
    main()