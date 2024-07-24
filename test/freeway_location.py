import glob
import math
import os
import sys
import threading
import time
import matplotlib.pyplot as plt
import numpy as np
import torchvision.transforms as transforms
from PIL import Image
from ultralytics import YOLO

from carla import Transform

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

subfolder_path1 = os.path.join(os.getcwd(), 'freeway_attack', 'simba', 'v1')
subfolder_path2 = os.path.join(os.getcwd(), 'freeway_attack', 'simba', 'v2')
subfolder_attack = os.path.join(os.getcwd(), 'freeway_attack', 'simba', 'normal')
count = 0

def get_distance(vehicle1, vehicle2):
    location1 = vehicle1.get_location()
    location2 = vehicle2.get_location()
    return math.sqrt((location1.x - location2.x)**2 + (location1.y - location2.y)**2 + (location1.z - location2.z)**2)

def dot_product(vec1, vec2):
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z

def collision_check(event, vehicle, collision_detected, subfolder):
    if not collision_detected[0]:
        velocity = vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
        acceleration = vehicle.get_acceleration()
        accel = (acceleration.x**2 + acceleration.y**2 + acceleration.z**2)**0.5
        control = vehicle.get_control()
        steer = control.steer
        output_file = os.path.join(subfolder, 'collision.txt')
        with open(output_file, 'a') as f:
            f.write(f'Speed: {speed}, Acceleration: {accel}, Steer: {steer}')
        collision_detected[0] = True

def lane_detection(event, vehicle, lane_detected, subfolder):
    if not lane_detected[0]:
        velocity = vehicle.get_velocity()
        speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        acceleration = vehicle.get_acceleration()
        accel = (acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2) ** 0.5
        control = vehicle.get_control()
        steer = control.steer
        output_file = os.path.join(subfolder, 'cross_lane.txt')
        with open(output_file, 'a') as f:
            f.write(f'Speed: {speed}, Acceleration: {accel}, Steer: {steer}\n')
            f.write('\nVehicle crossed line\n')
        lane_detected[0] = True

def process_image(image):
    global count
    count += 1
    # image.save_to_disk(os.path.join(subfolder_path1, f'{count}.png'))
    img_pil = None
    if count <= 15:
        image_path = os.path.join(subfolder_attack, f'{count}.png')
        img_pil = Image.open(image_path).convert('RGB')
    else:
        image_path = os.path.join(subfolder_attack, f'{15}.png')
        img_pil = Image.open(image_path).convert('RGB')
    pixels = []
    for y in range(img_pil.height):
        for x in range(img_pil.width):
            pixels.append((img_pil.getpixel((x, y))[0], img_pil.getpixel((x, y))[1], img_pil.getpixel((x, y))[2], 255))
    for i in range(640 * 640):
        image[i] = carla.Color(pixels[i][0], pixels[i][1], pixels[i][2], 255)
    return image

def camera_listen_control(camera_sensor):
    time.sleep(0.5)
    camera_sensor.listen(lambda image: process_image(image))
    time.sleep(0.5)
    camera_sensor.stop()

def speed_image(speed, time, label, ylabel, save_label, subfolder):
    np_speed = np.array(speed)
    np_time = np.array(time)
    filter_indices = (np_time >= 5) & (np_time <= 25)
    filter_time = np_time[filter_indices]
    filter_speed = np_speed[filter_indices]
    npy_path = os.path.join(subfolder, save_label + '.npy')
    txt_path = os.path.join(subfolder, save_label + '.txt')
    png_path = os.path.join(subfolder, save_label + '.png')
    np.save(npy_path, np.column_stack((filter_time, filter_speed)))
    np.savetxt(txt_path, np.column_stack((filter_time, filter_speed)), header=f'Time, {save_label}', comments='', delimiter=', ')
    plt.clf()
    plt.figure()
    plt.plot(filter_time, filter_speed, label=label)
    plt.xlabel('Time (seconds)')
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig(png_path)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town04')
    '''
    # set weather
    weather = world.get_weather()
    weather.cloudiness = 90
    weather.fog_density = 90
    weather.fog_distance = 50
    weather.wind_intensity = 20
    world.set_weather(weather)
    '''
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

    # spectator location
    spectator = world.get_spectator()
    spectator_location = carla.Location(x=330.00, y=41.2642, z=2.00)
    spectator.set_transform(carla.Transform(spectator_location + carla.Location(z=20), carla.Rotation(pitch=-90.00)))

    # camera sensor
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '640')
    camera_bp.set_attribute('fov', '90')

    # spawn vehicles
    vehicles = []
    vehicle1_location = carla.Location(x=330.00, y=42.2642, z=2.00)
    vehicle2_location = carla.Location(x=395.00, y=55.2642, z=0.00)
    vehicle1_rotation = carla.Rotation(pitch=-0.00, yaw=-0.00, roll=-0.00)
    vehicle2_rotation = carla.Rotation(pitch=-0.00, yaw=-75.00, roll=-0.00)
    vehicle1_spawn_point = carla.Transform(location=vehicle1_location, rotation=vehicle1_rotation)
    vehicle2_spawn_point = carla.Transform(location=vehicle2_location, rotation=vehicle2_rotation)
    vehicle2 = world.spawn_actor(car_bp, vehicle2_spawn_point)
    vehicle1 = world.spawn_actor(car_bp, vehicle1_spawn_point)
    vehicles.append(vehicle2)
    vehicles.append(vehicle1)
    camera_point = carla.Location(x=2.5, z=0.7)
    camera_transform = carla.Transform(camera_point)
    camera_sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle1)
    speed_data1 = []
    accel_data1 = []
    speed_data2 = []
    accel_data2 = []
    time_data = []

    # collision listening
    collision_bp = blueprint_library.find('sensor.other.collision')
    collision_sensor1 = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle1)
    collision_detected1 = [False]
    collision_sensor1.listen(lambda event: collision_check(event, vehicle1, collision_detected1, subfolder_path1))
    collision_sensor2 = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle2)
    collision_detected2 = [False]
    collision_sensor2.listen(lambda event: collision_check(event, vehicle1, collision_detected2, subfolder_path2))

    # cross lane listening
    lane_bp = blueprint_library.find('sensor.other.lane_invasion')
    lane_sensor1 = world.spawn_actor(lane_bp, carla.Transform(), attach_to=vehicle1)
    lane_detected1 = [False]
    lane_sensor1.listen(lambda event: lane_detection(event, vehicle1, lane_detected1, subfolder_path1))
    lane_sensor2 = world.spawn_actor(lane_bp, carla.Transform(), attach_to=vehicle2)
    lane_detected2 = [False]
    lane_sensor2.listen(lambda event: lane_detection(event, vehicle2, lane_detected2, subfolder_path2))

    for vehicle in vehicles:
        if vehicle:
            vehicle.set_autopilot(True)
            time.sleep(0.7)
    start_time = time.time()

    try:
        while True:
            spectator.set_transform(carla.Transform(vehicle1.get_transform().location + carla.Location(z=20), carla.Rotation(pitch=-90.00)))
            velocities = []
            accelerations = []
            for vehicle in vehicles:
                accelerations.append(vehicle.get_acceleration())
            for vehicle in vehicles:
                velocities.append(vehicle.get_velocity())
            speeds = []
            accels = []
            for velocity in velocities:
                speeds.append((velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5)
            for acceleration in accelerations:
                accels.append((acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2) ** 0.5)
            speed_data1.append(speeds[0])
            speed_data2.append(speeds[1])
            accel_data1.append(accels[0])
            accel_data2.append(accels[1])
            time_data.append(time.time() - start_time)
            if (time.time() - start_time) > 5.0 and (time.time() - start_time) < 10.0:
                camera_thread = threading.Thread(camera_listen_control(camera_sensor))
                camera_thread.start()
            if (time.time() - start_time) > 25.0:
                break
            world.wait_for_tick()
            time.sleep(0.1)
    finally:
        speed_image(speed_data1, time_data, 'Velocity (m/s)', 'Speed (m/s)', 'Velocity', subfolder_path1)
        speed_image(accel_data1, time_data, 'Acceleration (m2/s)', 'Acc (m2/s)', 'Acceleration', subfolder_path1)
        speed_image(speed_data2, time_data, 'Velocity (m/s)', 'Speed (m/s)', 'Velocity', subfolder_path2)
        speed_image(accel_data2, time_data, 'Acceleration (m2/s)', 'Acc (m2/s)', 'Acceleration', subfolder_path2)
        for vehicle in vehicles:
            vehicle.destroy()
        if camera_sensor.is_listening:
            camera_sensor.destroy()
        if lane_sensor1.is_listening:
            lane_sensor1.destroy()
        if lane_sensor2.is_listening:
            lane_sensor2.destroy()
        if collision_sensor1.is_listening:
            collision_sensor1.destroy()
        if collision_sensor2.is_listening:
            collision_sensor2.destroy()
        print("Destroyed all")

if __name__ == '__main__':
    main()