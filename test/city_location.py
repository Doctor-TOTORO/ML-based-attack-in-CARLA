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

subfolder_name1 = 'city_baseline'
subfolder_name2 = 'city_attack'
subfolder_path1 = os.path.join(os.getcwd(), subfolder_name1, '6')
subfolder_path2 = os.path.join(os.getcwd(), subfolder_name2)
subfolder_attack = os.path.join(os.getcwd(), subfolder_name2, 'simba')
count = 12

def get_distance(location1, location2):
    return math.sqrt((location1.x - location2.x)**2 + (location1.y - location2.y)**2 + (location1.z - location2.z)**2)

def set_trafficlight(world, spectator_location):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    nearest_traffic_light = None
    min_distance = float('inf')
    for traffic_light in traffic_lights:
        distance = get_distance(spectator_location, traffic_light.get_location())
        if distance < min_distance:
            min_distance = distance
            nearest_traffic_light = traffic_light
    if nearest_traffic_light is not None:
        nearest_traffic_light.set_state(carla.TrafficLightState.Red)
        nearest_traffic_light.set_red_time(999999)
        nearest_traffic_light.set_green_time(0)
        nearest_traffic_light.set_yellow_time(0)
        print("Set successfully")
    else:
        print("No traffic light")

def walker_generation(world, walker_bp, walkers, controller, start_location, walker_rotation):
    walker_spawn_point = carla.Transform(location=start_location, rotation=walker_rotation)
    walker = world.spawn_actor(walker_bp, walker_spawn_point)
    walker.apply_control(controller)
    walkers.append(walker)
    if len(walkers) > 4:
        for i in range(len(walkers) - 4):
            walkers[i].destroy()
            walkers.remove(walkers[i])
    world.tick()
    time.sleep(1.0)

def collision_check(event, vehicle, collision_detected):
    if not collision_detected[0]:
        velocity = vehicle.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
        acceleration = vehicle.get_acceleration()
        accel = (acceleration.x**2 + acceleration.y**2 + acceleration.z**2)**0.5
        control = vehicle.get_control()
        steer = control.steer
        output_file = os.path.join(subfolder_path2, 'collision.txt')
        with open(output_file, 'a') as f:
            f.write(f'Speed: {speed}, Acceleration: {accel}, Steer: {steer}')
        collision_detected[0] = True

def lane_detection(event, vehicle, lane_detected):
    if not lane_detected[0]:
        velocity = vehicle.get_velocity()
        speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        acceleration = vehicle.get_acceleration()
        accel = (acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2) ** 0.5
        control = vehicle.get_control()
        steer = control.steer
        output_file = os.path.join(subfolder_path2, 'cross_lane.txt')
        with open(output_file, 'a') as f:
            f.write(f'Speed: {speed}, Acceleration: {accel}, Steer: {steer}')
            f.write('\nVehicle crossed line.')
        lane_detected[0] = True

def process_image(image):
    global count
    count += 1
    image.save_to_disk(os.path.join(subfolder_path2, f'{count}.png'))
    '''
    img_pil = None
    if count <= 10:
        image_path = os.path.join(subfolder_attack, f'{count}.png')
        img_pil = Image.open(image_path).convert('RGB')
    else:
        image_path = os.path.join(subfolder_attack, f'{10}.png')
        img_pil = Image.open(image_path).convert('RGB')
    pixels = []
    for y in range(img_pil.height):
        for x in range(img_pil.width):
            pixels.append((img_pil.getpixel((x, y))[0], img_pil.getpixel((x, y))[1], img_pil.getpixel((x, y))[2], 255))
    for i in range(640 * 640):
        image[i] = carla.Color(pixels[i][0], pixels[i][1], pixels[i][2], 255)
    return image
    '''
def camera_listen_control(camera_sensor):
    time.sleep(0.5)
    camera_sensor.listen(lambda image: process_image(image))
    time.sleep(0.5)
    camera_sensor.stop()

def speed_image(speed, time, label, ylabel, save_label, subfolder):
    np_speed = np.array(speed)
    np_time = np.array(time)
    filter_indices = (np_time >= 0) & (np_time <= 15)
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
    plt.xticks(np.arange(0, 16, 3))
    plt.legend()
    plt.grid(True)
    plt.savefig(png_path)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # set weather
    weather = world.get_weather()
    weather.cloudiness = 90
    weather.fog_density = 90
    weather.fog_distance = 50
    weather.wind_intensity = 20
    world.set_weather(weather)

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
    light_location = carla.Location(x=-48.87, y=140.53, z=0.00)
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(spectator_location + carla.Location(z=20), carla.Rotation(pitch=-90)))
    set_trafficlight(world, light_location)

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
    start_time = time.time()
    max_time = time.time()
    speed_data = []
    accel_data = []
    time_data = []
    max_speed = 0.0
    period_speed = 0.0
    vehicle_move = True

    # spawn walkers
    start_location = carla.Location(x=-35.87, y=118.53, z=1.00)
    end_location = carla.Location(x=-56.87, y=118.53, z=1.00)
    direction_vector = carla.Vector3D(x=(end_location.x-start_location.x),
                                        y=(end_location.y-start_location.y),
                                        z=0.00)
    walker_rotation = carla.Rotation(pitch=0.00, yaw=180.00, roll=0.00)
    walker_bp = blueprint_library.filter('walker.pedestrian.0001')[0]
    controller = carla.WalkerControl(direction=direction_vector, speed=0.3)
    walkers = []

    # collision listening
    collision_bp = blueprint_library.find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    collision_detected = [False]
    collision_sensor.listen(lambda event: collision_check(event, vehicle, collision_detected))

    # cross lane listening
    lane_bp = blueprint_library.find('sensor.other.lane_invasion')
    lane_sensor = world.spawn_actor(lane_bp, carla.Transform(), attach_to=vehicle)
    lane_detected = [False]
    lane_sensor.listen(lambda event: lane_detection(event, vehicle, lane_detected))

    try:
        while True:
            current_time = time.time() - start_time
            walker_thread = threading.Thread(walker_generation(world, walker_bp, walkers, controller, start_location, walker_rotation))
            walker_thread.start()
            velocity = vehicle.get_velocity()
            speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            acceleration = vehicle.get_acceleration()
            accel = (acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2) ** 0.5
            speed_data.append(speed)
            accel_data.append(accel)
            time_data.append(current_time)
            if speed > 0.1 and vehicle_move:
                if speed > max_speed:
                    max_speed = speed
                    max_time = current_time
                if period_speed > speed:
                    camera_thread = threading.Thread(target=camera_listen_control, args=(camera_sensor, ))
                    camera_thread.start()
                period_speed = speed
            if speed < 0.1 and vehicle_move:
                end_time = current_time
                output_file = os.path.join(subfolder_path2, 'vehicle_stop.txt')
                with open(output_file, 'a') as f:
                    f.write(f'Time period: {current_time}')
                    f.write(f'\nSpeed slowing: {end_time - max_time}')
                    f.write(f'\nSteering: {vehicle.get_control().steer}')
                    f.write(f'\nLocation: {vehicle.get_location()}')
                print(f'Time period: {current_time}')
                print(f'Speed slowing: {end_time - max_time}')
                print(f'Steering: {vehicle.get_control().steer}')
                print(f'Location: {vehicle.get_location()}')
                vehicle_move = False
            if collision_detected[0] and collision_sensor.is_listening:
                collision_sensor.stop()
                collision_detected[0] = False
            if current_time > 20.0:
                break
            time.sleep(0.01)
    finally:
        output_file = os.path.join(subfolder_path2, 'vehicle_stop.txt')
        with open(output_file, 'a') as f:
            f.write(f'\nMax speed: {max_speed}')
        print(f'Max speed: {max_speed}')
        speed_image(speed_data, time_data, 'Velocity (m/s)', 'Speed (m/s)', 'Velocity', subfolder_path2)
        speed_image(accel_data, time_data, 'Acceleration (m2/s)', 'Acc (m2/s)', 'Acceleration', subfolder_path2)
        if collision_sensor.is_listening:
            collision_sensor.destroy()
        if camera_sensor.is_listening:
            camera_sensor.destroy()
        vehicle.destroy()
        for walker in walkers:
            walker.destroy()

if __name__ == '__main__':
    main()