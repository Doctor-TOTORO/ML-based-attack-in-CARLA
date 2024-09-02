# -*- coding: utf-8 -*-
"""
Scenario testing: stop in front of a trafficlight with passengers crossing with CARLA
"""

import os
import carla
import time
import math
import numpy as np

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import \
    add_current_time
from opencda.core.common.cav_world import CavWorld

file_path = os.path.join(os.getcwd(), 'evaluation_outputs', 'steer_data.npy')
brake_path = os.path.join(os.getcwd(), 'evaluation_outputs', 'brake_data.npy')
time_path = os.path.join(os.getcwd(), 'evaluation_outputs', 'time_data.npy')
distance_path = os.path.join(os.getcwd(), 'evaluation_outputs', 'distance_data.npy')

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
        nearest_traffic_light.set_state(carla.TrafficLightState.Green)
        nearest_traffic_light.set_red_time(0)
        nearest_traffic_light.set_green_time(999999)
        nearest_traffic_light.set_yellow_time(0)
        print("Set successfully")
    else:
        print("No traffic light")

def run_scenario(opt, scenario_params):
    try:
        bg_veh_list = []
        single_cav_list = []

        # first define the path of the yaml file and 2lanefreemap file
        scenario_params = add_current_time(scenario_params)
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("city_carla.log", True)
        
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])
            
        # spawn walkers
        walker = None
        blueprint_library = scenario_manager.world.get_blueprint_library()
        walker_bp = blueprint_library.filter('walker.pedestrian.0001')[0]
        start_location = carla.Location(x=-38.5, y=118.26, z=1.00)
        end_location = carla.Location(x=-56.81, y=118.26, z=1.00)
        direction_vector = carla.Vector3D(x=(end_location.x - start_location.x),
                                          y=(end_location.y - start_location.y),
                                          z=0.00)
        walker_rotation = carla.Rotation(pitch=0.00, yaw=180.00, roll=0.00)
        walker_spawn_point = carla.Transform(location=start_location, rotation=walker_rotation)
        controller = carla.WalkerControl(direction=direction_vector, speed=0.1)
        walker = scenario_manager.world.spawn_actor(walker_bp, walker_spawn_point)
        walker.apply_control(controller)

        light_location = carla.Location(x=-48.82, y=140.53, z=5.0)
        set_trafficlight(scenario_manager.world, light_location)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='city_carla',
                              current_time=scenario_params['current_time'])


        spectator = scenario_manager.world.get_spectator()
        steer_data = np.array([])
        time_data = np.array([])
        brake_data = np.array([])
        distance_data = np.array([])
        start_time = time.time()

        # run steps
        while True:
            scenario_manager.tick()

            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=50),
                    carla.Rotation(
                        pitch=-
                        90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
                '''
                if (single_cav.vehicle.get_transform().location.y > 118.26):
                    steer_data = np.append(steer_data, single_cav.vehicle.get_control().steer)
                    brake_data = np.append(brake_data, single_cav.vehicle.get_control().brake)
                    distance_data = np.append(distance_data, get_distance(single_cav.vehicle.get_transform().location, bg_veh_list[3].get_transform().location))
                    time_data = np.append(time_data, time.time()-start_time)
                    np.save(file_path, steer_data)
                    np.save(brake_path, brake_data)
                    np.save(distance_path, distance_data)
                    np.save(time_path, time_data)
                '''

    finally:

        eval_manager.evaluate()

        scenario_manager.client.stop_recorder()

        if opt.record:
            scenario_manager.close()

        for v in single_cav_list:
            v.destroy()

        for v in bg_veh_list:
            v.destroy()

        if walker is not None:
            walker.destroy()
