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


def run_scenario(opt, scenario_params):
    try:
        bg_veh_list = []
        # first define the path of the yaml file and 2lanefreemap file
        scenario_params = add_current_time(scenario_params)
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = \
            sim_api.ScenarioManager(scenario_params,
                                    opt.apply_ml,
                                    opt.version,
                                    town='Town04',
                                    cav_world=cav_world)
        if opt.record:
            scenario_manager.client. \
                start_recorder("freeway_carla.log", True)
        
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])
        

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='freeway_carla',
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
            steer_data = np.append(steer_data, single_cav_list[0].vehicle.get_control().steer)
            brake_data = np.append(brake_data, single_cav_list[0].vehicle.get_control().brake)
            distance_data = np.append(distance_data, get_distance(single_cav_list[0].vehicle.get_transform().location, single_cav_list[1].vehicle.get_transform().location))
            time_data = np.append(time_data, time.time()-start_time)
            np.save(file_path, steer_data)
            np.save(brake_path, brake_data)
            np.save(distance_path, distance_data)
            np.save(time_path, time_data)
            '''
            
    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()
        
        for cav in single_cav_list:
            cav.destroy()
        
        for v in bg_veh_list:
            v.destroy()
