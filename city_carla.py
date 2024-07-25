# -*- coding: utf-8 -*-
"""
Scenario testing: stop in front of a trafficlight with passengers crossing with CARLA
"""

import os
import carla
import time

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import \
    add_current_time
from opencda.core.common.cav_world import CavWorld


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
        '''
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])
        '''
        # spawn walkers
        walker = None
        blueprint_library = scenario_manager.world.get_blueprint_library()
        walker_bp = blueprint_library.filter('walker.pedestrian.0001')[0]
        start_location = carla.Location(x=-36.81, y=118.26, z=1.00)
        end_location = carla.Location(x=-56.81, y=118.26, z=1.00)
        direction_vector = carla.Vector3D(x=(end_location.x - start_location.x),
                                          y=(end_location.y - start_location.y),
                                          z=0.00)
        walker_rotation = carla.Rotation(pitch=0.00, yaw=180.00, roll=0.00)
        walker_spawn_point = carla.Transform(location=start_location, rotation=walker_rotation)
        walker = scenario_manager.world.spawn_actor(walker_bp, walker_spawn_point)
        controller = carla.WalkerControl(direction=direction_vector, speed=0.1)
        walker.apply_control(controller)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_carla',
                              current_time=scenario_params['current_time'])


        spectator = scenario_manager.world.get_spectator()

        # run steps
        while True:
            scenario_manager.tick()

            transform = bg_veh_list[0].get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=20),
                    carla.Rotation(
                        pitch=-
                        90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)




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