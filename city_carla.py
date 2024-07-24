# -*- coding: utf-8 -*-
"""
Scenario testing: stop in front of a trafficlight with passengers crossing with CARLA
"""

import os
import carla

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
        
        # spawn walkers
        

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        spectator = scenario_manager.world.get_spectator()

        # run steps
        while True:
            scenario_manager.tick()
            transform = bg_veh_list[0].get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))


    finally:

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in bg_veh_list:
            v.destroy()