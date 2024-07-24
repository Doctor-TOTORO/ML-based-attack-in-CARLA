import carla
import yaml
from omegaconf import OmegaConf
import sys
import os
dirname = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append('D:\\download\\OpenCDA')
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.version import __version__

default_yaml = os.path.join(dirname, 'OpenCDA', 'opencda/scenario_testing/config_yaml/default.yaml')
default_dict = OmegaConf.load(default_yaml)
cav_world = CavWorld(opt.apply_ml)
scenario_manager = sim_api.ScenarioManager(default_dict,
                                           opt.apply_ml,
                                           town='Town06',
                                           cav_world=cav_world)
