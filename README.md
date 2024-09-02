# ML-based-attack-in-CARLA
The main branch of the repository contains the necessary codes, and the experiments branch is about partial data recorded in this study, which is just for experimental results reference only.

Before testing, please download or copy OpenCDA folder from https://github.com/ucla-mobility/OpenCDA, and YOLOv5 from https://github.com/ultralytics/yolov5.
______________________________________________________________________________________________________________________
## attack
This folder contains the codes of FGSM and SimBA used in this study. The original resources are as follow:

FGSM: https://www.tensorflow.org/tutorials/generative/adversarial_fgsm

SimBA: https://github.com/cg563/simple-blackbox-attack
## configuration
This folder is about both urban scenario settings and motorway scenario settings, also the different weather conditions.

YAML files should be put in /opencda/scenario_testing/config_yaml of OpenCDA folder.
## customize
Codes in this folder are modified from the same-name files from OpenCDA resources, please replace them directly or put them in /opencda/customize of OpenCDA folder.
## scenario
Codes for operating specific scenarios are in this folder, please put them into /opencda/scenario_testing of OpenCDA folder.
## others
setup_opencda.txt is for configuring environment, which provides a step-by-step method that can be referenced.
## experiment
- Download or copy OpenCDA folder and YOLOv5 from github
- Copy scenario testing codes into /OpenCDA/opencda/scenario_testing
- Copy scenario configuration files (.yaml files) into /OpenCDA/opencda/scenario_testing/config_yaml
- Copy attack codes into /OpenCDA or anywhere you can use them directly
- Run the codes as setup_opencda.txt shows
- The default attack is FGSM, it can be changed to SimBA in line 531 and 532 in perception_manager.py
