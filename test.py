import carla
import time
import math

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

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    for vehicle in world.get_actors().filter('vehicle.*'):
        vehicle.destroy()
    for walker in world.get_actors().filter('walker.pedestrian*'):
        walker.destroy()
    
    blueprint_library = world.get_blueprint_library()
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
    car_bp = car_blueprints[0]

    vehicle_location = carla.Location(x=-48.82, y=80.26, z=0.00)
    vehicle_rotation = carla.Rotation(pitch=0.00, yaw=90.00, roll=0.00)
    vehicle_spawn_points = carla.Transform(location=vehicle_location, rotation=vehicle_rotation)
    vehicle = world.spawn_actor(car_bp, vehicle_spawn_points)
    vehicle.set_autopilot(False)

    light_location = carla.Location(x=-48.82, y=140.53, z=5.0)
    set_trafficlight(world, light_location)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        vehicle.destroy()

if __name__ == '__main__':
    main()