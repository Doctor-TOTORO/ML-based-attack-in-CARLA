import carla

def get_vehicle_location(vehicle):
    transform = vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation
    print(f"Vehicle location: {location.x:.2f}, {location.y:.2f}, {location.z:.2f}")
    print(f"Vehicle rotation: Pitch={rotation.pitch:.2f}, Yaw={rotation.yaw:.2f}, Roll={rotation.roll:.2f}")
    return location, rotation

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicle = world.get_actors().filter('vehicle.*')[0]
    print(vehicle.get_transform().location, vehicle.get_transform().rotation)

if __name__ == "__main__":
    main()