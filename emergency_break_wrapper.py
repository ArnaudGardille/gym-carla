
from random import random
import gym
import gym_carla
import carla
import random
class EmergencyBreak(gym.Wrapper):

    def __init__(self, env) -> None:
        super().__init__(env)

        self.hero_spawning_point = None
        self.obstacle_spawning_point = None

    def reset(self):
        # Is there no obstacle in the simulation?
        self.no_obstacle = True

        return super().reset()
        


    def set_hero_spawning_point(self, spawning_point_id): 
        self.hero_spawning_point = list(self.world.get_map().get_spawn_points())[spawning_point_id]

    def set_obsacle_spawning_point(self, spawning_point_id): 
        self.obstacle_spawning_point = list(self.world.get_map().get_spawn_points())[spawning_point_id]

    def set_obstacle_blueprint(self, blueprint):
        blueprint_library = self.world.get_blueprint_library()
        self.obstacle_blueprint = random.choice(blueprint_library.filter(blueprint))

    def add_obstacle(self, d=20):
        #print('adding obstacle')
        if self.obstacle_blueprint is None:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        else:
            vehicle_bp = self.obstacle_blueprint
        #location = self.ego.get_location()
        #location.x += 10.0
        #location.z += 0.2
        #relative_transform = carla.Transform(location, carla.Rotation(yaw=180))
        #relative_transform = carla.Transform(carla.Location(x=5, z=1))
        #obstacle = self.world.try_spawn_actor(vehicle_bp, relative_transform, attach_to=self.ego)
        
        obstacle_on_planned_route = True

        if obstacle_on_planned_route:
            vehicle_location = self.ego.get_transform().location
            vehicle_waypoint = self.world.get_map().get_waypoint(vehicle_location)
            possible_waypoints = vehicle_waypoint.next_until_lane_end(1)
            if len(possible_waypoints) < d:
                return True
            new_vehicle_waypoint = possible_waypoints[:d][-1]
            #print('new_vehicle_waypoint: ', new_vehicle_waypoint)

            new_vehicle_location = new_vehicle_waypoint.transform.location + carla.Location(0, 0 , 2)
            new_vehicle_rotation = new_vehicle_waypoint.transform.rotation
        else:
            vehicle_location = self.ego.get_transform().location
            vehicle_waypoint = self.world.get_map().get_waypoint(vehicle_location)

            new_vehicle_waypoint = vehicle_waypoint.next(d)[0]

            new_vehicle_location = new_vehicle_waypoint.transform.location + carla.Location(0, 0 , 2)
            new_vehicle_rotation = new_vehicle_waypoint.transform.rotation

        obstacle = self.world.try_spawn_actor(vehicle_bp, carla.Transform(new_vehicle_location, new_vehicle_rotation))
        self.obstacle = obstacle
        self.no_obstacle = obstacle == None

        if self.no_obstacle:
            print("Spawning failed")
        else:
            obstacle.apply_control(carla.VehicleControl(hand_brake = True))

        return self.no_obstacle

    def step(self, action):
        if self.no_obstacle and random.random()<self.proba_spawn_obstacle:
            self.no_obstacle = self.add_obstacle(self.distance_spawning)
            """if random.randint(0, max(1000-self.steps_after_reset, 100))==0 :

                (lateral_distance, heading_error, speed, front_vehicle, hazard_distance) =  self.get_state()
                self.no_obstacle = self.add_obstacle(4*speed + 10)"""
        else:
            self.steps_after_spawn += 1

        return super().step(action)

    def compute_distance_obstacle(self):
        ego_loc = self.ego.get_transform().location
        obstacle_loc = self.obstacle.get_transform().location
        return ego_loc.distance(obstacle_loc)


  