#!/usr/bin/env python

# Copyright (c) 2019: Jianyu Chen (jianyuchen@berkeley.edu).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import gym
import gym_carla
import carla
from tqdm import trange
#from gym_carla.envs.misc import *
from gym import spaces
import numpy as np
import math
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

import time

def compute_magnitude_angle(waypoint, current_location, orientation):
  """
  Compute relative angle and distance between a target_location and a current_location

  :param target_location: location of the target object
  :param current_location: location of the reference object
  :param orientation: orientation of the reference object
  :return: a tuple composed by the distance to the object and the angle between both objects
  """
  target_vector = np.array([waypoint[0] - current_location.x, waypoint[1] - current_location.y])
  norm_target = np.linalg.norm(target_vector)

  forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
  norm_forward = np.linalg.norm(forward_vector)

  d_angle = math.acos(np.dot(forward_vector, target_vector) / (norm_target * norm_forward))
  #d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / (norm_target * norm_forward)))
  #print(d_angle)
  if d_angle > np.pi / 2.0:
    d_angle = d_angle - np.pi
  
  #print(np.dot(forward_vector, target_vector))
  return (norm_target, forward_vector, d_angle)


def main():
  # parameters for the gym_carla environment
  params = {
    'number_of_vehicles': 10,
    'number_of_walkers': 100,
    'display_size': 256,  # screen size of bird-eye render
    'max_past_step': 1,  # the number of past steps to draw
    'dt': 0.1,  # time interval between two frames
    'discrete': False,  # whether to use discrete control space
    'discrete_acc': [-3.0, 0.0, 3.0],  # discrete value of accelerations
    'discrete_steer': [-0.2, 0.0, 0.2],  # discrete value of steering angles
    'continuous_accel_range': [-3.0, 3.0],  # continuous acceleration range
    'continuous_steer_range': [-0.3, 0.3],  # continuous steering angle range
    'ego_vehicle_filter': 'vehicle.lincoln*',  # filter for defining ego vehicle
    'port': 2000,  # connection port
    'town': 'Town03',  # which town to simulate
    'task_mode': 'random',  # mode of the task, [random, roundabout (only for Town03)]
    'max_time_episode': 1000,  # maximum timesteps per episode
    'max_waypt': 12,  # maximum number of waypoints
    'obs_range': 32,  # observation range (meter)
    'lidar_bin': 0.125,  # bin size of lidar sensor (meter)
    'd_behind': 12,  # distance behind the ego vehicle (meter)
    'out_lane_thres': 2.0,  # threshold for out of lane
    'desired_speed': 8,  # desired speed (m/s)
    'max_ego_spawn_times': 200,  # maximum times to spawn ego vehicle
    'display_route': True,  # whether to render the desired route
    'pixor_size': 64,  # size of the pixor labels
    'pixor': False,  # whether to output PIXOR observation
    'auto_steer':True
  }



  class AngleObjective(gym.ObservationWrapper):
    def __init__(self, env):
      super().__init__(env)

    def observation(self, observation):
      ego_transform = self.env.ego.get_transform()
      current_waypoint = self.env.waypoints[5]

      norm_target, forward_vector, d_angle = compute_magnitude_angle(current_waypoint, ego_transform.location, ego_transform.rotation.yaw)
      print(norm_target, forward_vector, d_angle)
      return super().observation(observation)
    
  class AutoSteer(gym.ActionWrapper):
    def __init__(self, env):
      super().__init__(env)
      if self.discrete:
        self.action_space = spaces.Discrete(self.n_acc) # self.n_steer
      else:
        self.action_space = spaces.Box(np.array(params['continuous_accel_range'][0]), np.array(params['continuous_accel_range'][1]), dtype=np.float32)

      self.expert = None
      self.previous_waypoint = None
    
    def action(self, act):
      if self.expert is None:
        self.expert = BasicAgent(self.env.ego, 3.6)


      current_waypoint = self.env.waypoints[4]

      if self.previous_waypoint is None:
        self.previous_waypoint = current_waypoint
      elif self.previous_waypoint != current_waypoint:
        self.expert.set_destination(current_waypoint)
        
      control = self.expert.run_step()

      ang_v = self.ego.get_angular_velocity()
      steer = - control.steer 

      if self.discrete:
          return act
      else:
          return [act, steer]


    def reset(self):
      self.expert = None
      self.previous_waypoint = None
      return self.env.reset()
      
  # Set gym-carla environment
  env = gym.make('carla-v0', params=params)
  env = AutoSteer(env)
  obs = env.reset()
  #env.ego.set_autopilot(True)

  pbar = trange(1000)
  for i in pbar:
    action = 1.5 #[2.0, 0.0]
    obs,r,done,info = env.step(action)
    #pbar.set_description('Reward: ' + str(r))
    #print(r)
    if done:
      obs = env.reset()


if __name__ == '__main__':
  main()