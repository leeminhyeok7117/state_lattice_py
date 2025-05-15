#!/usr/bin/env python3
# -*-coding:utf-8-*-
<<<<<<< HEAD
=======

>>>>>>> 6c227fa (ros2 able version)
import os, sys
import numpy as np
from collections import namedtuple
from scipy.spatial import KDTree

import rclpy
from rclpy.qos import QoSProfile

<<<<<<< HEAD
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64

from global_path import GlobalPath
from cubic_hermit_planner import hermite_with_constraints
=======
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from global_path import GlobalPath
from cubic_hermite_planner import hermite_with_constraints
>>>>>>> 6c227fa (ros2 able version)

TABLE_PATH = os.path.dirname(os.path.abspath(__file__)) + "/lookup_table.csv"
Param = namedtuple('Param',['nxy','nh','d','a_min','a_max','p_min','p_max','ns'])

class StateLatticePlanner:
    def __init__(self, node, gp_name):
        qos_profile = QoSProfile(depth=1)
        self.node = node
<<<<<<< HEAD
        self.candidate_pub = node.create_publisher(PointCloud,'/CDpath', qos_profile)
        self.selected_pub = node.create_publisher(PointCloud,'/SLpath',  qos_profile)
        self.current_speed = node.create_subscription(Float64, '/current_speed', self.current_speed_callback, qos_profile)

=======
        self.candidate_pub = node.create_publisher(PointCloud2,'/CDpath_st', qos_profile)
        self.selected_pub = node.create_publisher(PointCloud2,'/SLpath_st',  qos_profile)
        self.current_speed = node.create_subscription(Float64, '/current_speed', self.current_speed_callback, qos_profile)
        
>>>>>>> 6c227fa (ros2 able version)
        self.glob_path = GlobalPath(gp_name)
        self.lookup_table = self.get_lookup_table(TABLE_PATH)
        self.kdtree = KDTree(self.lookup_table[:, :3])
        
        self.max_steer = np.deg2rad(25.0)
        self.min_steer = -np.deg2rad(25.0)

        self.x_horizon = 0.0
        self.y_width = 0.0
        self.current_speed = 0.0
        
        self.max_cost = 100
        self.obstacle_force = 2.0 #2m
        self.weight_1 = 10
        self.weight_2 = 10
<<<<<<< HEAD
        
        self.visual = True
        self.cd_path = None
        self.sel_path = None
=======

    def visual_selected_path(self, selected_path):
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "glob_path"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        structured_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        points = np.array([(p[0], p[1], 0.0) for p in selected_path], dtype=structured_dtype)

        pc2_msg = point_cloud2.create_cloud(header=header, fields=fields, points=points)
        pc2_msg.is_dense = True 
                
        self.selected_pub.publish(pc2_msg)
    
    def visual_candidate_paths(self, candidate_paths):
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "glob_path"
    
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        structured_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        points = np.array([(p[0], p[1], 0.0) for path in candidate_paths for p in path], dtype=structured_dtype)
                
        pc2_msg = point_cloud2.create_cloud(header=header, fields=fields, points=points)
        pc2_msg.is_dense = True 

        self.candidate_pub.publish(pc2_msg)
>>>>>>> 6c227fa (ros2 able version)
        
    #classification using speed
    def current_speed_callback(self, data):
        v_max = 30.0 #나중에 환산 
        x_low, x_high = 3.0 ,21.0
        y_low, y_high = -10.0, 12.0
        n= 2.0 #억제 강화 or 완화 (1보다 크면 저속억제 작으면 고속억제)
                
        self.current_speed = data.data
        
        #Normalization
        r_n = (np.clip(self.current_speed / v_max, 0.0, 1.0))**n
        
        #limitation by speed x,y
        self.x_horizon = x_low + (x_high - x_low) * r_n
        self.y_width  = y_high - (y_high - y_low) * r_n
    
    def get_lookup_table(self, table_path):
        return np.loadtxt(table_path, delimiter=',', skiprows=1)
    
    def set_parameters(self, situation):
        nxy = 10 #조정해야됨 (가변으로 할꺼면 속도로 가변 해야될듯)
        nh = 3 #고정일듯?
        bound = np.arctan2(self.y_width, self.x_horizon)
        a_min, a_max = -bound, bound
        d = np.hypot(self.x_horizon, self.y_width) if situation == "BIASED" else self.x_horizon
        ns = 5 if situation == "BIASED" else 1
        p_min, p_max = -self.min_steer, self.max_steer
        
        return Param(nxy, nh, d, a_min, a_max, p_min, p_max, ns)
    
    #weight 조정해야됨
    def get_goal_angle_obs(self, obs_xy, max_lat_offset = 3.0, max_angle = np.deg2rad(20.0), weight = 0.8):
        l_car = self.glob_path.q_val_local(0, 0)
        dir_path = -np.sign(l_car)
        ratio_path = min(abs(l_car) / max_lat_offset, 1.0)
        angle_path = dir_path * ratio_path * max_angle

        obs_x, obs_y = obs_xy
        l_obs = self.glob_path.q_val_local(obs_x, obs_y) #나중에 장애물 복수로 받아지게 ㄱ

        dir_obs = -np.sign(l_obs)
        ratio_obs = min(abs(l_obs) / max_lat_offset, 1.0)
        angle_obs = dir_obs * ratio_obs * max_angle

        goal_angle = weight * angle_obs + (1 - weight) * angle_path
        return goal_angle

##########################Sampling (Normal, Biased)##########################
    def sample_states(self, angle_samples, a_min, a_max, d, p_min, p_max, nh):
        angle_samples = np.array(angle_samples)
        a_vals = a_min + (a_max - a_min) * angle_samples  

        xf = d * np.cos(a_vals)
        yf = d * np.sin(a_vals)

        if nh == 1:
            yawf = ((p_max - p_min) / 2) + a_vals
            states = np.stack((xf, yf, yawf), axis=1)
        else:
            j_vals = np.linspace(0, nh - 1, nh)
            yaw_offsets = p_min + (p_max - p_min) * j_vals / (nh - 1)  
            yawf = (yaw_offsets[None, :] + a_vals[:, None])  

            xf = np.tile(xf[:, None], (1, nh))               
            yf = np.tile(yf[:, None], (1, nh))              

            states = np.stack((xf, yf, yawf), axis=2).reshape(-1, 3)

        return states

    def calc_normal_coordinate(self):
        """
        calc normal coordinate parameters
    
        nxy :
            number of position sampling
        nh :
            number of heading sampleing
        d :
            distance of terminal state
        a_min :
            position sampling min angle
        a_max :
            position sampling max angle
        p_min :
            heading sampling min angle
        p_max :
            heading sampling max angle
            
        """
        p = self.set_parameters(situation="NORMAL")
        angle_samples = np.linspace(0.0, 1.0, p.nxy)
        states = self.sample_states(angle_samples, p.a_min, p.a_max, p.d, p.p_min, p.p_max, p.nh)

        return states
    
    def calc_biased_coordinate(self,goal_angle):
        """
        calc biased coordinate parameters

        goal_angle: 
            goal orientation for biased sampling
        ns: 
            number of biased sampling
        nxy: 
            number of position sampling
        nh: 
            number of heading sampleing
        d:  
            distance of terminal state
        a_min: 
            position sampling min angle
        a_max: 
            position sampling max angle
        p_min: 
            heading sampling min angle
        p_max: 
            heading sampling max angle
        
        """
        p = self.set_parameters(situation="BIASED")
        ns = p.ns
        asi = p.a_min + (p.a_max - p.a_min) * np.arange(ns - 1) / (ns - 1)
        cnav = np.pi - np.abs(asi - goal_angle)

        total = cnav.max() * ns - cnav.sum()
        cnav_norm = (cnav.max() - cnav) / total

        csumnav = np.cumsum(cnav_norm)

        nxy = p.nxy
        fractions = np.arange(nxy) / (nxy - 1)              
        idx = np.ceil(fractions * ns).astype(int)         
        idx = np.clip(idx, 0, ns - 2)                  

        di = csumnav[idx]                               

        states = self.sample_states(di, p.a_min, p.a_max, p.d, p.p_min, p.p_max, p.nh)
        return states
############################################################################# 

    #Select candidate points at sampling points (Parameter: path_num)
    def candidate_point_at_sampling(self, states, goal_angle, path_num=9):
        angles = np.arctan2(states[:, 1], states[:, 0])  
    
        diffs = angles - goal_angle
        diffs = (diffs + np.pi) % (2 * np.pi) - np.pi
        idx = np.argsort(np.abs(diffs))[:path_num]
        
        return states[idx]
        
    def generate_candidate_points(self, obs_xy, x, y, yaw, path_num = 9):
        self.glob_path.local_path(x, y, yaw)
        
        curve_flag, goal_angle_curve = self.glob_path.det_sharp_curve()
        obs_flag = True if obs_xy is not None else False
        if obs_flag: ###조건문 수정좀 해야될듯
<<<<<<< HEAD
            goal_angle = self.get_goal_angle_obs(obs_xy, x, y)
=======
            goal_angle = self.get_goal_angle_obs(obs_xy)
>>>>>>> 6c227fa (ros2 able version)
        elif curve_flag:
            goal_angle = goal_angle_curve
        else:
            goal_angle = 0.0
            
        states = self.calc_biased_coordinate(goal_angle) if curve_flag or obs_flag else self.calc_normal_coordinate()
        candidate_points_sampling = self.candidate_point_at_sampling(states, goal_angle, path_num)
        
        candidate_points = []
        _, idxs = self.kdtree.query(candidate_points_sampling, k=1) 
        candidate_points = self.lookup_table[idxs]
            
        return candidate_points
    
    def generate_hermite_spline(self, candidate_points, point_num=5):
        candidate_paths = []                
        for state in candidate_points:
<<<<<<< HEAD
            x_vals, y_vals, yaw_vals = hermite_with_constraints([0,0], [state[0], state[1]], 0, state[2]) #현재 yaw값 적용해야되는지 잘 몰겟(아닐듯?)
=======
            x_vals, y_vals, yaw_vals, _ = hermite_with_constraints([0,0], [state[0], state[1]], 0, state[2]) #현재 yaw값 적용해야되는지 잘 몰겟(아닐듯?)
>>>>>>> 6c227fa (ros2 able version)
            
            idxs = np.round(np.linspace(0, len(x_vals)-1, point_num)).astype(int)
            candidate_path = np.stack([x_vals[idxs],y_vals[idxs],yaw_vals[idxs]], axis=1) 
            candidate_paths.append(candidate_path)

        return candidate_paths      
    
    def cost_function(self, pose, obs_xy):
        cost1, cost2 = 0.0, 0.0
        path_separation = self.glob_path.q_val_local(pose[-1][0], pose[-1][1])
        cost1 = abs(path_separation/2.3) if -0.5 <= path_separation <= 0.5 else abs(path_separation * 100)
        
        if obs_xy is not None:
            obs_xy = np.array(obs_xy)
<<<<<<< HEAD
=======
            if obs_xy.ndim == 1:
                obs_xy = obs_xy.reshape(1, 2)
                
>>>>>>> 6c227fa (ros2 able version)
            points_xy = pose[1:, :2]
            dx = points_xy[:, np.newaxis, 0] - obs_xy[np.newaxis, :, 0]
            dy = points_xy[:, np.newaxis, 1] - obs_xy[np.newaxis, :, 1]
            
            min_dists = np.min(np.hypot(dx, dy), axis=1) #각 점과 모든 장애물 거리 비교해서 가장 작은 값 추출

            for obs_d in min_dists:
                if 0.0 < obs_d < 1.0:
                    cost2 = self.max_cost
                    break
                else:
                    cost2 += (self.obstacle_force - obs_d) / self.obstacle_force if obs_d < self.obstacle_force else 0

        return cost1*self.weight_1 + cost2*self.weight_2
        
<<<<<<< HEAD
    def state_lattice_planner(self, obs_xy, x, y, yaw):
        candidate_points = self.generate_candidate_points(obs_xy, x, y, yaw)
        candidate_paths = np.array(self.generate_hermite_spline(candidate_points))
        selected_path = min(candidate_paths,key=lambda p: self.cost_function(p, obs_xy))   
=======
    def state_lattice_planner(self, x, y, heading, obs_xy=None):
        candidate_points = self.generate_candidate_points(obs_xy, x, y, heading)
        candidate_paths = np.array(self.generate_hermite_spline(candidate_points))
        self.visual_candidate_paths(candidate_paths)
        selected_path = min(candidate_paths,key=lambda p: self.cost_function(p, obs_xy))   
        self.visual_selected_path(selected_path)
>>>>>>> 6c227fa (ros2 able version)
        
        return selected_path
        
        