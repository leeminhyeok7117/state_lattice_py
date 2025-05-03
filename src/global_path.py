#!/usr/bin/env python
# -*-coding:utf-8-*-

"""
Created on Mon Aug 24 15:12:36 2020

@author: JHP
"""

# 경로 데이터 로드, 좌표 변환, 

import numpy as np
from math import sin, cos, tan, copysign, sqrt, degrees, pi
from scipy.spatial import distance
import os, sys
import time
import bisect
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import cubic_spline_planner # 같은 path_planning 파일에 있어서 가능.
import cartesian_frenet_conversion # 얘도 마찬가지

MAX_SPEED = 3
MIN_SPEED = 1

MAX_CONERING_SPEED = 2
MIN_CONERING_SPEED = 1

# filename은 다른 파일에서 GlobalPath 객체를 생성해서 지정하면 덮어씌워짐.

class GlobalPath:
    def __init__(self, filename='/home/macaron/catkin_ws/src/macaron_5/path/round.npy', x=[], y=[], ds=0.1):
        # /home/macaron/catkin_ws/src/macaron_4/path/8jung_test2.npy
        self.rx, self.ry, self.ryaw, self.rk, self.rdk, self.s = [], [], [], [], [], 0.0
        if len(x) > 0:
            self.rx, self.ry, self.ryaw, self.rk, self.rdk, self.s = cubic_spline_planner.calc_spline_course(x, y, ds)
        else:
            pathArray = np.load(file=filename)
            gx = pathArray[0:pathArray.shape[0] - 1, 0]
            gy = pathArray[0:pathArray.shape[0] - 1, 1]
            self.rx, self.ry, self.ryaw, self.rk, self.rdk, self.s = cubic_spline_planner.calc_spline_course(gx, gy, ds)
        self.vel_param = 0
        self.rvel = self.det_target_speed()    
        self.cur_ref_index = 0
        self.cur_s_ref_index = 0
        self.last_search_time = 0
     
    #######################지역 경로 용도로 사용######################    
    def get_local_path(self, x, y, yaw, lookahead_s: 20.0):
        idx_start, idx_end = self.cur_ref_index-3, self.cur_ref_index+ lookahead_s

        gx = np.array(self.rx[idx_start:idx_end])
        gy = np.array(self.ry[idx_start:idx_end])

        dx = gx - x       
        dy = gy - y
        c = np.cos(yaw)
        s = np.sin(yaw)

        lx =  c*dx + s*dy
        ly = -s*dx + c*dy

        return lx, ly 
    
    def local_path(self, x, y, yaw):
        lx, ly = self.get_local_path(x, y, yaw)
        self.local_x, self.local_y, self.local_yaw,_ ,_ ,_ = cubic_spline_planner.calc_spline_course(lx, ly, ds=0.1)  
    
    def getClosestSIndexCurXY_local(self, x, y, mode=0, mission=None): 
        ref_index = 0
        iteration = len(self.rx)
        ref_index = cartesian_frenet_conversion.getClosestSPoint(self.local_x, self.local_y, x, y, self.cur_ref_index, iteration, mode=mode, mission=mission)
        return ref_index

    def q_val_local(self, x, y, mode=0, base_iter=30, mission=None):
        ref_index = self.getClosestSIndexCurXY_local(x, y, mode=mode, base_iter=base_iter, mission=mission)
        return cartesian_frenet_conversion.calcOffsetPoint(x, y, self.local_x[ref_index], self.local_y[ref_index], self.local_yaw[ref_index]) 
    ################################################################
   
    def getClosestSIndexCurXY(self, x, y, mode=0, base_iter=30, mission=None): 
        ref_index = 0

        if mode == 0:  # 가장 가까운 s 인덱스를 찾을 때 기존 위치 근처에서부터 탐색
            cur_time = time.time()
            time_elapsed = len(self.rx) if cur_time - self.last_search_time > len(
                self.rx) else cur_time - self.last_search_time
            iteration = int(time_elapsed + 1) * base_iter
            self.last_search_time = cur_time
            ref_index = cartesian_frenet_conversion.getClosestSPoint(self.rx, self.ry, x, y, self.cur_ref_index,
                                                                     iteration, mode=mode, mission=mission)
            self.cur_ref_index = ref_index

        elif mode == 1:  # 가장 가까운 s 인덱스를 찾을 때 전체 경로에서 탐색
            iteration = len(self.rx)
            ref_index = cartesian_frenet_conversion.getClosestSPoint(self.rx, self.ry, x, y, self.cur_ref_index,
                                                                     iteration, mode=mode, mission=mission)
        else:
            pass

        return ref_index                                                                         

    def getClosestSIndexCurS(self, s):
        return bisect.bisect(self.s, s) - 1

    # mode 0 -> 찾던 위치 근처에서 찾기, mode 1 처음부터 찾기
    def xy2sl(self, x, y, mode=0, base_iter=30, mission=None):
        ref_index = self.getClosestSIndexCurXY(x, y, mode=mode, base_iter=base_iter, mission=mission)
        self.cur_ref_index = ref_index
        return self.s[ref_index], cartesian_frenet_conversion.calcOffsetPoint(x, y, self.rx[ref_index],
                                                                              self.ry[ref_index], self.ryaw[ref_index]) #l값

    def get_current_reference_point(self):
        return self.rx[self.cur_ref_index], self.ry[self.cur_ref_index], self.ryaw[self.cur_ref_index], self.rk[
            self.cur_ref_index]

    def get_current_reference_yaw(self):
        return self.ryaw[self.cur_s_ref_index]

    def get_current_reference_kappa(self):
        return self.rk[self.cur_s_ref_index]

    def sl2xy(self, s, l):
        ref_index = self.getClosestSIndexCurS(s)
        self.cur_s_ref_index = ref_index
        return cartesian_frenet_conversion.sl2xy(s, l, self.rx[ref_index], self.ry[ref_index], self.ryaw[ref_index])

    def getPathFromTo(self, pos1, pos2):
        index1 = self.getClosestSIndexCurXY(pos1[0], pos1[1], 1)
        index2 = self.getClosestSIndexCurXY(pos2[0], pos2[1], 1)
        print(index1, index2)
        return self.rx[index1:index2], self.ry[index1:index2]
    
    def max_curvature(self, last_yaw, current_yaw):
       max_max = abs(current_yaw - last_yaw)
    
       if max_max >= pi:  # yaw 값이 360도를 넘어갈 때
           max_max = 2 * pi - max_max
       
       return max_max
   
    #############################곡선 판단용 새로만듬 ################################
    def det_sharp_curve(self, threshold_avg_kappa = 0.15, threshold_delta_yaw = np.deg2rad(20), offset = 0): ##threshold, offset 다 parameter
        cur_index = self.cur_ref_index
        target_index = min(cur_index + offset, len(self.ryaw) - 1)
        
        avg_kappa = np.mean(self.rk[cur_index:target_index])
        delta_yaw = self.max_curvature(self.ryaw[target_index], self.ryaw[cur_index])
        
        sharp = True if abs(avg_kappa) > threshold_avg_kappa and delta_yaw > threshold_delta_yaw else False
        
        goal_angle = self.ryaw[target_index] if sharp else 0.0
        return sharp, goal_angle
    #############################################################################
    
    def det_target_speed(self):
        # 곡선도로 타겟 속도 계산
        max_index = len(self.ryaw)
        target_speed = MAX_SPEED
        target_speed_list = []
        for index in range(0, max_index):
            try:
                cur_diff_front = abs(self.max_curvature(self.ryaw[index+50], self.ryaw[index]))
                cur_diff_last = abs(self.max_curvature(self.ryaw[index+100], self.ryaw[index]))
                cur_diff = max(cur_diff_front, cur_diff_last)
            except:
                cur_diff = abs(self.max_curvature(self.ryaw[max_index-1], self.ryaw[index]))
                
            if cur_diff <= 5 * pi / 180:
                target_speed = int((MAX_CONERING_SPEED - MAX_SPEED)/(5 * pi/180) * cur_diff + MAX_SPEED)
            elif cur_diff >= 30 * pi / 180:
                target_speed = MIN_CONERING_SPEED
            else:
                target_speed = int(((MIN_CONERING_SPEED - MAX_CONERING_SPEED)/(25 * pi/180)) * (cur_diff - 5*pi/180) + MAX_CONERING_SPEED)
            
            # 직선도로 속도는 최대속도 (MAX_CONERING_SPEED --> MAX_SPEED)
            if cur_diff <= pi / 180:
                target_speed = MAX_SPEED    

            if target_speed >= MAX_SPEED:
                target_speed = MAX_SPEED
            elif target_speed <= MIN_SPEED:
                target_speed = MIN_SPEED
            
            target_speed_list.append(target_speed)
            index += 1
            
        return target_speed_list


def main():
    pass


if __name__ == '__main__':
    main()