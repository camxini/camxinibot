from vision import Vision
from math import sqrt, sin, cos, atan2, exp
import numpy as np
import copy
import time
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

def normal_rad(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

class Range:
    def __init__(self, min: float, max: float):
        self.min = min
        self.max = max
    
class Pos:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

class Vel:
    def __init__(self, vx: float, vw: float):
        self.vx = vx
        self.vw = vw

class DWARace:
    def __init__(self, final_x, final_y, 
                 vx_min: float=-2000.0, vx_max: float=2000.0, 
                 dx_min: float=-2500.0, dx_max: float=2500.0, 
                 vw_max: float=3, 
                 dw_max: float=1, 
                 dt=0.2, simt=0.8, res=10):
                #3500, 3000, 10, 10, simt=0.3
        self.obstacles_history = []  # 障碍物历史
        self.obstacles = []  # 障碍物
        self.obstacles_predict = []  # 障碍物预测
        
        # state config
        self.pos = Pos(0.0, 0.0, 0.0)
        self.vel = Vel(0.0, 0.0)
        # range config
        self.vx_range = Range(vx_min, vx_max)
        self.dx_range = Range(dx_min, dx_max)
        self.vw_range = Range(-vw_max, vw_max)
        self.dw_range = Range(-dw_max, dw_max)
        # target config
        self.target = None
        self.final_x = final_x
        self.final_y = final_y
        self.start = None
        self.reach_dist = 800
        self.dangerous_dist = 300
        # simulation config
        self.dt = dt
        self.simt = simt
        self.res = res
        # global config
        self.last_update_time = 0   
        self.cur_update_time = 0
        
    def update_vision(self, vision: Vision):    # for dynamic case
        # update current time
        self.last_update_time = self.cur_update_time
        self.cur_update_time = time.time()

        time_interval = 0.033
        
        # update current position
        self.pos = Pos(vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation)
        # update obstacles
        self.obstacles_history = copy.copy(self.obstacles)
        self.obstacles = []
        self.obstacles_predict = []
        for robot in vision.blue_robot:
            if robot.id != 0:
                self.obstacles.append(Pos(robot.x, robot.y, robot.orientation))
        for robot in vision.yellow_robot:
            if robot.id in [3,4]:
                self.obstacles.append(Pos(robot.x, robot.y, robot.orientation))
        
        if time_interval > 0:
            for obs, obs_histroy in zip(self.obstacles, self.obstacles_history):
                
                vx = (obs.x - obs_histroy.x) / time_interval
                signx = np.sign(vx)
                vy = (obs.y - obs_histroy.y) / time_interval
                signy = np.sign(vy)

                
                for i in range(3):
                    self.obstacles_predict.append(Pos(obs.x, obs.y + (i-1) * 100, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x - 110, obs.y, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x + 110, obs.y, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x - 110, obs.y + signy * 1500 / 3, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x + 110, obs.y + signy * 1500 / 3, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x, obs.y + signy * 2 * 1500 / 3, obs.theta))

        for robot in vision.yellow_robot:
            if robot.id in [6, 9, 12]:
                self.obstacles.append(Pos(robot.x, robot.y, robot.orientation))
        
        if time_interval > 0:
            for obs, obs_histroy in zip(self.obstacles, self.obstacles_history):
                
                vx = (obs.x - obs_histroy.x) / time_interval
                signx = np.sign(vx)
                vy = (obs.y - obs_histroy.y) / time_interval
                signy = np.sign(vy)

                
                for i in range(3):
                    self.obstacles_predict.append(Pos(obs.x, obs.y + (i-1) * 100, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x - 110, obs.y, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x + 110, obs.y, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x - 110, obs.y + signy * 1500 / 3, obs.theta))
                    self.obstacles_predict.append(Pos(obs.x + 110, obs.y + signy * 1500 / 3, obs.theta))

        for robot in vision.yellow_robot:
            if robot.id not in [3,4,6,9,12]:
                self.obstacles.append(Pos(robot.x, robot.y, robot.orientation))

        return self.obstacles_predict
            
    def if_reach_target(self):  
        if self.target.x == self.final_x and self.target.y == self.final_y:
            if sqrt(pow(self.target.x - self.pos.x, 2) + pow(self.target.y - self.pos.y, 2)) < 100:
                return True
        elif sqrt(pow(self.target.x - self.pos.x, 2) + pow(self.target.y - self.pos.y, 2)) < self.reach_dist:
            return True
        else:
            return False
    
    def predict_pos(self, vx_sim, vw_sim):
        pred_pos = copy.copy(self.pos)  # 初始拷贝当前位置
        traj = []
        traj.append(copy.copy(pred_pos))  # 添加初始位置的拷贝
        for i in range(int(self.simt / self.dt)):
            if vw_sim < 0.1:
                # 直线运动模型
                delta_x = vx_sim * self.dt * cos(pred_pos.theta)
                delta_y = vx_sim * self.dt * sin(pred_pos.theta)
                pred_pos.x += delta_x
                pred_pos.y += delta_y
                pred_pos.theta += vw_sim * self.dt
            else:
                # 圆弧运动模型
                delta_theta = vw_sim * self.dt
                pred_pos.x += -vx_sim / vw_sim * (sin(pred_pos.theta) - sin(pred_pos.theta + delta_theta))
                pred_pos.y += vx_sim / vw_sim * (cos(pred_pos.theta) - cos(pred_pos.theta + delta_theta))
                pred_pos.theta += delta_theta
            # 添加当前位置的拷贝到轨迹列表
            traj.append(copy.copy(pred_pos))
        return pred_pos, traj
    
    def navigate(self):
        
        if self.vel.vx < 200:
            temp_vw_max = 4
        else:
            temp_vw_max = self.vw_range.max
        
        vx_min = max(
            self.vx_range.min, 
            self.vel.vx - self.simt * self.dx_range.max,
        )
        vx_max = min(
            self.vx_range.max, 
            self.vel.vx + self.simt * self.dx_range.max,
        )
        vw_min = max(
            # self.vw_range.min, 
            -temp_vw_max,
            self.vel.vw - self.simt * self.dw_range.max,
        )
        vw_max = min(
            # self.vw_range.max,
            temp_vw_max, 
            self.vel.vw + self.simt * self.dw_range.max,
        )
        
        # start simulation
        final_score = 0
        slct_vx = 0
        slct_vw = 0
        best_traj = []
        
        for vx_iter in np.linspace(vx_min, vx_max, self.res):
            for vw_iter in np.linspace(vw_min, vw_max, self.res):
             
                pred_pos,traj = self.predict_pos(vx_iter, vw_iter)

                # heading
                heading_score = abs(np.arctan2(self.target.y - self.pos.y, self.target.x - self.pos.x) - normal_rad(pred_pos.theta)) / (2 * np.pi)
                # print(np.arctan2(self.target.y - self.pos.y, self.target.x - self.pos.x), normal_rad(self.pos.theta))
                
                # target
                target_score =  1 / (1 + sqrt(pow(pred_pos.x - self.target.x, 2) + pow(pred_pos.y - self.target.y, 2))-100)
                
                # dist
                min_dist = 1000000
                
                for obs in (self.obstacles + self.obstacles_predict):
                    for traj_point in traj:
                        dist = sqrt(pow(traj_point.x - obs.x, 2) + pow(traj_point.y - obs.y, 2))
                        if dist < min_dist:
                            min_dist = dist
                    
                if min_dist < self.dangerous_dist:
                    if min_dist > 100:
                        dist_score = 1 - pow(1 - (min_dist - 100) / (self.dangerous_dist - 100), 4) # 1 - (1 - d)^4
                    elif min_dist < 100:
                        dist_score = -np.inf
                    # print('dangerous!')
                else:
                    dist_score = 1.0
                
                # velocity
                if sqrt(pow(pred_pos.x - self.target.x, 2) + pow(pred_pos.y - self.target.y, 2)) < 100:
                    vel_score = 1.0 - vx_iter / self.vx_range.max
                else:
                    vel_score = vx_iter / self.vx_range.max
                vel_score = vx_iter / self.vx_range.max
                
                # sum up
                score = 0 * heading_score + 50 * dist_score + 0 * vel_score + 20 * target_score
                if score > final_score:
                    
                    # print(heading_score)
                    
                    best_pos = pred_pos
                    best_traj = traj
                    final_score = score
                    slct_vw = vw_iter
                    slct_vx = vx_iter

        self.vel = Vel(slct_vx, slct_vw)


        return slct_vx, slct_vw, best_traj, self.obstacles_predict
        # return 0, 0, pred_pos.x, pred_pos.y

    def set_target(self, target_x, target_y):
        self.target = Pos(target_x, target_y, 0.0)