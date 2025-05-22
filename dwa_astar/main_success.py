from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
from prm import PRM
from dwa import DWA
from Astar import Astar
import time
import numpy as np

def go_to(vision, action, debugger, planner, goal_x, goal_y):
    # 1. path planning & velocity planning
    
    start_time = time.time()
    
    while True:
        start_x, start_y = vision.my_robot.x, vision.my_robot.y
        
        path_x, path_y, road_map, sample_x, sample_y = planner.plan(vision=vision, 
            start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
        if len(path_x) > 0 and len(path_y) > 0:
            break

    dwa = DWA(goal_x, goal_y, 
              vx_max=1600, vx_min = 0)
    
    for target_x, target_y in zip(reversed(path_x), reversed(path_y)):  # visit target points in correct order   

        dwa.set_target(target_x, target_y)
        
        while True:
            # update version information
            dwa.update_vision(vision)
            
            if dwa.if_reach_target():
                # change target point
                break
            
            vx, vw, best_traj,obs_move= dwa.navigate()
            if target_x == path_x[0] and target_y == path_y[0]:
                vx=0.8*vx
                vw=0.8*vw
            
            '''if target_x == path_x[-6] and target_y == path_y[-6]:
                vx=0.5*vx

            if target_x == path_x[-5] and target_y == path_y[-5]:
                vx=0.4*vx

            if target_x == path_x[-4] and target_y == path_y[-4]:
                vx=0.3*vx

            if target_x == path_x[-3] and target_y == path_y[-3]:
                vx=0.2*vx'''
            
            for i in range(-9, -3):
                if target_x == path_x[i] and target_y == path_y[i]:
                    vx = vx * abs(i) / 10
            
            # 2. send command
            action.sendCommand(vx=vx, vy=0, vw=vw)

            best_traj_x = []
            best_traj_y = []

            for i in range(len(best_traj)):
                best_traj_x.append(best_traj[i].x)
                best_traj_y.append(best_traj[i].y)

            obs_move_x = []
            obs_move_y = []

            for i in range(len(obs_move)):
                obs_move_x.append(obs_move[i].x)
                obs_move_y.append(obs_move[i].y)
            # 3. draw debug msg
            # debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
            package = Debug_Msgs()

            debugger.draw_points(package,best_traj_x,best_traj_y)
            debugger.draw_points(package, path_x, path_y)
            debugger.draw_circle(package, target_x, target_y, radius=100)
            debugger.draw_points(package, obs_move_x, obs_move_y)
            debugger.send(package)

            time.sleep(0.01)
            
        action.sendCommand(vx=0, vy=0, vw=0)
        
    stop_time = time.time()
    
    return stop_time - start_time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = Astar(N_SAMPLE=1000)
    
    cnt = 0
    while True:    
        go_to(vision, action, debugger, planner, -2400, -1500)
        cnt = cnt + 1
        print(f"count: {cnt}\n")
        go_to(vision, action, debugger, planner, 2400, 1500)
        cnt = cnt + 1
        print(f"count: {cnt}\n")

        
