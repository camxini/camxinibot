from vision import Vision
from action import Action
from debug import Debugger
from Astar import Astar
import time
import index

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = Astar(N_SAMPLE = 1000)
    
    cnt = 1
    while True:
        print(f"Task {cnt} started")
        index.pathPlan(vision, action, debugger, planner, -2400, -1500)
        print(f"Task {cnt} finished")
        cnt = cnt + 1
        

        print(f"Task {cnt} started")
        index.pathPlan(vision, action, debugger, planner, 2400, 1500)
        print(f"Task {cnt} finished")
        cnt = cnt + 1
        

        if cnt == 11:
            print(f"Task finished")
            break

        
