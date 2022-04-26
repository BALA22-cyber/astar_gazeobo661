from traceback import print_tb
import numpy as np
import cv2 as cv
import math
from math import dist
import matplotlib.pyplot as plt

cspace = np.zeros((1000, 1000, 36), np.uint8)
map = np.ones((1000, 1000, 3), np.uint8)*255

def dist(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def obstacle(pt, cl = 20):

    x, y, th = pt
    if np.sqrt((x - 200)**2 + (y - 200)**2) <= 100 + cl:
        return True
    
    if np.sqrt((x - 200)**2 + (y - 800)**2) <= 100 + cl:
        return True
    
    if 25 - cl <= x <= 175 + cl:
        if 425 -cl <= y <= 575 + cl:
            return True
    
    if 375 - cl <= x <= 625 + cl:
        if 425 -cl <= y <= 575 + cl:
            return True
    
    if x <= 0 or x >= 1000: return True
    if y <= 0 or y >= 1000: return True

    return False

# Creating nodes and storing in a dictionary
def get_node( pos, parent, action, cost):
    Node  = {'pos': pos,
             'parent': parent, 
             'action': action,
             'cost': cost}
    return Node


def check_goal(child_pos, goal_pos):

    dst = dist(child_pos[:2], goal_pos)
    if dst < 60: 
        return  True
    else: 
        return False


x_s, y_s, theta_start = 100, 100, 0

x_g, y_g = 900, 900
start_pos = (x_s, y_s, theta_start)
goal_pos = (x_g, y_g)
rpm1, rpm2 = 50, 100

all_actions = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]


def Action(Xi,Yi,Thetai,UL,UR, vis = False):
    t = 0
    r = 0.038*100
    L = 0.354*100
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = np.pi*Thetai/180
    xs, ys = [], []

    UL, UR = UL*0.10472, UR*0.10472


    v = 0.5*r * (UL + UR) * dt
    w = ((r / L) * (UR - UL) * dt)
    D = 0
    for _ in range(10):
        t = t + dt
        Xn += v * np.cos(Thetan)
        Yn += v * np.sin(Thetan)
        Thetan += w
        D = D + math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        xs.append(Xn)
        ys.append(Yn)

    Xn, Yn = int(Xn), int(Yn)
    Thetan = int(((Thetan*180/np.pi)%360)/10)*10

    return (Xn, Yn, Thetan), D, (xs, ys)
    

def close(pos):
    x, y, th = pos
    cspace[x, y, int(th/10)] = 1

def is_open(pos):
    x, y, th = pos
    return obstacle(pos) is False and cspace[x, y, int(th/10)] == 0 


c2g = lambda start_pos: 1.5*dist(start_pos[:2], goal_pos[:2])

##A Star Algorithm
def A_star(start_pos, goal_pos):
    start_node = get_node(start_pos, None, None, 0)
    open_dict = {start_pos: c2g(start_pos)}
    explored = {start_pos: start_node}

    while len(open_dict):
        curr_pos = min(open_dict, key = open_dict.get)
        curr_node = explored[curr_pos]
        open_dict.pop(curr_pos)
        close(curr_pos)

        if check_goal(curr_pos, goal_pos): 
            print("solution found")
            return backtrack(curr_node), explored

        # gives the minimum position of nodes
        for action in all_actions:
            rpm_l, rpm_r = action
            pos, D, _ = Action(*curr_pos, rpm_l, rpm_r)
            # print(pos)
            cost = curr_node['cost'] + D
            if is_open(pos):
                if pos not in explored:
                    child_node = get_node(pos, curr_node, action, cost)
                    explored[pos] = child_node
                    open_dict[pos] = cost + c2g(pos)
                else:
                    child_node = explored[pos]
                    if cost < child_node['cost']:
                        child_node['cost'] = cost
                        child_node['parent'] = curr_node
                        child_node['action'] = action
                        open_dict[pos] = cost + c2g(pos)

    print("###################################")
    print(explored)
    return None, explored

#######BackTracking
def backtrack(node):
    print("Tracking Back")
    path = []
    while node['parent'] is not None:
        path.append(node)
        node = node['parent']
    path.reverse()
    return path

####A = Backtracking

#visulizing the path explored nodes
def visualize(path, explored, name = 'result'):
        map = np.ones((1000, 1000, 3), np.uint8)*255
        cv.circle(map, (200, 200), 100, (0, 0, 0), thickness = -1)
        cv.circle(map, (200, 800), 100, (0, 0, 0), thickness = -1)

        cv.rectangle(map, (25, 425), (175, 575), (0, 0, 0), thickness= -1)
        cv.rectangle(map, (375, 425), (625, 575), (0, 0, 0), thickness= -1)
        cv.rectangle(map, (725, 800), (875, 600), (0, 0, 0), thickness= -1)

        h, w, _ = map.shape
        img = map
        # open video writerd
        out = cv.VideoWriter(f'{name}.mp4', cv.VideoWriter_fourcc(*'mp4v'), 30.0, (w, h))
        # visualize exploration
        k = 0
        for pos, node in explored.items():
            parent = node['parent']
            if parent is None: continue
            pos, D, (xs, ys) = Action(*parent['pos'], *node['action'])
            cv.polylines(img, [np.int32(np.c_[xs, w - np.array(ys) - 1])], False, [0, 80 ,0], 1)
            if k%20 == 0:
                out.write(img)
                cv.imshow('Exploration', img)
                cv.waitKey(1)
            k += 1
            
        # visualise path
        if path is not None:
            for node in path:
                parent = node['parent']
                if parent is None: parent = path[0]
                pos, D, (xs, ys) = Action(*parent['pos'], *node['action'])
                cv.polylines(img, [np.int32(np.c_[xs, w - np.array(ys) - 1])], False, [0, 0, 255], 2)
            for i in range(100): out.write(img)
        out.release()
        #show final image and save video
        cv.imshow('Exploration', img)
        cv.imwrite('final.jpg', img)
        print('\npress any key to exit')
        cv.waitKey(0)
        print(f'Video saved as {name}.mp4')
            






if __name__ == "__main__":
   


    # give the input points over here
    x_s = int(input('Give x-co-ordinate of the start position: '))
    y_s = int(input('Give y-co-ordinate of the start position: '))
    theta_start = int(input('Give initial orientation: '))
    while (theta_start/30 != 0) and theta_start not in range (-60, 61):
        print('Invalid entry')
        theta_start = int(input('Give initial orientation: '))

    x_g = int(input('Give x-co-ordinate of the goal position: '))
    y_g = int(input('Give y-co-ordinate of the goal position: '))
    rpm1 = int(input('Give RPM 1: '))
    rpm2 = int(input('Give RPM 2: '))

    start_pos = (x_s, y_s, theta_start)
    goal_pos = (x_g, y_g)




    all_actions = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

    A, explore = A_star(start_pos, goal_pos)
    visualize(A, explore)



