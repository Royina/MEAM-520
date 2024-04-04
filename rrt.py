
import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK
from lib.IK_position_null import IK

fk = FK()
ik = IK()

class TreeNode:
    def __init__(self, q, parent):
        self.q = q
        self.parent = parent

def distance_cal(A, B):
    _, _, T0e_A = fk.forward(A)
    _, _, T0e_B = fk.forward(B)
    distance = np.linalg.norm(T0e_A[:-1, -1] - T0e_B[:-1, -1])
    return distance

def find_closest_node(tree, q_random):
    distance = [distance_cal(node.q, q_random) for node in tree]
    min_index = np.argmin(distance)
    return min_index, tree[min_index]

def check_self_collision(new, current):
    for i in range(4, 8):
        for j in range(0, 4):
            x, y, z = new[i]
            x_min, y_min, z_min, x_max, y_max, z_max = current[j]
            if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
                return True
    return False

def check_collision(closest, q_rand, map):
    size_inc = 0.09
    segments = 40
    closest_joint_pos, _, _ = fk.forward(closest.q)
    q_rand_joint_pos, _, _ = fk.forward(q_rand)

    boundaries = np.zeros((np.shape(closest_joint_pos)[0], 6))
    boundaries[:, 0:3] = closest_joint_pos - size_inc
    boundaries[:, 3:6] = closest_joint_pos + size_inc

    self_collision = np.array(check_self_collision(q_rand_joint_pos, boundaries))
    if np.any(self_collision):
        return True

    for obstacle in map[0]:
        obstacle[:3] = obstacle[:3] - size_inc
        obstacle[3:] = obstacle[3:] + size_inc
        list =  np.array(detectCollision(closest_joint_pos, q_rand_joint_pos, obstacle))
        if np.any(list):
            return True

        for i in range(segments):
            a = closest.q + i*(q_rand - closest.q)/segments
            b = closest.q + (i+1)*(q_rand - closest.q)/segments
            joint_pos_a = fk.forward(a)
            joint_pos_b = fk.forward(b)
            collide_array = np.array(detectCollision(joint_pos_a,joint_pos_b,obstacle))
            if np.any(collide_array):
                return True

    return False

def rrt(map, start, goal):
    path = []
    lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    if np.any(np.logical_or(start <= lowerLim, start >= upperLim)) or np.any(np.logical_or(goal <= lowerLim, goal >= upperLim)):
        print('Start or Goal configurations are not within the joint limits')
        return np.array(path)

    elif len(map) == 0:
        path.append(start)
        path.append(goal)
        return np.array(path)

    else:
        tree_start = [TreeNode(start, None)]
        tree_goal = [TreeNode(goal, None)]
        max_iterations = 100
        i = 0
        goal_reached = False
        while i <= max_iterations and not goal_reached:
            q_random = np.random.uniform(lowerLim, upperLim)
            start_check = False
            goal_check = False

            st_close_id, st_close_q = find_closest_node(tree_start, q_random)
            goal_reached = False
            if not check_collision(st_close_q, q_random, map):
                q_new = TreeNode(q_random, st_close_id)
                tree_start.append(q_new)
                start_check = True

            goal_close_id, goal_close_q = find_closest_node(tree_goal, q_random)
            if not check_collision(goal_close_q, q_random, map):
                    q_new = TreeNode(q_random, goal_close_id)
                    tree_goal.append(q_new)
                    goal_check = True

            if start_check and goal_check:
                i = i + 1
                goal_reached = True  # Set to True for simplicity; adjust based on your goal condition

        if goal_reached:
            current_node = tree_start[-1]
            path.append(current_node.q)
            while current_node.parent is not None:
                current_node = tree_start[current_node.parent]
                path.append(current_node.q)
            path = path[::-1]

            current_node = tree_goal[-1]
            path.append(current_node.q)

            while current_node.parent is not None:
                current_node = tree_goal[current_node.parent]
                path.append(current_node.q)
    return np.array(path)

if __name__ == '__main__':
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0, -1, 0, -2, 0, 1.57, 0])
    goal = np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(path)
