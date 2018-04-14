# Created on Sun Mar 11 10:26:54 2018
# Author: Chaitanya Pb

#%% Package imports

import random
import numpy as np

#%% Helper classes and functions

class Node:
    def __init__(self, state):
        self.state = state
        self.parent = None
        self.cost = 0

def distance(s1, s2, metric='euclidean'):
    if len(s1) != len(s2):
        raise ValueError("Input states are not valid!")
    else:
        if metric == 'euclidean':
            acc = 0
            for i in range(len(s1)):
                acc += pow(s1[i] - s2[i], 2)
            return np.sqrt(acc)
        elif metric == 'manhattan':
            acc = 0
            for i in range(len(s1)):
                acc += abs(s1[i] - s2[i])
            return acc
        else:
            raise ValueError("Unsupported distance metric!")

def randomSampler(a, b):
    return random.uniform(a, b)

#%% Motion Planner base class

class MotionPlanner:
    def __init__(self, env):
        self.env = None
        self.start = None
        self.goal = None
        
        if self.is_env_valid(env):        
            self.env = env
        else:
            raise ValueError("Env is invalid!")
    
    def is_env_valid(self, env):
        if len(env.state_limits) != 2:
            return False
        else:
            for lim in env.state_limits:
                if len(lim) != env.state_dim:
                    return False
            return True
    
    def set_start(self, start):
        if len(start) == self.env.state_dim:        
            self.start = start
        else:
            raise ValueError("Invalid Start!")
    
    def set_goal(self, goal):
        if len(goal) == self.env.state_dim:        
            self.goal = goal
        else:
            raise ValueError("Invalid Goal!")
    
    def plan(self, start, goal):
        self.set_start(start)
        self.set_goal(goal)

#%% Rapidly-exploring Random Tree (RRT) Algorithm

class RRT(MotionPlanner):
    def __init__(self, env, step_size, growth='connect'):
        MotionPlanner.__init__(self, env)
        self.step_size = step_size
        self.growth = growth
        self.tree = []

    def start_tree(self, start, goal):
        MotionPlanner.plan(self, start, goal)        
        self.tree = [Node(self.start)]
        if self.env.in_collision(start):
            raise ValueError("Start is in collision!")
    
    def clear_tree(self):
        self.tree = []
    
    def extend_tree(self, des_state=None, stopAtGoal=False):
        old_len = len(self.tree)
        reachedGoal = False
        
        if des_state == None:
            des_state = self.random_free_state()
        
        near_node = self.find_nearest_neighbor(des_state)
        success, next_state, cost = self.env.steer(des_state, near_node.state, self.step_size)
        if success:
            next_node = Node(next_state)
            next_node.parent = near_node
            next_node.cost = near_node.cost + cost
            self.tree.append(next_node)
            reachedRand = self.env.distance(des_state, next_state) < self.step_size
            reachedGoal = self.env.distance(self.goal, next_state) < self.step_size
        
        if self.growth == 'connect':
            while success and not reachedRand and not (stopAtGoal and reachedGoal):
                near_node = next_node
                success, next_state, cost = self.env.steer(des_state, near_node.state, self.step_size)
                if success:
                    next_node = Node(next_state)
                    next_node.parent = near_node
                    next_node.cost = near_node.cost + cost
                    self.tree.append(next_node)
                    reachedRand = self.env.distance(des_state, next_state) < self.step_size
                    reachedGoal = self.env.distance(self.goal, next_state) < self.step_size
        elif self.growth == 'extend':
            pass
        else:
            pass
        
        return (stopAtGoal and reachedGoal), self.tree[old_len:len(self.tree)]

    def grow_tree(self, start, mode='samples', goal=None, samples=0):
        self.start_tree(start)
        new_nodes = [start]
        goalFound = False
        
        if mode == 'samples':
            for i in range(samples):
                self.extend_tree()
        elif mode == 'goal' and goal != None:
            while not goalFound:
                goalFound, new_nodes = self.extend_tree(stopAtGoal=True)
        else:
            raise ValueError("Invalid mode!")
    
    def search_for_goal(self, goal, nodes_list):
        for node in nodes_list:
            if self.env.distance(node.state, goal) < self.step_size:
                return True, node
        return False, None
    
    def find_path(self, goal):
        MotionPlanner.set_goal(self, goal)
        
        if len(self.tree) == 0:
            raise ValueError("Tree is empty!")
        
        best_node = self.find_nearest_neighbor(self.goal)
        found_path = self.path_to_root(best_node)
        path_cost = best_node.cost
        
        return found_path, path_cost
    
    def plan(self, start, goal):
        MotionPlanner.plan(self, start, goal)
        self.grow_tree(start)
        path, cost = self.find_path(goal)
        return path, cost

    def random_free_state(self):
        while True:
            rand_state = []
            lim = self.env.state_limits
            for i in range(self.env.state_dim):
                rand_state += [randomSampler(lim[0][i], lim[1][i])]
            rand_state = tuple(rand_state)
            if not self.env.in_collision(rand_state):
                break
        return rand_state
    
    def find_nearest_neighbor(self, state):
        min_dist = float('inf')
        min_node = None
        
        for node in self.tree:
            node_dist = distance(state, node.state)
            if node_dist < min_dist:
                min_dist = node_dist
                min_node = node
        
        return min_node
    
    def next_state_action(self, des_state, node):
        min_dist = distance(node.state, des_state)
        sel_action = None
        next_state = None
        
        for action in self.env.action_list:
            new_state = self.env.next_state(node.state, action)
            if self.env.in_collision(new_state):
                continue
            new_dist = distance(new_state, des_state)
            if new_dist < min_dist:
                min_dist = new_dist
                sel_action = action
                next_state = new_state
        
        return next_state, sel_action
    
    def path_to_root(self, node):
        path_to_root = []
        curr_node = node
        
        while curr_node != None:
            path_to_root += [curr_node]
            curr_node = curr_node.parent
        
        return path_to_root

#%% Bi-directional RRT Algorithm

class BiRRT(RRT, MotionPlanner):
    def __init__(self, env, step_size, growth='ce'):
        MotionPlanner.__init__(self, env)
        self.step_size = step_size
        self.meet_thr = 0.1
        if growth == 'ce':
            self.start_growth = 'connect'
            self.goal_growth = 'extend'
        elif growth == 'cc':
            self.start_growth = 'connect'
            self.goal_growth = 'connect'
        elif growth == 'ec':
            self.start_growth = 'extend'
            self.goal_growth = 'connect'
        elif growth == 'ee':
            self.start_growth = 'extend'
            self.goal_growth = 'extend'
        else:
            raise ValueError("Invalid tree growth types!")
        
        self.rrt_start = RRT(env, self.step_size, self.start_growth)
        self.rrt_goal = RRT(env, self.step_size, self.goal_growth)
    
    def start_tree(self, start, goal):
        MotionPlanner.plan(self, start, goal)
        self.rrt_start.start_tree(start, goal)
        self.rrt_goal.start_tree(goal, start)
    
    def clear_tree(self):
        self.rrt_start.clear_tree()
        self.rrt_goal.clear_tree()
    
    def extend_tree(self):
        goalFound_g, new_goal_nodes = self.rrt_goal.extend_tree(stopAtGoal=True)
        if len(new_goal_nodes) != 0:
            target = new_goal_nodes[-1].state
        else:
            target = None
        goalFound_s, new_start_nodes = self.rrt_start.extend_tree(des_state=target, stopAtGoal=True)
        
        treesMet, meet_point = self.tree_meet(new_start_nodes, new_goal_nodes)
        return treesMet, meet_point, new_start_nodes, new_goal_nodes
    
    def grow_tree(self, start, goal):
        self.start_tree(start, goal)
        treesMet = False
        
        while not treesMet:
            treesMet, meet_point, start_nodes, goal_nodes = self.extend_tree()
        
        return meet_point
    
    def plan(self, start, goal):
        meet_point = self.grow_tree(start, goal)
        found_path, path_cost = self.find_path(meet_point)
        return found_path, path_cost
    
    def tree_meet(self, new_start_nodes=None, new_goal_nodes=None):
        for s in self.rrt_start.tree:
            for g in new_goal_nodes:
                if self.env.distance(s.state, g.state) < self.step_size:
                    return True, (s, g)
        for g in self.rrt_goal.tree:
            for s in new_start_nodes:
                if self.env.distance(s.state, g.state) < self.step_size:
                    return True, (s, g)
        return False, (None, None)
    
    def find_path(self, meet_point):
        start_meet = meet_point[0]
        goal_meet = meet_point[1]
        
        start_path = self.rrt_start.path_to_root(start_meet)
        goal_path = self.rrt_goal.path_to_root(goal_meet)
        path_cost = meet_point[0].cost + meet_point[1].cost + self.env.distance(meet_point[0].state, meet_point[1].state)
        
        goal_path.reverse()
        return goal_path + start_path, path_cost
