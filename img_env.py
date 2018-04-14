# Created on Sat Apr 14 02:44:59 2018
# Author: Chaitanya Pb

#%% Package imports

import euts
import rrts
import numpy as np

#%% Holonomic Image Environment Definition

def HoloSteer(des_state, curr_state, step_size):
    err_state = euts.vec_combine(des_state, '-', curr_state)
    unit_vec = euts.vec_operate(err_state, '/', np.linalg.norm(err_state))
    new_state = euts.vec_combine(curr_state, '+', euts.vec_operate(unit_vec, '*', step_size)) 
    return new_state

class ImageEnv:
    def __init__(self, img):
        self.state_dim = 2
        self.img = img
        self.FREE = 0
        self.state_limits = [(0, 0), (img.shape[0]-1, img.shape[1]-1)]
    
    def distance(self, s1, s2, metric='euclidean'):
        return rrts.distance(s1, s2, metric)
    
    def within_limits(self, point):
        lim = self.state_limits
        for i in range(self.state_dim):
            if point[i] < lim[0][i] or point[i] > lim[1][i]:
                return False
        return True
    
    def in_collision(self, point):
        if self.within_limits(point) and self.img[point] == self.FREE:
            return False
        else:
            return True
    
    def steer(self, des_state, curr_state, step_size):
        new_state = HoloSteer(des_state, curr_state, step_size)
        new_state = euts.vec_operate(new_state, 'int')
        cost = self.distance(new_state, curr_state)
        if self.in_collision(new_state):
            success = False
        else:
            success = True
        return success, new_state, cost