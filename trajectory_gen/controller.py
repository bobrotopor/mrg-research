


import numpy as np
from numpy.typing import NDArray
from numpy import sin, cos


class Controller():

    def __init__(self, dt, k1, k2, init_x, init_y, init_theta) -> None:
        
        self.k1 = k1
        self.k2 = k2
        self.dt = dt
        # self.prev_odom = np.zeros(3)
        # self.prev_errors = np.zeros(3)
        
        self.odom =np.array([init_x, init_y, init_theta])
        self.err = np.zeros(3)


    def calc_err(self, et_odom: NDArray) -> NDArray:
        err = self.odom - et_odom
        self.err = err
        return err
    


    def tick(self, et_odom: NDArray, et_ctrl: NDArray):

        et_theta = et_odom[2]
        et_vel = et_ctrl[0] 
        et_omega = et_ctrl[1] 

        err = self.calc_err(et_odom)
        c1 = -np.array([cos(et_theta), sin(et_theta), 0])
        c2 = -np.array([et_vel*sin(et_theta), et_vel*cos(et_theta), self.k2])

        u1 = self.k1 * err @ c1
        u2 = err @ c2

        vel = et_vel + u1
        omega = et_omega + u2

        x = self.odom[0]
        y = self.odom[1]
        theta = self.odom[2]
        self.odom[0] = vel*cos(theta)*self.dt + x
        self.odom[1] = vel*sin(theta)*self.dt + y
        self.odom[2] = omega*self.dt + theta

        return self.odom
    
