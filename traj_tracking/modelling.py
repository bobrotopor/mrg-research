

import numpy as np
from traj_gen import TrajGenGPR
from controller import Controller


def modelling_tracking(points, line_types, ctrl: Controller):
    # вот это все хранится в контроллере - scan_vel, max_vel, max_omega, dt
    tj = TrajGenGPR(dt=ctrl.dt, scan_vel=ctrl.scan_vel)
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=line_types)
    et_ctrl_ = tj.control_from_traj(traj)

    time = traj[:,0]
    n = traj.shape[0]
    et_odom = np.hstack((traj[:,1].reshape((n,1)), traj[:,2].reshape((n,1)), et_ctrl_[:,0].reshape((n,1))))
    et_ctrl = np.hstack((et_ctrl_[:,2].reshape((n,1)), et_ctrl_[:,1].reshape((n,1))))

