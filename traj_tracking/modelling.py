"""Функция моделирования управления МР."""

import numpy as np
from traj_gen import TrajGenGPR
from controller import Controller
from logger import Logger


def run_modelling(points, line_types, ctrl: Controller, lgr: Logger):

    tj = TrajGenGPR(dt=ctrl.mr_model.dt, vel=ctrl.mr_model.scan_v)
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=line_types)
    et_ctrl_ = tj.control_from_traj(traj)   # TODO: убрать костыль ! Отсюда вылазят скорости и угол поворота

    time = traj[:,0]
    n = traj.shape[0]
    et_odom = np.hstack((traj[:,1].reshape((n,1)), traj[:,2].reshape((n,1)), et_ctrl_[:,0].reshape((n,1))))
    et_ctrl = np.hstack((et_ctrl_[:,2].reshape((n,1)), et_ctrl_[:,1].reshape((n,1))))

    for idx in range(n):

        odom, vel, omega, errs = ctrl.tick(et_odom[idx], et_ctrl[idx])
        lgr.log('vel', vel)
        lgr.log('omega', omega)
        lgr.log('odom', odom)
        lgr.log('errs', errs)

    lgr.data_dict['time'] = time
    lgr.data_dict['et_odom'] = et_odom
    lgr.data_dict['et_ctrl'] = et_ctrl

    return n, time[-1]
