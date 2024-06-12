"""Сравнение регуляторов."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from controller import Controller, unpack_vec3, VelocityModelMR
from logger import Logger
from modelling import run_modelling
import plotter as tr_plt


if __name__ == '__main__':

    # ========== модель МР и контроллер ==========
    mr_model = VelocityModelMR(
        dt=0.01,
        init_odom=[0.1, -0.1, 0.57],
        scan_v=0.4,
        max_v=0.7,
        max_w=2.44,
        max_dvdt=0.5,
        max_dwdt=5.5,
    )
    
    mr_ctrl = Controller(
        k=[2,35,5], 
        ctrl_type='rot',
        sat_type='global',
        mr_model=mr_model,
    )

    # ========== параметры траектории =============
    points = np.array([[0,0],[0,1],[0.5,1],[0.5,-0.25], [1,-0.25], [1,1.25]])
    l_types = ['l', 'c', 'l', 'c', 'l']

    # ========== запуск моделирования =============
    lgr = Logger()
    num_steps, anim_time = run_modelling(points=points, line_types=l_types, ctrl=mr_ctrl, lgr=lgr)

    