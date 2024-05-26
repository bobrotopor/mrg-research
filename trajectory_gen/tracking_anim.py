"""Анимация движения эталонной модели МР."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from trajectory_gen import TrajGenGPR
from tracking_ctrl import Controller

import matplotlib.animation as animation



def get_arrow(x,y,theta):
    arr_parts_len = 0.05
    base_len = 0.15
    arr_ang = 2.5
    arrow_end_x = base_len*np.cos(theta) + x
    arrow_end_y = base_len*np.sin(theta) + y

    x_arrow = [
        arrow_end_x,
        arr_parts_len*np.cos(theta + arr_ang) + arrow_end_x, 
        arr_parts_len*np.cos(theta - arr_ang) + arrow_end_x,
        arrow_end_x,
        x,
    ]
    y_arrow = [
        arrow_end_y,
        arr_parts_len*np.sin(theta + arr_ang) + arrow_end_y, 
        arr_parts_len*np.sin(theta - arr_ang) + arrow_end_y,
        arrow_end_y,
        y,
    ]
    return x_arrow, y_arrow


if __name__ == '__main__':

    tj = TrajGenGPR(dt=0.01, scan_vel=0.5)
    mr_ctrl = Controller(dt=0.01, k=[2,5,1], init_odom=[0.1, 0.2, 0.2], ctrl_type='rot')

    points = np.array([[0,0],[0,1],[0.35,1],[0.35,-0.25], [0.6,-0.25], [0.6,1.25]])
    l_types = ['l', 'c', 'l', 'c', 'l']
    gpr_flags = [1,0,1,1,1,1,1]
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)
    ctrl = tj.control_from_traj(traj)

    time = traj[:,0]
    et_x = traj[:,1]
    et_y = traj[:,2]
    et_theta = ctrl[:,0]
    et_omega = ctrl[:,1]
    et_vel = ctrl[:,2]
    

    plt.figure('Параметры управления')
    plt.title('Параметры управления')
    plt.plot(traj[:, 0], et_theta*180/np.pi)
    plt.grid()


    L = 2
    shift_x = 1
    shift_y = 1
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(
        autoscale_on=False, 
        xlim=(-L + shift_x, L + shift_x), ylim=(-L + shift_y, L + shift_y),
    )
    ax.set_aspect('equal')
    ax.grid()

    etalon, = ax.plot([], [], lw=3, c='g')
    real, = ax.plot([], [], lw=3, c='r')
    trace, = ax.plot([], [], 'o', lw=1, ms=1, c='r')

    ax.scatter(traj[:, 1], traj[:,2], c=traj[:, 0], s=0.8)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


    trace_x = []
    trace_y = []
    def animate(i):

        et_odom = np.array([et_x[i] , et_y[i], et_theta[i]])
        et_ctrl = np.array([et_vel[i], et_omega[i]])
        odom = mr_ctrl.tick(et_odom, et_ctrl)

        anim_et_x, anim_et_y = get_arrow(x=et_x[i], y=et_y[i], theta=et_theta[i])
        anim_x, anim_y = get_arrow(x=odom[0], y=odom[1], theta=odom[2])
        trace_x.append(odom[0])
        trace_y.append(odom[1])

        etalon.set_data(anim_et_x, anim_et_y)
        real.set_data(anim_x, anim_y)
        trace.set_data(trace_x,trace_y)
        time_text.set_text(time_template % time[i])
        return etalon, real, trace, time_text


    ani = animation.FuncAnimation(
        fig, animate, len(et_y), interval=time[-1], blit=True)
    plt.show()