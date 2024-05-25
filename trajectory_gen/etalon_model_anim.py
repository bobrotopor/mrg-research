"""Анимация движения эталонной модели МР."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from trajectory_gen import TrajGenGPR

import matplotlib.animation as animation


if __name__ == '__main__':

    tj = TrajGenGPR(dt=0.01, scan_vel=0.5)

    points = np.array([[0,0],[0,2],[0.25,2.25],[0.75,2.25], [1,2], [1,0]])
    l_types = ['l', 'l', 'l', 'l', 'l']
    gpr_flags = [1,0,1,1,1,1,1]
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)


    vel, omega, theta = tj.get_control_from_traj(traj)

    time = traj[:,0]
    x = traj[:,1]
    y = traj[:,2]
    

    plt.figure('Параметры управления')
    plt.title('Параметры управления')
    plt.plot(traj[:, 0], theta*180/np.pi)
    plt.grid()


    L = 3
    shift_x = 2
    shift_y = 2
    fig = plt.figure(figsize=(5, 4))
    ax = fig.add_subplot(
        autoscale_on=False, 
        xlim=(-L + shift_x, L + shift_x), ylim=(-L + shift_y, L + shift_y),
    )
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'o-', lw=2)
    ax.scatter(traj[:, 1], traj[:,2], c=traj[:, 0], s=0.8)
    # trace, = plt.ax.plot([], [], '.-', lw=1, ms=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


    def animate(i):
        thisx = [x[i]]      # , x[i] + 0.3*np.cos(theta[i])
        thisy = [y[i]]      # , x[i] + 0.3*np.sin(theta[i])

        line.set_data(thisx, thisy)
        time_text.set_text(time_template % time[i])
        return line, time_text


    ani = animation.FuncAnimation(
        fig, animate, len(y), interval=time[-1], blit=True)
    plt.show()
