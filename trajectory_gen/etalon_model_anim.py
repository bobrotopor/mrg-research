"""Анимация движения эталонной модели МР."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from trajectory_gen import TrajGenGPR

import matplotlib.animation as animation



def get_arrow(x,y,theta):
    arrow_len = 0.05
    x_arrow = [
        arrow_len*np.cos(theta + 2*np.pi/3) + x, 
        arrow_len*np.cos(theta - 2*np.pi/3) + x,
        x,
    ]
    y_arrow = [
        arrow_len*np.sin(theta + 2*np.pi/3) + y, 
        arrow_len*np.sin(theta - 2*np.pi/3) + y,
        y,
    ]
    return x_arrow, y_arrow

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

    line, = ax.plot([], [], lw=3, c='g')
    ax.scatter(traj[:, 1], traj[:,2], c=traj[:, 0], s=0.8)
    # trace, = plt.ax.plot([], [], '.-', lw=1, ms=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


    def animate(i):
        x_ = x[i] + vel[i]*np.cos(theta[i])
        y_ = y[i] + vel[i]*np.sin(theta[i])
        x_arrow, y_arrow = get_arrow(x=x_, y=y_, theta=theta[i])
        thisx = [x[i], x_] + x_arrow    
        thisy = [y[i], y_] + y_arrow     

        line.set_data(thisx, thisy)
        time_text.set_text(time_template % time[i])
        return line, time_text


    ani = animation.FuncAnimation(
        fig, animate, len(y), interval=time[-1], blit=True)
    plt.show()
