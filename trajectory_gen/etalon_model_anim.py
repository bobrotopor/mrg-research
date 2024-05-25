

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from trajectory_gen import TrajGenGPR



if __name__ == '__main__':
    tj = TrajGenGPR(dt=0.01, scan_vel=0.5)

    points = np.array([[0,0],[0,2],[0.25,2.25],[0.75,2.25], [1,2], [1,0]])
    l_types = ['l', 'l', 'l', 'l', 'l']
    gpr_flags = [1,0,1,1,1,1,1]
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)


    vel, omega, theta = tj.get_control_from_traj(traj)
    plt.figure('Параметры управления')
    plt.title('Параметры управления')
    plt.plot(traj[:, 0], theta*180/np.pi)
    plt.grid()
    plt.show()
