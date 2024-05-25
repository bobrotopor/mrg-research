

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt


class TrajGenGPR(object):


    def __init__(self, dt: float,  scan_vel: float, turning_vel: float = None) -> None:
        
        self.dt = dt
        self.scan_vel = scan_vel
        if turning_vel is None:
            self.turning_vel = scan_vel
        else:
            self.turning_vel = turning_vel

        self.traj = None


    def gen_gpr_scanning_traj(self, points: NDArray, line_types: list, gpr_flags: list):
        """Сгенерировать всю траекторию движения МР, с метками времени и флажками включения георадара."""
        is_first_iter = True
        scan_traj = np.empty((0,4)) # time/x/y/gpr_flags
        line_types = ['pass'] + line_types
        gpr_flags = [0] + gpr_flags
        
        for idx in range(1, np.shape(points)[0]):
            p_start = points[idx - 1]
            p_end = points[idx]
            
            if line_types[idx] == 'l':
                if is_first_iter is True:
                    section_points = self.gen_line(p1=p_start, p2=p_end, init_time=0)
                    is_first_iter = False
                else:
                    section_points = self.gen_line(p1=p_start, p2=p_end, init_time=scan_traj[-1][0])

            if line_types[idx] == 'c':
                continue

            n = section_points.shape[0]
            g_flags = gpr_flags[idx] + np.zeros((n,1))
            part_traj = np.hstack((section_points, g_flags))
            scan_traj = np.vstack((scan_traj, part_traj))

        self.traj = scan_traj
        return scan_traj

            

    def gen_line(self, p1: NDArray, p2: NDArray, init_time: float) -> NDArray:
        """Сгенерировать отрезок прямой на плоскости между двумя опорными точками."""
        step =  self.scan_vel * self.dt
        len_line = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) 
        dir_vec = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        dir_vec *= 1 / len_line

        n = int(np.ceil(len_line / step))
        t_arr = init_time + np.array([ idx*self.dt for idx in range(1,n+1)])
        x_arr = np.array([ p1[0] + idx*dir_vec[0]*step for idx in range(n)])
        y_arr = np.array([ p1[1] + idx*dir_vec[1]*step for idx in range(n)])

        t_arr = t_arr.reshape((n,1))
        x_arr = x_arr.reshape((n,1))
        y_arr = y_arr.reshape((n,1))
        return np.hstack((t_arr, x_arr, y_arr))


    def get_control_from_traj(self, traj: NDArray):
        
        theta = [0, 0]  
        omega = [0, 0]
        vel = [0, 0]
        for idx in range(2, traj.shape[0]):
            dx_dt = (traj[idx][1] - traj[idx-1][1]) / self.dt
            dy_dt = (traj[idx][2] - traj[idx-1][2]) / self.dt
            pr_dx_dt = (traj[idx-1][1] - traj[idx-2][1]) / self.dt
            pr_dy_dt = (traj[idx-1][2] - traj[idx-2][2]) / self.dt
            # vel.append(np.sqrt)
            ang = np.arctan2(dy_dt, dx_dt)
            pr_ang = np.arctan2(pr_dy_dt, pr_dx_dt)
            omega.append((ang-pr_ang)/self.dt)
            theta.append(ang)
            vel.append(np.sqrt(dx_dt**2 + dy_dt**2))
        
        return np.array(vel), np.array(omega), np.array(theta)
    


if __name__ == '__main__':
    tj = TrajGenGPR(dt=0.01, scan_vel=0.5)

    points = np.array([[0,0],[0,2],[0.25,2.25],[0.75,2.25], [1,2], [1,0]])
    l_types = ['l', 'l', 'l', 'l', 'l']
    gpr_flags = [1,0,1,1,1,1,1]
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)


    plt.figure('Траектория')
    plt.title('Траектория')
    plt.scatter(traj[:, 1], traj[:,2], c=traj[:, 0])
    plt.grid()



    vel, omega, theta = tj.get_control_from_traj(traj)

    plt.figure('Параметры управления')
    plt.title('Параметры управления')
    plt.plot(traj[:, 0][2:], theta[2:]*180/np.pi)
    plt.grid()
    plt.show()

