

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
        self.control = None


    def gen_gpr_scanning_traj(self, points: NDArray, line_types: list, gpr_flags: list):
        """Сгенерировать всю траекторию движения МР, с метками времени и флажками включения георадара."""
        is_first_iter = True
        scan_traj = np.empty((0,3)) # time/x/y/gpr_flags
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
                section_points = self.gen_circ_arc(prev_p1=scan_traj[-1][1:], p1=p_start, p2=p_end, init_time=scan_traj[-1][0])

            # n = section_points.shape[0]
            # g_flags = gpr_flags[idx] + np.zeros((n,1))
            # part_traj = np.hstack((section_points, g_flags))
            # scan_traj = np.vstack((scan_traj, part_traj))
            scan_traj = np.vstack((scan_traj, section_points))

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
    

    def gen_circ_arc(self, prev_p1: NDArray, p1: NDArray, p2: NDArray, init_time: float) -> NDArray:
        

        diam = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        to_center_vec = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        to_center_vec *= 1 / diam
        radius = diam/2
        center = radius * to_center_vec + p1

        to_center_vec3 = np.array([to_center_vec[0], to_center_vec[1], 0])
        prev_vec3 = np.array([p1[0] - prev_p1[0], p1[1] - prev_p1[1], 0])
        arc_build_dir = np.sign(np.cross(prev_vec3,to_center_vec3)[2]) 

        step =  self.scan_vel * self.dt
        ang_step = step/radius
        rot_ang = np.arctan2(-to_center_vec[1], -to_center_vec[0])
        
        n = int(np.ceil(np.pi * radius / step))
        t_arr = init_time + np.array([ idx*self.dt for idx in range(1,n+1)])
        x_arr = np.array([ center[0] + arc_build_dir*radius*np.cos(rot_ang+idx*ang_step) for idx in range(n)])
        y_arr = np.array([ center[1] + arc_build_dir*radius*np.sin(rot_ang+idx*ang_step) for idx in range(n)])
        
        t_arr = t_arr.reshape((n,1))
        x_arr = x_arr.reshape((n,1))
        y_arr = y_arr.reshape((n,1))
        
        return np.hstack((t_arr, x_arr, y_arr))

    def control_from_traj(self, traj: NDArray):
        
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
        
        n = traj.shape[0]
        theta = np.array(theta).reshape((n,1))
        omega = np.array(omega).reshape((n,1))
        vel = np.array(vel).reshape((n,1))
        return np.hstack((theta, omega, vel))
    
    
    def etalon_from_latest_calcs(self):
        ctrl = self.control_from_traj(self.traj)
        return np.hstack((self.traj, ctrl))
    


if __name__ == '__main__':
    tj = TrajGenGPR(dt=0.01, scan_vel=0.5)

    points = np.array([[0,0],[0,1],[0.35,1],[0.35,-0.25], [0.6,-0.25], [0.6,1.25]])
    l_types = ['l', 'c', 'l', 'c', 'l']
    gpr_flags = [1,0,1,1,1,1,1]
    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)

    plt.figure('Траектория')
    plt.title('Траектория')
    plt.scatter(traj[:, 1], traj[:,2], c=traj[:, 0])
    plt.grid()

    ctrl = tj.control_from_traj(traj)

    plt.figure('theta')
    plt.title('theta')
    plt.plot(traj[:, 0][2:], ctrl[:, 0][2:]*180/np.pi)
    plt.grid()

    plt.figure('velocity')
    plt.title('velocity')
    plt.plot(traj[:, 0][2:], ctrl[:, 2][2:])
    plt.grid()


    plt.show()

