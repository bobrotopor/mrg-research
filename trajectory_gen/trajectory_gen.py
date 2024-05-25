import numpy as np
from numpy.typing import NDArray


class TrajGenGPR(object):


    def __init__(self, dt: float,  scan_vel: float, turning_vel: float = None) -> None:
        
        self.dt = dt
        self.scan_vel = scan_vel
        if turning_vel is None:
            self.turning_vel = scan_vel
        else:
            self.turning_vel = turning_vel

    def gen_gpr_scanning_traj(self, points: NDArray, line_types: list, gpr_flags: list):
        
        is_first_iter = True
        scan_traj = np.empty((0,3)) # time/x/y/gpr_flags
        line_types = ['pass'] + line_types
        gpr_flags = [0] + gpr_flags
        
        for idx in range(1, np.shape(points)[0]):
            p_start = points[idx - 1]
            p_end = points[idx]
            
            if line_types[idx] == 'line':
                if is_first_iter is True:
                    section_points = self.gen_line(p1=p_start, p2=p_end, init_time=0)
                    is_first_iter = False
                else:
                    section_points = self.gen_line(p1=p_start, p2=p_end, init_time=scan_traj[-1][0])

            if line_types[idx] == 'curve':
                continue

            # n = section_points.shape[0]
            # g_flags = gpr_flags[idx] + np.zeros((n,1))
            # part_tr
            scan_traj = np.vstack((scan_traj, section_points))
            print(f'iter={idx}')
            print(scan_traj)

        return scan_traj

            

    def gen_line(self, p1: NDArray, p2: NDArray, init_time: float) -> NDArray:
        """Сгенерировать отрезок прямой на плоскости между двумя опорными точками."""
        step =  self.scan_vel * self.dt
        line_len = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) 
        time = line_len / self.scan_vel

        n = int(np.ceil(line_len / step))
        t_arr = init_time + np.linspace(0, time, n)
        x_arr = p1[0] + np.linspace(0, p2[0], n)
        y_arr = p1[1] + np.linspace(0, p2[1], n)

        t_arr = t_arr.reshape((n,1))
        x_arr = x_arr.reshape((n,1))
        y_arr = y_arr.reshape((n,1))
        return np.hstack((t_arr, x_arr, y_arr))



if __name__ == '__main__':
    tj = TrajGenGPR(dt=0.1, scan_vel=0.5)

    points = np.array([[0,0],[0,1],[1,0.5],[0,0.5]])
    l_types = ['line', 'curve', 'line']
    gpr_flags = [1,0,1]
    tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)
    # p1 = np.array([0,0])
    # p2 = np.array([1,2])
    # print(tj.gen_line(p1, p2, 10))