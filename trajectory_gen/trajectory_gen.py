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
        
        scan_traj = np.empty((0,4)) # time/x/y/gpr_flags
        line_types = ['empty'] + line_types
        gpr_flags = [0] + gpr_flags
        
        for idx, z in enumerate(zip(points, line_types, gpr_flags)):
            print(f'idx={idx}, curr_point={z[0]}, curr_line_type={z[1]}, curr_gpr_flag={z[2]}')


if __name__ == '__main__':
    tj = TrajGenGPR(1,1)

    points = np.array([[0,0],[0,1],[1,0.5],[0,0.5]])
    l_types = ['line', 'curve', 'line']
    gpr_flags = [1,0,1]
    # gpr_flags = np.array([1,0,1])
    tj.gen_gpr_scanning_traj(points=points, line_types=l_types, gpr_flags=gpr_flags)