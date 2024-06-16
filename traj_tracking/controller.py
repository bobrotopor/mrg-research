"""Контроллер модели МР."""

import numpy as np
from numpy.typing import NDArray
from numpy import sin, cos


def unpack_vec3(odom: NDArray) -> [float, float, float]:
    x = odom[0]
    y = odom[1]
    z = odom[2]
    return x,y,z


def linear_sat(val, val_max):
    """Функция лийнейного насыщения."""
    sign = np.sign(val)
    if abs(val) > val_max: 
        return val_max * sign 
    else:
        return val


class Saturator():

    def __init__(self, dt, max_v, max_w, max_dvdt, max_dwdt):
        self.dt = dt
        self.max_vels = np.array([max_v, max_w])
        self.max_accels = np.array([max_dvdt, max_dwdt])
        self.sat_cmd = np.zeros(2)

    def lsat(self, cmd):
        sat_cmd = np.zeros(2)
        for idx, max_cmd in enumerate(self.max_vels):
            sat_cmd[idx] = linear_sat(cmd[idx], max_cmd)
        return sat_cmd

    def accel_lim(self, cmd):
        for idx, max_a in enumerate(self.max_accels):
            dcmd = cmd[idx] - self.sat_cmd[idx]
            if abs(dcmd) / self.dt > max_a:
                self.sat_cmd[idx] += self.dt * max_a * np.sign(dcmd)
            else:
                self.sat_cmd[idx] = cmd[idx]
        return self.sat_cmd


class MobileRobot():
    """Модель мобильного робота."""
    def __init__(
        self, 
        dt: float, 
        init_odom: list[float, float, float], 
        scan_v: float,
        max_v: float, 
        max_w: float,
        max_dvdt: float,
        max_dwdt: float,
        cv: float = None,
        cw: float = None,
    ):
        self.init_odom = np.array(init_odom)
        self.odom = self.init_odom.copy()
        self.vels = np.zeros(2)
        self.dt = dt
        self.scan_v = scan_v
        self.max_v = max_v 
        self.max_w = max_w 
        self.max_dvdt = max_dvdt
        self.max_dwdt = max_dwdt
        # параметры динамики
        self.cv = cv
        self.cw = cw


    def tick(self, cmd_vel, cmd_omega):
        """Шаг модели."""

        if self.cv and self.cw is None:
            self.odom[2] += cmd_omega*self.dt
            theta = self.odom[2]
            self.odom[0] += cmd_vel*cos(theta)*self.dt
            self.odom[1] += cmd_vel*sin(theta)*self.dt
            

        else:
            self.vels[0] += self.dt*self.cv*(cmd_vel-self.vels[0])
            self.vels[1] += self.dt*self.cw*(cmd_omega-self.vels[1])
            self.odom[2] += self.vels[1]*self.dt
            theta = self.odom[2]
            self.odom[0] += self.vels[0]*cos(theta)*self.dt
            self.odom[1] += self.vels[0]*sin(theta)*self.dt

        return self.odom
    
    def reset_odom(self, new_odom: NDArray = None):
        """Переустановить значение параметров ориентации МР."""
        if new_odom is None:
            self.odom = self.init_odom
        else:
            self.odom = new_odom
    
        
class Controller():

    def __init__(
        self, 
        mr_model: MobileRobot,
        k: list, 
        ctrl_type: str ='approx', 
        sat_type: str = None,
    ) -> None:
        
        self.k1 = k[0]
        self.k2 = k[1]
        self.k3 = k[2]

        self.ctrl_type = ctrl_type
        self.sat_type = sat_type

        if ctrl_type != 'approx' and ctrl_type !='rot':
            raise Exception('Ошибка имени контроллера!')
        
        self.mr_model = mr_model
        self.sat = Saturator(
            dt=self.mr_model.dt, 
            max_v=self.mr_model.max_v,
            max_w=self.mr_model.max_w,
            max_dvdt=self.mr_model.max_dvdt, 
            max_dwdt=self.mr_model.max_dwdt,
        )



    def calc_err(self, et_odom: NDArray) -> NDArray:
        """Вычислить вектор ошибок православным методом - влоб."""
        err = self.mr_model.odom - et_odom
        return err
    
    def calc_rot_method_err(self, et_odom: NDArray) -> NDArray:
        """Вычислить вектор ошибок для метода матрицы поворота."""
        et_x,et_y,et_theta = unpack_vec3(et_odom)
        x,y,theta = unpack_vec3(self.mr_model.odom)

        err = np.zeros(3)
        err[0] =  cos(theta)*(et_x - x) + sin(theta)*(et_y - y)
        err[1] = -sin(theta)*(et_x - x) + cos(theta)*(et_y - y)
        err[2] = et_theta - theta
        return err
    
    def approx_ctrl(self, et_theta, et_vel, et_omega, err):
        """Регулятор, полученный методом приближений при малых ошибках."""
        c1 = -np.array([cos(et_theta), sin(et_theta), 0])
        c2 = -np.array([et_vel*sin(et_theta), et_vel*cos(et_theta), self.k2])

        u1 = self.k1 * err @ c1
        u2 = err @ c2

        vel = et_vel + u1
        omega = et_omega + u2

        return vel, omega
    
    def rot_method_ctrl(self, et_vel, et_omega, err):
        """Регулятор полученный методом матрицы поворота."""
        vel = et_vel*cos(err[2]) + self.k1*err[0]
        omega = et_omega + self.k2*err[1]*et_vel*sin(err[2])/err[2] + self.k3*err[2]
        return vel, omega

    def tick(self, et_odom: NDArray, et_ctrl: NDArray):
        """Один шаг управления."""
        et_theta = et_odom[2]
        et_vel = et_ctrl[0] 
        et_omega = et_ctrl[1]
        
        err_to_comparison = np.zeros(3)
        if self.ctrl_type == 'approx':
            err = self.calc_err(et_odom)
            err_to_comparison = err
            vel,omega = self.approx_ctrl(et_theta, et_vel, et_omega, err)
        if self.ctrl_type == 'rot':
            err = self.calc_rot_method_err(et_odom)
            err_to_comparison = self.calc_err(et_odom)
            vel,omega = self.rot_method_ctrl(et_vel, et_omega, err)
        if self.sat_type == 'global':
            sat_cmd = self.sat.lsat(np.array([vel, omega]))
            cmd = self.sat.accel_lim(sat_cmd)
            vel = cmd[0]
            omega = cmd[1]

        # эволюция местоположения модели МР
        self.mr_model.tick(cmd_vel=vel, cmd_omega=omega)

        return self.mr_model.odom, vel, omega, err_to_comparison
    