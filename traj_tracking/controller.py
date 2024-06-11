"""Контроллер модели МР."""

import numpy as np
from numpy.typing import NDArray
from numpy import sin, cos


def unpack_vec3(odom: NDArray) -> [float, float, float]:
    x = odom[0]
    y = odom[1]
    z = odom[2]
    return x,y,z


class VelocityModelMR():
    """Скоростная модель мобильного робота."""
    def __init__(
        self, 
        dt: float, 
        init_odom: list[float, float, float], 
        scan_v: float,
        max_v: float, 
        max_w: float,
        max_dvdt: float=None,
        max_dwdt: float=None,
    ):
        self.odom = np.array(init_odom)
        self.dt = dt
        self.scan_v = scan_v
        self.max_v = max_v 
        self.max_w = max_w 


    def tick(self, cmd_vel, cmd_omega):
        theta = self.odom[2]
        self.odom[0] += cmd_vel*cos(theta)*self.dt
        self.odom[1] += cmd_vel*sin(theta)*self.dt
        self.odom[2] += cmd_omega*self.dt
        return self.odom
    
        
class Controller():

    def __init__(
        self, 
        mr_model: VelocityModelMR,
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
        
        if self.ctrl_type == 'approx':
            err = self.calc_err(et_odom)
            vel,omega = self.approx_ctrl(et_theta, et_vel, et_omega, err)
        if self.ctrl_type == 'rot':
            err = self.calc_rot_method_err(et_odom)
            vel,omega = self.rot_method_ctrl(et_vel, et_omega, err)
        if self.sat_type == 'global':
            vel = self.sat(val=vel, val_max=self.mr_model.max_v)
            omega = self.sat(val=omega, val_max=self.mr_model.max_w)

        # эволюция местоположения модели МР
        self.mr_model.tick(cmd_vel=vel, cmd_omega=omega)

        return self.mr_model.odom, vel, omega
    
    def sat(self, val: float, val_max: float):
        """Функция лийнейного насыщения."""
        sign = np.sign(val)
        if abs(val) > val_max: 
            return val_max * sign 
        else:
            return val
