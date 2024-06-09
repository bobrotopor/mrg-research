"""Анимация движения МР при работе алгоритма слежения за траекторией."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from traj_gen import TrajGenGPR
from controller import Controller, unpack_vec3
from logger import Logger

import matplotlib.animation as animation
from pathlib import Path
from matplotlib import rcParams

FFMPEG_EXE_PATH = Path(__file__).parent / 'ffmpeg.exe'


def get_arrow(odom: NDArray):
    """Получить координаты стрелки для визуализации положения МР.
    
    :param odom: одометрия формата x,y,theta.
    """
    arr_parts_len = 0.05
    base_len = 0.15
    arr_ang = 2.5
    x, y, theta = unpack_vec3(odom)
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


def start_anim(fig, anim_fun, frames_num, time_interval):
    """Запуск анимации или сохранение в mp4."""
    key = input('"s" - сохранить анимацию в mp4;\n'
                '"p" - запустить анимацию в окне pyplot;\n'
                '"любая другая клавиша" - выйти из программы\n')
    match key:
        case 's':
            ani = animation.FuncAnimation(
                fig, anim_fun, frames_num, interval=time_interval, blit=True)
            print('start saving animation')
            ani.save("trajectory_gen/anim.mp4")
            print('saved')
        case 'p': 
            ani = animation.FuncAnimation(
                fig, anim_fun, frames_num, interval=time_interval, blit=True)
            plt.show()
        case _:
            return
    
def activate_ffmpeg(ffmpeg_path):
    if not ffmpeg_path.is_file():
        raise Exception(
            f'Файл ffmpeg.exe отсутствует по пути {ffmpeg_path} !',
        )
    rcParams['animation.ffmpeg_path'] = ffmpeg_path


if __name__ == '__main__':

    activate_ffmpeg(FFMPEG_EXE_PATH)

    # ========== контроллер модели МР ==========
    mr_ctrl = Controller(
        dt=0.01, 
        k=[2,10,5], 
        init_odom=[0, 0, 0.57], 
        ctrl_type='rot',
        max_v=0.7,
        max_w=2.44,
        sat_type='global',
    )

    # ========== генерация траектории =============
    tj = TrajGenGPR(dt=0.01, scan_vel=0.35)

    points = np.array([[0,0],[0,1],[0.5,1],[0.5,-0.25], [1,-0.25], [1,1.25]])
    l_types = ['l', 'c', 'l', 'c', 'l']

    traj = tj.gen_gpr_scanning_traj(points=points, line_types=l_types)
    et_ctrl_ = tj.control_from_traj(traj)

    time = traj[:,0]
    n = traj.shape[0]
    et_odom = np.hstack((traj[:,1].reshape((n,1)), traj[:,2].reshape((n,1)), et_ctrl_[:,0].reshape((n,1))))
    et_ctrl = np.hstack((et_ctrl_[:,2].reshape((n,1)), et_ctrl_[:,1].reshape((n,1))))

    # ========== настройка окна анимации =============
    width = 2
    shift_x = 1
    shift_y = 1
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(
        autoscale_on=False, 
        xlim=(-width + shift_x, width + shift_x), ylim=(-width + shift_y, width + shift_y),
    )
    ax.set_aspect('equal')
    ax.grid()

    etalon, = ax.plot([], [], lw=3, c='g')
    real, = ax.plot([], [], lw=3, c='r')
    trace, = ax.plot([], [], 'o', lw=1, ms=1, c='r')

    ax.scatter(traj[:, 1], traj[:,2], c=traj[:, 0], s=0.8)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

    # ========== функция анимации и расчёта управления =============  
    # замечание! тк управление осуществляетя внутри функции анимации:
    #       1) анимация работает медленнее;
    #       2) если масштабировать окно впроцессе анимации позиция модели МР может сбиться.
    # TODO: исправить этот косяк, если отсанется время
    trace_x = []
    trace_y = []
    ctrl_logger = Logger()

    def animate(i):
        """Анимация."""
        odom, vel, omega = mr_ctrl.tick(et_odom[i], et_ctrl[i])
        ctrl_logger.log('vel', vel)
        ctrl_logger.log('omega', omega)
        ctrl_logger.log('time', time[i])

        anim_et_x, anim_et_y = get_arrow(et_odom[i])
        anim_x, anim_y = get_arrow(odom)
        trace_x.append(odom[0])
        trace_y.append(odom[1])

        etalon.set_data(anim_et_x, anim_et_y)
        real.set_data(anim_x, anim_y)
        trace.set_data(trace_x,trace_y)
        time_text.set_text(time_template % time[i])
        return etalon, real, trace, time_text

    start_anim(fig, animate, frames_num=n, time_interval=time[-1])

    ctrl_fig = plt.figure('Управляющие сигналы МР')
    gs = ctrl_fig.add_gridspec(2, 1, figure=ctrl_fig)
    vel = ctrl_fig.add_subplot(gs[0, 0])
    omega = ctrl_fig.add_subplot(gs[1, 0], sharex=vel)

    vel.set_ylabel('Линейная скорость, [м/с]')
    vel.plot(ctrl_logger['time'], ctrl_logger['vel'])
    vel.grid()

    omega.set_ylabel('Угловая скорость, [рад/с]')
    omega.set_xlabel('Время, [с]')
    omega.plot(ctrl_logger['time'], ctrl_logger['omega'])
    omega.grid()
    plt.show()