"""Анимация движения МР при работе алгоритма слежения за траекторией."""

import numpy as np
from numpy.typing import NDArray
from matplotlib import pyplot as plt

from controller import Controller, unpack_vec3, VelocityModelMR
from logger import Logger
from modelling import run_modelling

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


def keyboard_handler(fig, anim_fun, frames_num, time_interval, lgr: Logger):
    """Выполнить инструкцию в соотвертствии с нажатой клавишей."""
    commands = '"s" - сохранить анимацию в mp4;\n' \
                '"p" - запустить анимацию в окне pyplot;\n' \
                '"g" - вывести все графики моделирования;\n' \
                '"другие клавиши" - выйти из программы.\n'
    
    key = input(commands)
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
        case 'g':
            plot_ctrl_mr(lgr)
            plt.show()
        case _:
            return
    
def activate_ffmpeg(ffmpeg_path):
    if not ffmpeg_path.is_file():
        raise Exception(
            f'Файл ffmpeg.exe отсутствует по пути {ffmpeg_path} !',
        )
    rcParams['animation.ffmpeg_path'] = ffmpeg_path

def plot_ctrl_mr(lgr: Logger):
    ctrl_fig = plt.figure('Управляющие сигналы МР')
    ctrl_fig.suptitle('Управляющие сигналы МР')
    gs = ctrl_fig.add_gridspec(2, 1, figure=ctrl_fig)
    vel = ctrl_fig.add_subplot(gs[0, 0])
    omega = ctrl_fig.add_subplot(gs[1, 0], sharex=vel)

    vel.set_ylabel('Линейная скорость, \n[м/с]')
    vel.plot(lgr['time'], lgr['vel'])
    vel.grid()

    omega.set_ylabel('Угловая скорость, \n[рад/с]')
    omega.set_xlabel('Время, [с]')
    omega.plot(lgr['time'], lgr['omega'])
    omega.grid()


if __name__ == '__main__':

    activate_ffmpeg(FFMPEG_EXE_PATH)

    # ========== модель МР и контроллер ==========
    mr_model = VelocityModelMR(
        dt=0.01,
        init_odom=[0, 0, 0.57],
        scan_v=0.4,
        max_v=0.7,
        max_w=2.44,
        max_dvdt=0.5,
        max_dwdt=5.5,
    )
    mr_ctrl = Controller(
        k=[2,10,5], 
        ctrl_type='rot',
        sat_type='global',
        mr_model=mr_model,
    )

    # ========== параметры траектории =============
    points = np.array([[0,0],[0,1],[0.5,1],[0.5,-0.25], [1,-0.25], [1,1.25]])
    l_types = ['l', 'c', 'l', 'c', 'l']

    # ========== запуск моделирования =============
    lgr = Logger()
    num_steps, anim_time = run_modelling(points=points, line_types=l_types, ctrl=mr_ctrl, lgr=lgr)

    # ========== настройка окна анимации =============
    plt.rcParams.update({
        'font.size': '16',
        'font.family': 'arial',
        'font.style': 'italic',
        'font.weight': 'bold',
        'axes.titlesize': 'medium',
        'axes.titleweight': 'bold',
        'axes.linewidth': '1.1',
    })
    width = 2
    shift_x = 1
    shift_y = 1
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(
        autoscale_on=False, 
        xlim=(-width + shift_x, width + shift_x), ylim=(-width + shift_y, width + shift_y),
    )
    ax.set_aspect('equal')
    ax.set_xlabel('X, [м]')
    ax.set_ylabel('Y, [м]')
    ax.grid()

    etalon, = ax.plot([], [], lw=3, c='g')
    real, = ax.plot([], [], lw=3, c='r')
    trace, = ax.plot([], [], 'o', lw=1, ms=1, c='r')

    ax.scatter(lgr['et_odom'][:, 0], lgr['et_odom'][:, 1], c=lgr['time'], s=0.8)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

    trace_x = []
    trace_y = []
    def animate(i):
        """Анимация."""

        anim_et_x, anim_et_y = get_arrow(lgr['et_odom'][i])
        anim_x, anim_y = get_arrow(lgr['odom'][i])
        trace_x.append(lgr['odom'][i][0])
        trace_y.append(lgr['odom'][i][1])

        etalon.set_data(anim_et_x, anim_et_y)
        real.set_data(anim_x, anim_y)
        trace.set_data(trace_x,trace_y)
        time_text.set_text(time_template % lgr['time'][i])
        return etalon, real, trace, time_text

    keyboard_handler(fig, animate, frames_num=num_steps, time_interval=anim_time, lgr=lgr)

    ani = animation.FuncAnimation(
        fig, animate, num_steps, interval=anim_time, blit=True)
    print('start saving animation')
    ani.save("traj_tracking/anim.mp4")
    print('saved')
