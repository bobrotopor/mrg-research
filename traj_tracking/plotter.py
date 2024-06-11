"""Отрисовщик граификов моделирования слежения за траекторией."""


from logging import Logger

from matplotlib import pyplot as plt


def plot_ctrl_mr(lgr: Logger):
    """Отрисовать управляющие скорости МР."""
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