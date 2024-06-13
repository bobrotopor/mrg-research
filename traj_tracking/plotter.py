"""Отрисовщик граификов моделирования слежения за траекторией."""


from logging import Logger

from matplotlib import pyplot as plt


def show():
    plt.show()


def configure_mpl_plot() -> None:
    """Устанавливает заданный стиль окнам matplotlib.

    ГОСТовский шрифт НЕ используем, так как он есть не на всех машинах.
    Поддержку Latex НЕ включаем, так как она есть не на всех машинах.

    Чтобы увидеть весь список параметров, можно вызвать: print(plt.rcParams)
    """
    plt.rcParams.update({
        'font.size': '16',
        'font.family': 'arial',
        'font.style': 'italic',
        'font.weight': 'bold',
        'axes.titlesize': 'medium',
        'axes.titleweight': 'bold',
        'axes.linewidth': '1.1',
        # 'text.usetex': True,
        # 'axes.prop_cycle': cycler('color',[...]),
    })


def plot_ctrl_mr(*lgrs: Logger):
    """Отрисовать управляющие скорости МР."""
    fig = plt.figure('Управляющие сигналы МР')
    fig.suptitle('Управляющие сигналы МР')
    gs = fig.add_gridspec(2, 1, figure=fig)
    vel = fig.add_subplot(gs[0, 0])
    omega = fig.add_subplot(gs[1, 0], sharex=vel)

    vel.set_ylabel('Линейная скорость, [м/с]')
    omega.set_ylabel('Угловая скорость, [рад/с]')
    omega.set_xlabel('Время, [с]')
    
    vel.grid()
    omega.grid()

    legend = []
    if len(lgrs) == 1:
        lgr = lgrs[0]
        vel.plot(lgr['time'], lgr['vel'], c='g')
        omega.plot(lgr['time'], lgr['omega'])
    else:
        for lgr in lgrs:
            vel.plot(lgr['time'], lgr['vel'])
            omega.plot(lgr['time'], lgr['omega'])
            legend.append(lgr.name)
    
        vel.legend(legend)
        omega.legend(legend)


def plot_errors(*lgrs: Logger):
    """Отрисовать ошибки."""
    fig = plt.figure('Ошибки управления')
    fig.suptitle('Ошибки управления')
    gs = fig.add_gridspec(3, 1, figure=fig)
    e1 = fig.add_subplot(gs[0, 0])
    e2 = fig.add_subplot(gs[1, 0], sharex=e1)
    e3 = fig.add_subplot(gs[2, 0], sharex=e1)

    e1.set_ylabel('Ошибка по X, [м]')
    e2.set_ylabel('Ошибка по Y, [м]')
    e3.set_ylabel('Угловая ошибка, [рад]')
    e3.set_xlabel('Время, [с]')
    
    e1.grid()
    e2.grid()
    e3.grid()

    legend = []
    if len(lgrs) == 1:
        lgr = lgrs[0]
        e1.plot(lgr['time'], lgr['errs'][:, 0], c='g')
        e2.plot(lgr['time'], lgr['errs'][:, 1], c='r')
        e3.plot(lgr['time'], lgr['errs'][:, 2])

    else:
        for lgr in lgrs:
            e1.plot(lgr['time'], lgr['errs'][:, 0])
            e2.plot(lgr['time'], lgr['errs'][:, 1])
            e3.plot(lgr['time'], lgr['errs'][:, 2])
            legend.append(lgr.name)
    
        e1.legend(legend)
        e2.legend(legend)
        e3.legend(legend)

