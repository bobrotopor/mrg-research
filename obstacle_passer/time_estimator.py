
import numpy as np
from matplotlib import pyplot as plt 


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

def eval_diff(w_arr, c_arr):
    l_bypass = 3*c_arr + 6*w_arr
    l_snake = c_arr + 8*w_arr
    return l_bypass - l_snake

if __name__ == '__main__':

    n = 100
    c_arr = np.linspace(0, 6, n)
    w_arr = np.linspace(0, 0.5, n)

    
    z = eval_diff(*np.meshgrid(w_arr, c_arr, sparse=True))
    
    title = 'Разность оценок длины пути при георазведке'
    configure_mpl_plot()
    plt.figure('Разность оценок длины пути при георазведке')
    plt.title(title)
    plt.contourf(w_arr, c_arr, z, levels=6) # , levels=20, cmap='nipy_spectral')
    plt.axvline(0, color='k')
    plt.axhline(0, color='k')

    plt.xlabel('Ширина сетки, [м]')
    plt.ylabel('Длина обхода препятствия, [м]')

    cbar=plt.colorbar()
    cbar.ax.set_ylabel('Разность оценок, [м]')
    plt.grid()
    plt.show()