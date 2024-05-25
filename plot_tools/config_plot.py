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