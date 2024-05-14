import pandas as pd
from matplotlib import pyplot as plt 
from pathlib import Path

AMCL_N_NAV_DIR = Path(__file__).parent / 'amcl_n_nav'
AMCL_N_TELEOP_DIR = Path(__file__).parent / 'amcl_n_teleop'


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


def shift_arr_to_zero(data: pd.Series):
    return data - data[0]

def process_raw_df_dict(df_dict: dict[pd.DataFrame], shift_to_zero_param: str = 'time'):
    for key in df_dict.keys():
        param_history = df_dict[key][shift_to_zero_param]
        df_dict[key][shift_to_zero_param] = shift_arr_to_zero(param_history)

def head_df_dict(df_dict: dict[pd.DataFrame]):
    for key in df_dict.keys():
        print(df_dict[key].head())


def plot_df_dict(title: str, df_dict: dict[pd.DataFrame]):
    plt.figure(title)
    plt.title(title)
    legend = []
    for key in df_dict.keys():
        legend.append(key)
        plt.plot(df_dict[key]['time'], df_dict[key]['trans_err'])
    plt.legend(legend)
    plt.grid()


if __name__ == '__main__':

    ang30_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_ang/ang30_inplace.csv')
    ang45_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_ang/ang45_inplace.csv')
    ang60_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_ang/ang60_inplace.csv')
    ang90_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_ang/ang90_inplace.csv')

    ang_inplace = {
        'ang30': ang30_inplace, 
        'ang45': ang45_inplace, 
        'ang60': ang60_inplace, 
        'ang90': ang90_inplace,
    }
    process_raw_df_dict(ang_inplace)

    shift_1m_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_shift/shift_1m_inplace.csv')
    shift_2m_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_shift/shift_2m_inplace.csv')
    shift_3m_inplace = pd.read_csv(AMCL_N_TELEOP_DIR / 'inplace_shift/shift_3m_inplace.csv')

    inplace_shift = {
        'shift_1m': shift_1m_inplace, 
        'shift_2m': shift_2m_inplace, 
        'shift_3m': shift_3m_inplace, 
    }
    process_raw_df_dict(inplace_shift)

    shift_1m_vel_max = pd.read_csv(AMCL_N_TELEOP_DIR / 'shift_vel_max/shift_1m_vel_max.csv')
    shift_2m_vel_max = pd.read_csv(AMCL_N_TELEOP_DIR / 'shift_vel_max/shift_2m_vel_max.csv')
    shift_3m_vel_max = pd.read_csv(AMCL_N_TELEOP_DIR / 'shift_vel_max/shift_3m_vel_max.csv')

    shift_vel_max = {
        'shift_1m': shift_1m_vel_max, 
        'shift_2m': shift_2m_vel_max, 
        'shift_3m': shift_3m_vel_max, 
    }
    process_raw_df_dict(shift_vel_max)

    complex_2m_ang30 = pd.read_csv(AMCL_N_TELEOP_DIR / 'complex/complex_2m_ang30.csv')
    complex_2m_ang45 = pd.read_csv(AMCL_N_TELEOP_DIR / 'complex/complex_2m_ang45.csv')
    complex_3m_ang30 = pd.read_csv(AMCL_N_TELEOP_DIR / 'complex/complex_3m_ang30.csv')

    complex = {
        '2m_ang30': complex_2m_ang30, 
        '2m_ang45': complex_2m_ang45, 
        '3m_ang30': complex_3m_ang30, 
    }
    process_raw_df_dict(complex)

    # печать
    configure_mpl_plot()
    plot_df_dict('Начальная угловая ошибка\n(вращение МР на месте)', ang_inplace)
    plot_df_dict('Начальная ошибка смещения\n(вращение МР на месте)', inplace_shift)
    plot_df_dict('Начальная ошибка смещения\n(движение МР по прямой с макс. скоростью)', shift_vel_max)
    plot_df_dict('Начальное смещение и поворот\n(движение МР по прямой с макс. скоростью)', complex)
    plt.show()