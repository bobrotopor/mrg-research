import pandas as pd
import numpy as np
from matplotlib import pyplot as plt 
from pathlib import Path


AMCL_N_NAV_DIR = Path(__file__).parent / 'amcl_n_nav'
AMCL_N_TELEOP_DIR = Path(__file__).parent / 'amcl_n_teleop'
TRACKING_DIR = Path(__file__).parent / 'tracking'


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
    data-=data[0]
    return

def revert_ang_2pi(df:  dict[pd.DataFrame]):
    idx = df.index[df['rot_err'] < -2.5]
    df.loc[idx, 'rot_err'] += 2*np.pi
    return

def process_raw_df_dict(df_dict: dict[pd.DataFrame], shift_to_zero_param: str = 'time'):
    for key in df_dict.keys():
        shift_arr_to_zero(df_dict[key][shift_to_zero_param])
        revert_ang_2pi(df_dict[key])

def head_df_dict(df_dict: dict[pd.DataFrame]):
    for key in df_dict.keys():
        print(df_dict[key].head())


def plot_df_dict(title: str, df_dict: dict[pd.DataFrame]):
    
    legend = []

    fig = plt.figure(title)
    fig.suptitle(title)
    gs = fig.add_gridspec(2, 1)
    trans_err = fig.add_subplot(gs[0, 0])
    rot_err = fig.add_subplot(gs[1, 0], sharex=trans_err)

    trans_err.set(ylabel='Ошибка смещения, [м]')
    rot_err.set(xlabel='Время, [с]', ylabel='Ошибка угла, [рад]')

    for key in df_dict.keys():
        legend.append(key)
        trans_err.plot(df_dict[key]['time'], df_dict[key]['trans_err'], lw=2)
        rot_err.plot(df_dict[key]['time'], df_dict[key]['rot_err'], lw=2)

    trans_err.grid()
    rot_err.grid()
    
    trans_err.legend(legend)
    rot_err.legend(legend)



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

    tracking_race1 = pd.read_csv(TRACKING_DIR / 'tracking_race1.csv')
    tracking_race2 = pd.read_csv(TRACKING_DIR / 'tracking_race2.csv')
    tracking_race3 = pd.read_csv(TRACKING_DIR / 'tracking_race3.csv')

    tracking_race = {
        "race1": tracking_race1,
        "race2": tracking_race2,
        "race3": tracking_race3,
    }
    process_raw_df_dict(tracking_race)

    tracking_linear = {
        "linear_04":  pd.read_csv(TRACKING_DIR / 'tracking_04_vel.csv'),
        "linear_06":  pd.read_csv(TRACKING_DIR / 'tracking_06_vel.csv'),
    }
    process_raw_df_dict(tracking_linear)

    # печать
    turning = '(Движение МР: разворот на месте - 2.84 рад/с)'
    moving_forward = '(Движение МР: по прямой - 0.6 м/с)'
    init_loc_amcl = 'Рассогласование локализации AMCL'
    
    configure_mpl_plot()
    plot_df_dict(f'{init_loc_amcl} по углу\n{turning}', ang_inplace)
    plot_df_dict(f'{init_loc_amcl} по смещению\n{turning}', inplace_shift)
    plot_df_dict(f'{init_loc_amcl} по смещению\n{moving_forward}', shift_vel_max)
    plot_df_dict(f'{init_loc_amcl} по смещению и повороту\n{moving_forward}', complex)
    plot_df_dict('Отслеживание локации\n(Движение МР: произвольная траектория)', tracking_race)
    plot_df_dict(f'Отслеживание локации\n{moving_forward}', tracking_linear)
    plt.show()
