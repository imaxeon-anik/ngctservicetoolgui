import pandas as pd
import os.path
from sklearn import metrics
from sklearn.linear_model import LinearRegression
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.patches import Rectangle
# import matplotlib as mpl
# mpl.style.use('seaborn-v0_8')


def plot_time_span(ax, range_list: list):
    """
    ax: is the axis from matplotlib
    range_list: the list contains a list of tuble of 3 entries (ts, te, value)
    the odd value will be darker than the light value
    """
    for ts, te, val in range_list:
        if val & 1 == 1:
            ax.axvspan(ts, te, facecolor='xkcd:sky blue', alpha=0.2)
        else:
            # ax.axvspan(ts, te, facecolor='xkcd:eggshell', alpha=0.1)
            pass


# noinspection DuplicatedCode
def plot_a_b_time_series(ax, df: pd.DataFrame, a_name, b_name,
                         a_y_axis_name=None,
                         b_y_axis_name=None,
                         highlight_list=None,
                         combine_legends=False):
    """
    Plot a and b on a primary and secondary axis
    :param ax:
    :param df:
    :param a_name: a column name or a list of column names
    :param b_name: a column name or a list of column names
    :param a_y_axis_name:
    :param b_y_axis_name:
    :param highlight_list:
    :param combine_legends:
    :return:
    """
    a_colors = ["blue", "lime", "aqua", "orange", "olive", "#bc13fe"]   # "#bc13fe" neon purple
    b_colors = ["brown", "cyan", "red", "purple", "indigo", "#fe019a"]  # "#fe019a" neon pink
    if a_y_axis_name is None:
        a_y_axis_name = a_name
    if b_y_axis_name is None:
        b_y_axis_name = b_name

    if type(a_name) == str:
        a_name = [a_name]

    if type(b_name) == str:
        b_name = [b_name]

    df.plot(x="T", y=a_name, ax=ax, color=a_colors, label=a_name)

    ax.set_ylabel(a_y_axis_name, color=a_colors[0])
    ax.tick_params(axis='y', color=a_colors[0])

    ax.set_xlabel("Time(s)")
    h1, l1 = ax.get_legend_handles_labels()
    if len(b_name):
        ax2 = ax.twinx()
        df.plot(x="T", y=b_name, ax=ax2, color=b_colors, label=b_name)

        ax2.tick_params(axis='y', color=b_colors[0])
        ax2.set_ylabel(b_y_axis_name, color=b_colors[0])

        h2, l2 = ax2.get_legend_handles_labels()
        if combine_legends:
            ax.get_legend().set_visible(False)
            ax2.get_legend().set_visible(False)
            if len(l1) > 0 and len(l2) > 0:
                l1 = [x + "(left)" for x in l1]
                l2 = [x + "(right)" for x in l2]
            ax.legend(h1 + h2, l1 + l2)
        else:
            ax.legend(loc=2)    # 2 = top left
            ax2.legend(loc=1)  # 1 = top right
            # ax2.legend(loc=1)   # 1 = top right
            # 5,7 = y-center-right
            # 6 = y-center-left

    ax.grid()

    if highlight_list is not None:
        plot_time_span(ax, highlight_list)


# copy from linear_regression:
def linear_regression(x_train, y_train):
    regressor = LinearRegression()
    regressor.fit(x_train, y_train)
    model_data = [regressor.intercept_] + list(regressor.coef_)
    # print("Linear regression", model_data)
    return regressor, model_data


# noinspection DuplicatedCode
def evaluate_regression(y_test, y_pred, verbose=True) -> tuple[float, float, float]:
    """
    return a tuple of mae, mse, and rmse of the model
    """
    mae = metrics.mean_absolute_error(y_test, y_pred)
    mse = metrics.mean_squared_error(y_test, y_pred)
    rmse = np.sqrt(metrics.mean_squared_error(y_test, y_pred))
    if verbose:
        print("Regression:")
        print('\tMean Absolute Error:', mae)
        print('\tMean Squared Error:', mse)
        print('\tRoot Mean Squared Error:', rmse)
    return mae, mse, rmse


# noinspection DuplicatedCode
def get_regression(df, x_key, y_key, verbose=True):
    """return a tuple of regressor, model_data, mae, mse, rmse"""
    x = df[x_key].to_numpy().reshape(-1, 1)
    y = df[y_key].to_numpy().reshape(-1, 1)
    regressor, model_data = linear_regression(x, y)
    if verbose:
        print(x_key, y_key, "Linear regression for motor %.2f %.2f" % (model_data[0][0], model_data[1][0]))

    estimated = regressor.predict(x)
    mae, mse, rmse = evaluate_regression(y, estimated, verbose=verbose)
    return regressor, model_data, mae, mse, rmse


# -----------------------------------------------------------------------------------


# noinspection DuplicatedCode,PyTypeChecker
class PIDAnalyser:
    # noinspection PyPep8Naming
    def __init__(self, filename, output_dir=".", trim=200, do_conversion=False, save_after_filtering=False):
        self.filename = filename

        # Expecting fields in CSV
        #   CMT,lag,T,Vt,Va,Pt,Pa,pid_error,pid_integral,pid_derivative,pid,last_u,Pr,effort,pressure
        # Motor,lag,T,Vt,Va,Pt,Pa,pid.e,pid.i,pid.d,pid,last_u,Pr,effort,pwm0,t1_adc,t2_adc,pressure,current,reserve
        self.df = pd.read_csv(filename)
        self.output_dir = output_dir
        self.output_prefix = os.path.join(output_dir, os.path.basename(filename))
        if len(self.df) <= 1:
            print("Empty data frame")
            return
        print("Loaded", filename, "data", len(self.df))
        self.df.sort_values(by=['T'], inplace=True)

        # find the first nonzero vt and keep the first zero sample:
        if len(self.df) and False:
            t0 = self.df.loc[0, 'T']  # first entry as T0
            print("t0", t0)
            self.df['T'] -= t0

        if do_conversion:
            self.df['pressure'] = self.df['pressure'] / 4096.0 * 500.00

            # current hack: CMT is used as current
            # From Centargo Motor current (ADC value 1.6V == 1Amp; using 5V reference 12 bit count)
            self.df['MCurrent'] = self.df['current'] / float(1 << 12) * 5.0 / 1.54 * 1000
            # self.df["MCurrent(mA)"] = self.df["MCurrent"] * 5 / 1023 / 1.54 * 1000
            # T,Vt,pos,target_pos,pid_error,pid_integral,pid,last_u

            # Convert positions become relative to the first data point:
            P0 = self.df['Pa'][0]
            self.df['Pa'] -= P0
            self.df['Pt'] -= P0

            # convert to revolution
            self.df['Pt'] /= 512.0
            self.df['Pa'] /= 512.0

            self.df['Va'] /= 256.0  # 24.8 fixed point count to Motor Count Per Interrupt (CPI)
            self.df['pid.i'] /= 256.0  # fixed point PID integral
            # todo Kp
            # self.df['pid_sum'] = self.df['pid_error'] * Kp + self.df['pid_integral'] + self.df['pid_derivative']

            # CPI to RPM
            # interrupt time = 300 us
            # 1 revolution = 512 count
            # RPS = CPI * 10^6/ 300 / 512
            # RPM = CPI * 60 * 10^6/ 300 / 512
            #     = CPI * 10^5 / 512
            self.df['Va'] = self.df['Va'] * 100000.0 / 256.0
            self.df['Vt'] = self.df['Vt'] * 100000.0 / 256.0

            # T = interrupt counts at 280200 ns interval
            # convert to seconds
            self.df['T'] = self.df['T'] * 280200 / 1000000000 / 5  # todo div by 5  is hacked

        else:
            self.df['MCurrent'] = self.df['current']

        if len(self.df) > 500:
            self.target_rpm = self.df['Vt'][500:].mean()
        else:
            self.target_rpm = self.df['Vt'].mean()

        self.trim = trim

        if save_after_filtering:
            self.df.to_csv(filename + "filter.csv", float_format="%f")

    @classmethod
    def trim_data(cls, cdf: pd.DataFrame, trim_head: bool, trim_tail: bool, gap=100, trim_per_motor=False) -> pd.DataFrame:
        if len(cdf) < gap:
            return cdf
        if trim_per_motor:
            indices = cdf.MotorIndex.unique()
            data = []
            for motor in indices:
                fdf = cls.trim_data(pd.DataFrame(cdf[cdf.MotorIndex == motor]), trim_head, trim_tail, gap, trim_per_motor=False)
                data.append(fdf)
            cdf = pd.concat(data)
        cdf = pd.DataFrame(cdf)

        nz_indices = np.where(cdf.ProgramEnd == 0)[0]
        if len(nz_indices) <= 1:
            return cdf
        start = 0
        end = len(cdf.ProgramEnd)

        # t0 = cdf.loc[nz_indices[0], 'T']  # the first entry where the program start - not working all the time
        t0 = cdf.iloc[nz_indices[0]]['T']  # the first entry where the program start - ok
        if trim_head:
            start = max(nz_indices[0] - gap, 0)
        if trim_tail:
            end = min(nz_indices[-1] + gap, end)
        cdf = pd.DataFrame(cdf[start:end])
        # offset the time since the first program start
        cdf["T"] = cdf["T"] - t0
        print("trim_data: Tmin, Tmax:", cdf["T"].min(), cdf["T"].max())
        return cdf

    def plot_it(self, save_figure=False):
        if len(self.df) <= 1:
            print("Too little data to plot")
            return
        indices = self.df.MotorIndex.unique()
        if self.trim > 0:
            # self.plot_df(self.df.take(range(0, self.trim)), save_figure, short_name)
            for i in indices:
                fdf = self.df[self.df.MotorIndex == i]
                trim_df = self.trim_data(fdf, trim_head=True, trim_tail=True)
                short_name = "%s-motor-%d-%d.png" % (os.path.splitext(self.output_prefix)[0], i, self.trim)
                self.plot_df(trim_df, save_figure, os.path.join(self.output_dir, short_name))

        for i in indices:
            fdf = self.df[self.df.MotorIndex == i]
            full_name = "%s-motor-%d.png" % (os.path.splitext(self.output_prefix)[0], i)
            self.plot_df(fdf, save_figure, os.path.join(self.output_dir, full_name))

        if not save_figure:
            plt.show()
        plt.clf()
        plt.close(fig='all')

    def plot_df(self, df: pd.DataFrame, save_figure, output_name):
        # hack way to swap the order of the plot quickly :)
        # dpi = 300 if save_figure else 96
        fig, (ax1, ax2, ax5, ax6, ax3, ax4) = plt.subplots(
            figsize=[6, 10], nrows=6, ncols=1, sharex=True,
            clear=True)
        fig.subplots_adjust(hspace=0.025, wspace=0)
        plt.grid()

        highlight_list = self.get_transition_time_ranges(df, "ProgramEnd")
        print("Transition list", highlight_list)

        plot_a_b_time_series(ax1, df, ["Vt", "Va"], ['T1'],
                             a_y_axis_name='Speed(mL/s)',
                             b_y_axis_name='Temperature(°C)',
                             highlight_list=highlight_list)

        plot_a_b_time_series(ax2, df, ["pid_d", "pid_i", "pid_e", "pid", "FilteredPid"], [],
                             a_y_axis_name='PID value',
                             b_y_axis_name=None,
                             highlight_list=highlight_list)

        plot_a_b_time_series(ax3, df, ["Pt", "Pa"], ['EncoderIndex'],
                             a_y_axis_name="Volume(ml)",
                             b_y_axis_name='Encoder Index Count',
                             highlight_list=highlight_list)

        plot_a_b_time_series(ax4, df, ["last_u", "Pr"], ["Va"],
                             a_y_axis_name="Rotor Position(count)",
                             b_y_axis_name="Va",
                             highlight_list=highlight_list)

        # plot current and lag angle with dual axis
        plot_a_b_time_series(ax5, df, 'MCurrent', 'pressure',
                             a_y_axis_name='Current(mA)',
                             b_y_axis_name='Pressure(psi)',
                             highlight_list=highlight_list)

        # effort, pressure with dual axis
        plot_a_b_time_series(ax6, df, "effort", ['PredictedPressureKpa', 'pressure'],
                             b_y_axis_name='Pressure(kpa)',
                             highlight_list=highlight_list)

        name = os.path.splitext(os.path.basename(output_name))[0]
        ax1.set_title(name)

        if save_figure:
            plt.savefig(output_name)
            plt.clf()
            plt.close(fig='all')
            print("Output", output_name)

    # noinspection PyUnresolvedReferences
    @staticmethod
    def get_last_active_interval(df: pd.DataFrame) -> tuple:
        mdf = pd.DataFrame(df)
        block_indices = (mdf.ProgramEnd != mdf.ProgramEnd.shift()).cumsum()
        blocks = sorted(block_indices.unique())
        for block in reversed(blocks):
            dff = pd.DataFrame(mdf[block_indices == block])
            # print("dff", len(dff), dff.shape)
            # when the motor is active (ProgramEnd==0)
            if dff['ProgramEnd'].max() == dff['ProgramEnd'].min() == 0.0:
                start_time = min(dff['T'])
                end_time = max(dff['T'])
                # print("start_time", start_time, end_time)
                return start_time, end_time

        start_time = mdf.iloc[0]['T']
        end_time = mdf.iloc[-1]['T']
        return start_time, end_time

    def plot_df_pressure_data(self, df: pd.DataFrame, save_figure, output_name, min_pressure_in_psi=10):
        # hack way to swap the order of the plot quickly :)
        dpi = 300 if save_figure else 96
        print("keys", df.keys())

        indices = df.MotorIndex.unique()
        if len(indices) != 2:
            print("ERROR: need both motor data. The available motor is", list(indices))
            return
        fdf0 = df.loc[self.df.MotorIndex == 0]
        fdf1 = df.loc[self.df.MotorIndex == 1]

        t00, t01 = self.get_last_active_interval(fdf0)
        t10, t11 = self.get_last_active_interval(fdf1)

        # the pull motor will briefly stop when the push motor stop, then it do pressure relief
        # So we want the one with the smaller time
        print("trim last active - use longest range")
        if t01 < t11:
            t_min, t_max = t00, t01
            calibrate_motor_index = 0
        else:
            t_min, t_max = t10, t11
            calibrate_motor_index = 1

        gap = 2  # +/- 2 seconds
        t_min -= gap
        t_max += gap
        df = pd.DataFrame(df[df['T'].between(t_min, t_max)])
        print("trim", df.shape)
        fdf0 = df.loc[df.MotorIndex == 0]
        fdf1 = df.loc[df.MotorIndex == 1]
        # print("fdf0", fdf0.shape)
        # print("fdf1", fdf1.shape)

        full_name = "%s-time.png" % (os.path.splitext(output_name)[0])

        fig, (ax1, ax2, ax3, ax4) = plt.subplots(
            figsize=[14, 8], nrows=4, ncols=1, clear=True,
            sharex=True, sharey=False, dpi=dpi)
        fig.subplots_adjust(hspace=0.025, wspace=0)

        highlight_list_0 = self.get_transition_time_ranges(fdf0, "ProgramEnd")
        highlight_list_1 = self.get_transition_time_ranges(fdf1, "ProgramEnd")

        # Motor 0
        plot_a_b_time_series(ax1, fdf0, ["Vt", "Va", "Vl"], ["MCurrent"],
                             a_y_axis_name="M0 Speed(ml/s)",
                             b_y_axis_name="AngleOffset",
                             highlight_list=highlight_list_0)

        plot_a_b_time_series(ax2, fdf0, 'FilteredPid', ['PredictedPressureKpa', 'pressure'],
                             a_y_axis_name='M0 FilteredPid',
                             b_y_axis_name='Pressure(kpa)',
                             highlight_list=highlight_list_0)

        # Motor 1
        plot_a_b_time_series(ax3, fdf1, 'FilteredPid', ['PredictedPressureKpa', 'pressure'],
                             a_y_axis_name='M1 FilteredPid',
                             b_y_axis_name='Pressure(kpa)',
                             highlight_list=highlight_list_1)

        plot_a_b_time_series(ax4, fdf1, ["Vt", "Va", "Vl"], ["MCurrent"],
                             a_y_axis_name="M1 Speed(ml/s)",
                             b_y_axis_name="AngleOffset",
                             highlight_list=highlight_list_1)

        name = os.path.splitext(os.path.basename(full_name))[0]
        ax1.set_title(name)
        if save_figure:
            plt.savefig(full_name)
            print("output ", full_name)
        else:
            plt.show()
        plt.clf()
        plt.close(fig='all')

        # restore the gap for time series plotting
        t_min += gap
        t_max -= gap
        dt = t_max - t_min
        t_max -= (dt / 20)  # 5% off the end

        t_min += (dt / 10)  # ignore 10% of the initial assume pressure built up
        # t_max -= 0.1        # ignore a fraction the time before it stop. Need ?
        adf0 = pd.DataFrame(fdf0[fdf0['T'].between(t_min, t_max)])
        adf1 = pd.DataFrame(fdf1[fdf1['T'].between(t_min, t_max)])
        # print("adf0", adf0.shape)
        # print("adf1", adf1.shape)
        # filter by pressure

        # filter out pressure below the threshold because the pressure build up has not

        if min_pressure_in_psi < max(df['pressure']):
            adf0 = pd.DataFrame(adf0[adf0['pressure'] >= min_pressure_in_psi])
            adf1 = pd.DataFrame(adf1[adf1['pressure'] >= min_pressure_in_psi])
        else:
            print("WARNING: pressure is lower than the min pressure - might observe some non-linearity",
                  min_pressure_in_psi, max(df['pressure']))
        base_name = os.path.splitext(output_name)[0]
        adf0.to_csv(base_name + "-active-0.csv", index=False)
        adf1.to_csv(base_name + "-active-1.csv", index=False)

        for mdf in [adf0, adf1]:
            if len(mdf) <= 1:
                print("Skip due to empty data")
                continue
            print("len(mdf)", len(mdf), mdf.shape)
            vt = mdf['Vt'].mean()
            regressor, model_data, mae, mse, rmse = get_regression(mdf, "FilteredPid", "pressure")
            x_min = mdf['FilteredPid'].min()
            x_max = mdf['FilteredPid'].max()
            if x_min == x_max:
                print("skip due to empty regression PID", x_min, x_max)
                continue
            motor_index = mdf.iloc[0].MotorIndex
            if calibrate_motor_index != motor_index:
                continue
            print("time", x_min, x_max, "range", x_max - x_min, "shape", mdf.shape)

            base_name = os.path.splitext(output_name)[0]
            full_csv_name = "%s-motor-%d-pressure.csv" % (base_name, motor_index)
            mdf.to_csv(full_csv_name, index=False)
            print("output", full_csv_name)

            full_name = "%s-motor-%d-pressure-scatter.png" % (base_name, motor_index)
            fig, (ax0, ax1) = plt.subplots(
                figsize=[14, 8], nrows=1, ncols=2, clear=True, sharex=False, sharey=False,
                dpi=dpi)

            plot_a_b_time_series(ax0, mdf, 'FilteredPid', 'pressure',
                                 a_y_axis_name='FilteredPid',
                                 b_y_axis_name='Pressure(psi)')

            x0 = np.arange(x_min, x_max, 0.5).reshape(-1, 1)
            yhat0 = regressor.predict(x0)
            equation = "P(v=%.1f) = %.2f + %.2fX" % (vt, model_data[0][0], model_data[1][0])

            mdf.plot.scatter(x="FilteredPid", y='pressure', c='Brown', marker="+", ax=ax1, label="Pressure vs PID(A)")
            ax1.plot(x0, yhat0, label=equation)
            ax1.grid()
            ax1.legend()
            ax1.set_title("Scatter: Pressure vs PID")
            ax0.set_title("Pressure and PID over time")
            fig.suptitle("%s - Motor %d" % (name, motor_index))

            if save_figure:
                plt.savefig(full_name)
                print("output ", full_name)
            else:
                plt.show()
            plt.close(fig='all')

    def plot_df_fields(self, df: pd.DataFrame, left_fields: list, right_fields: list, combine_motors: bool, save_figure: bool, output_name: str, show_figure: bool):
        # dpi = 300 if save_figure else 96
        dpi = None
        indices = df.MotorIndex.unique()
        if len(indices) != 2:
            print("ERROR: need both motor data. The available motor is", list(indices))
            return
        fdf0 = df.loc[self.df.MotorIndex == 0]
        fdf1 = df.loc[self.df.MotorIndex == 1]
        highlight_list_0 = self.get_transition_time_ranges(fdf0, "ProgramEnd")
        highlight_list_1 = self.get_transition_time_ranges(fdf1, "ProgramEnd")

        if combine_motors:
            full_name = "%s-selected-all-motors.png" % (os.path.splitext(output_name)[0])

            fig, (ax1, ax2) = plt.subplots(figsize=[14, 8], nrows=2, ncols=1, clear=True, sharex=True, sharey=False, dpi=dpi)
            if len(fdf0) > 0:
                plot_a_b_time_series(ax1, fdf0, left_fields, right_fields,
                                     a_y_axis_name=", ".join(left_fields),
                                     b_y_axis_name=", ".join(right_fields),
                                     highlight_list=highlight_list_0)
                ax1.set_title("Motor 0")
            if len(fdf1) > 0:
                ax2.set_title("Motor 1")
                plot_a_b_time_series(ax2, fdf1, left_fields, right_fields,
                                     a_y_axis_name=" ".join(left_fields),
                                     b_y_axis_name=" ".join(right_fields),
                                     highlight_list=highlight_list_1)

            name = os.path.splitext(os.path.basename(full_name))[0]
            fig.suptitle(name)
            if save_figure:
                plt.savefig(full_name)
                print("output ", full_name)
        else:
            for motor_idx, fdf in enumerate([fdf0, fdf1]):
                if len(fdf) == 0:
                    continue

                highlight_list = self.get_transition_time_ranges(fdf, "ProgramEnd")
                fig, ax1 = plt.subplots(figsize=[14, 8], nrows=1, ncols=1, clear=True, sharex=True, sharey=False, dpi=dpi)
                plot_a_b_time_series(ax1, fdf, left_fields, right_fields,
                                     a_y_axis_name=" ".join(left_fields),
                                     b_y_axis_name=" ".join(right_fields),
                                     highlight_list=highlight_list)

                full_name = "%s-selected-motor-%d.png" % (os.path.splitext(output_name)[0], motor_idx)
                name = os.path.splitext(os.path.basename(full_name))[0]
                fig.suptitle(name)
                if save_figure:
                    plt.savefig(full_name)
                    print("output ", full_name)

        if show_figure:
            plt.show()
        plt.clf()
        plt.close(fig='all')

    def plot_df_motor_current(self, df: pd.DataFrame, save_figure, output_name):
        # hack way to swap the order of the plot quickly :)
        dpi = 300 if save_figure else 96
        print("keys", df.keys())

        indices = df.MotorIndex.unique()
        for i in indices:
            fdf = df.loc[self.df.MotorIndex == i]
            full_name = "%s-motor-%d.png" % (os.path.splitext(output_name)[0], i)
            fig, (ax1, ax2, ax3) = plt.subplots(
                figsize=[6, 10], nrows=1, ncols=3, clear=True, sharex=True, sharey=False,
                dpi=dpi)

            plot_a_b_time_series(ax1, fdf, 'current', ['Vt', 'Va'], a_y_axis_name='Current(mA)',
                                 b_y_axis_name='Speed')
            plot_a_b_time_series(ax2, fdf, 'current', ["pid", "FilteredPid"], a_y_axis_name='Current(mA)',
                                 b_y_axis_name='Speed')
            plot_a_b_time_series(ax3, fdf, 'current', ['AngleOffset'], a_y_axis_name='FilteredPid',
                                 b_y_axis_name='Offset and Lag')

            name = os.path.splitext(os.path.basename(full_name))[0]
            ax1.set_title(name)

            if save_figure:
                plt.savefig(full_name)

        if not save_figure:
            plt.show()
        plt.close(fig='all')

    @staticmethod
    def get_transition_time_ranges(df: pd.DataFrame, field_name="ProgramEnd") -> list:
        """ return a list of tuple defined the range  (start_time, end_time, value) """
        df = pd.DataFrame(df)
        states = df[field_name].unique()
        idx = df[field_name]

        # noinspection PyUnresolvedReferences
        blocks = (idx != idx.shift()).cumsum()
        df['block'] = blocks
        print("end-states", states, "blocks", blocks.unique())

        range_list = list()
        start_time = df["T"].min()
        for state in df['block'].unique():
            dff = df[df['block'] == state]
            end_time = dff["T"].max()
            range_list.append((start_time, end_time, dff[field_name].min()))
            start_time = end_time
        return range_list

    def plot_df_with_motor_state_colors(self, df: pd.DataFrame):
        # hack way to swap the order of the plot quickly :)
        # dpi = 300 if save_figure else 96

        df = pd.DataFrame(df)

        fig, (ax1, ax2) = plt.subplots(
            figsize=[6, 10], nrows=2, ncols=1, sharex=True,
            clear=True)
        plt.grid()

        highlight_list = self.get_transition_time_ranges(df, "ProgramEnd")
        print("Transition list", highlight_list)

        plot_a_b_time_series(ax1, df, ["Vt", "Va", "Vl"], "Vl",
                             a_y_axis_name="Speed(ml/s)",
                             b_y_axis_name="Vl", highlight_list=highlight_list)

        plot_a_b_time_series(ax2, df, ["Vt", "Va", "Vl"], ["ProgramEnd"],
                             a_y_axis_name="Speed(ml/s)",
                             b_y_axis_name="Motor state", highlight_list=None)

        plt.show()

    def eval_pid(self):
        # part 1: rising
        # part 2: steady state

        # diff between actual speed and target speed normalised over the target speed
        dv = (self.df["Va"] - self.df["Vt"]) / self.df["Vt"]
        dt = self.df['T'].diff()
        err = dv[1:] * dt[1:]  # entry zero is nan
        l2_err = np.dot(err, err)
        l1_err = np.sum(np.abs(err))
        vt_0 = self.df.loc[0, "Vt"]
        return l2_err, l1_err, self.get_raising_time(vt_0)

    def get_raising_time(self, vt: float):
        """
        :param vt: the target speed
        :return: the duration for Va to go from 10% to 90%
        """
        v10 = vt * 0.1
        v90 = vt * 0.9
        if vt > 0.0:
            indices = np.bitwise_and(self.df['Va'] >= v10, self.df['Va'] <= v90)
        else:
            indices = np.bitwise_and(self.df['Va'] <= v10, self.df['Va'] >= v90)
        nzi = np.nonzero(indices.to_numpy())[0]
        if len(nzi) == 0:
            return 0.0
        # Find a gap where the index not 1-step increment
        delta = nzi[1:] - nzi[0:-1]
        start = nzi[0]
        not_1_step_index = np.where(delta != 1)[0]
        if len(not_1_step_index):
            end = nzi[not_1_step_index[0]]
        else:
            end = nzi[-1]
        t_start = self.df['T'][start]
        # v_start = self.df['Va'][start]
        t_end = self.df['T'][end]
        # v_end = self.df['Va'][end]
        t_raised = t_end - t_start
        # dv = v_end - v_start

        # find the first entry that speed over 100%
        if vt > 0:
            indices = self.df['Va'] >= vt
        else:
            indices = self.df['Va'] <= vt
        arr = np.nonzero(indices.to_numpy())[0]
        if len(arr) == 0:
            # Vt was never reach so give it a large number
            t_100 = 99999999999
        else:
            t_100 = self.df['T'][arr[0]]

        if t_100 == 0:
            return 0, t_raised, t_100
        return t_raised / t_100, t_raised, t_100


def plot_all(output_dir: str, trim: int, save_figure: bool):
    if not os.path.exists(output_dir):
        print("Invalid output directory", output_dir)
        return
    for filename in os.listdir(output_dir):
        if not filename.endswith(".csv"):
            continue
        full_name = os.path.join(output_dir, filename)
        print("loading", full_name)
        analyser = PIDAnalyser(full_name, output_dir, trim=trim)
        analyser.plot_it(save_figure=save_figure)


def process_csv(filename: str, output_dir: str, want_pressure_data: bool, save_figure: bool):
    print("Processing", filename)
    analyser = PIDAnalyser(filename, output_dir)
    if not want_pressure_data:
        analyser.df = analyser.trim_data(analyser.df, trim_head=True, trim_tail=True, trim_per_motor=True)
        analyser.plot_it(save_figure)
        if len(analyser.df) > 1:
            print(filename, "L2 error", analyser.eval_pid())
    else:
        output_name = os.path.abspath(filename)
        analyser.plot_df_pressure_data(analyser.df, save_figure, output_name)


def analyse_pid_directory(output_dir: str, want_pressure_data: bool, save_figure: bool):
    for filename in os.listdir(output_dir):
        full_name = os.path.join(output_dir, filename)
        if not filename.endswith(".csv"):
            if os.path.isdir(full_name):
                print("Analysing", full_name)
                analyse_pid_directory(full_name, want_pressure_data, save_figure)  # recursively call it
            continue
        process_csv(full_name, output_dir, want_pressure_data, save_figure)


def main(csv_filename_or_directory: str, want_pressure_data: bool, save_figure: bool):
    if os.path.isdir(csv_filename_or_directory):
        analyse_pid_directory(csv_filename_or_directory, want_pressure_data, save_figure)
    else:
        output_dir = os.path.dirname(csv_filename_or_directory)
        process_csv(csv_filename_or_directory, output_dir, want_pressure_data, save_figure)
    print("done")


def main1():
    analyser = PIDAnalyser("C:\\tmp\\PID\\Round_1_P\\Vt_20.0000_PID_0.200_0.03000_0.002.csv", trim=100)
    analyser.plot_it(False)
    print("L2 error", analyser.eval_pid())
    print("Raising time", analyser.get_raising_time(analyser.df['Vt'][0]))
    print("DONE")


def plot_with_motor_state_color(filename):
    analyser = PIDAnalyser(filename, trim=100)
    df = analyser.trim_data(analyser.df, True, True)
    fdf0 = df.loc[df.MotorIndex == 0]
    # fdf1 = df.loc[df.MotorIndex == 1]

    analyser.plot_df_with_motor_state_colors(fdf0)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='PID Data Analysis')
    parser.add_argument('csv', help='The CSV PID filename or the directory contain the PID CSV files')
    parser.add_argument('--pressure', action='store_true', help='Analyse pressure data')
    parser.add_argument('--save_figure', action='store_true', help='Save the figure')
    args = parser.parse_args()
    main(args.csv, args.pressure, args.save_figure)
    # plot_with_motor_state_color(args.csv)
