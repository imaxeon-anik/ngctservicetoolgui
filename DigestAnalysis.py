import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pid_analysis import plot_a_b_time_series, get_regression
import glob

# xkcd color map https://matplotlib.org/stable/tutorials/colors/colors.html
speed_colors = ['aqua', 'azure', 'beige', 'blue', 'brown', 'chocolate', 'coral', 'crimson', 'cyan', 'darkblue',
                'darkgreen',
                'fuchsia', 'gold']
speed_color_map = {ix: x for ix, x in enumerate(speed_colors)}


def get_speed_color(speed_x100: int) -> str:
    index = speed_x100 // 100
    if index in speed_color_map:
        return speed_color_map[index]
    print("WARNING: can't map %d to a color, pick black" % speed_x100)
    return 'black'


class DigestAnalysis:
    def __init__(self):
        self.digest_csv_filename = ""
        self.df = pd.DataFrame()

    def load(self, digest_csv_filename):
        self.digest_csv_filename = digest_csv_filename
        self.df = pd.read_csv(self.digest_csv_filename)
        print("Loading", digest_csv_filename, self.df.shape)
        # print("fields", self.df.columns)
        self.df['Pressure(psi)'] = self.df.PressureADC / 4095.0 * (
                    500.00 + 14.7) - 14.7  # sensor conversion 0-5V = -14-500psi
        self.df['T(s)'] = self.df['T(ms)'] / 1000
        self.df['T'] = self.df['T(s)']  # for plotting

    def plot_pressure(self):
        ax = self.df.plot("T(s)", ["Pressure(psi)", 'ContrastPressureKpa', 'FlushPressureKpa'])
        ax2 = ax.twinx()
        self.df.plot("T(s)", ["FlushCurrentADC", 'ContrastCurrentADC'], ax=ax2)
        ax.grid()
        plt.show()

    def get_last_active_interval(self, motor_str: str) -> tuple:
        """Get the last syringe active duration duration"""
        mdf = pd.DataFrame(self.df)
        if motor_str.upper() == "FLUSH":
            status = self.df.FlushSyringeStatus == 'PROCESSING'
        else:
            status = self.df.ContrastSyringeStatus == 'PROCESSING'
        block_indices = (status != status.shift()).cumsum()
        blocks = sorted(block_indices.unique())
        mdf['status'] = status
        for block in reversed(blocks):
            dff = pd.DataFrame(mdf[block_indices == block])
            # print("dff", len(dff), dff.shape)
            if dff['status'].max() == dff['status'].min() == 1.0:
                start_time = min(dff['T(ms)'])
                end_time = max(dff['T(ms)'])
                return start_time, end_time

        start_time = mdf.iloc[0]['T(ms)']
        end_time = mdf.iloc[-1]['T(ms)']
        return start_time, end_time

    def filter_pressure_calibration_data(self, min_pressure_in_psi: int):
        ts_min, ts_max = self.get_last_active_interval("FLUSH")
        tc_min, tc_max = self.get_last_active_interval("CONTRAST")
        df = self.df
        flush_df = pd.DataFrame(df[df['T(ms)'].between(ts_min, ts_max)])
        contrast_df = pd.DataFrame(df[df['T(ms)'].between(tc_min, tc_max)])
        flush_vt_average = flush_df['FlushFlow_x100'].mean()
        contrast_vt_average = flush_df['ContrastFlow_x100'].mean()
        print("Flush speed", flush_vt_average, "Contrast speed", contrast_vt_average)

        if flush_vt_average > 0:  # the contrast motor must be the pull motor
            cal_motor = "FLUSH"
            df = flush_df
            df['FilteredPid'] = df['FlushMotorPID']
            df['RealtimePid'] = df['FlushRealtimePID']
            df['Vt'] = df['FlushFlow_x100']
            df['Va'] = df['FlushActualSpeed_x100']
        else:
            cal_motor = "CONTRAST"
            df = contrast_df
            df['FilteredPid'] = df['ContrastMotorPID']
            df['RealtimePid'] = df['ContrastRealtimePID']
            df['Vt'] = df['ContrastFlow_x100']
            df['Va'] = df['ContrastActualSpeed_x100']

        # remove acc point:
        #
        print("mean", df.Vt.mean())
        df = pd.DataFrame(df[df.Vt >= df.Vt.mean()])
        df = pd.DataFrame(df[df['Pressure(psi)'] >= min_pressure_in_psi])  # remove non-linearity at the bottom
        return df, cal_motor

    @staticmethod
    def analyse_and_plot(digest_csv_filename, min_pressure_in_psi, show_plot) -> str:
        da = DigestAnalysis()
        da.load(digest_csv_filename)
        da.df, cal_motor = da.filter_pressure_calibration_data(min_pressure_in_psi=min_pressure_in_psi)
        saved_csv_filename = da.plot_predict_pressure()
        if show_plot:
            plt.show()
        plt.clf()
        plt.close()
        return saved_csv_filename

    def plot_predict_pressure(self) -> str:
        """ return the string where the csv are saved. (not very nice but work)"""
        # using target speed to predict pressure
        dpi = 150
        name = os.path.splitext(os.path.basename(self.digest_csv_filename))[0]
        base_name = os.path.splitext(self.digest_csv_filename)[0]
        mdf = self.df  # todo
        full_csv_name = "%s-pressure.csv" % base_name
        full_png_name = "%s-pressure.png" % base_name
        mdf.to_csv(full_csv_name, index=False)
        print("output", full_csv_name)
        # noinspection PyTypeChecker
        fig, (ax0, ax1) = plt.subplots(
            figsize=[14, 8], nrows=1, ncols=2, clear=True, sharex=False, sharey=False, dpi=dpi)

        plot_a_b_time_series(ax0, mdf, 'FilteredPid', 'Pressure(psi)',
                             a_y_axis_name='FilteredPid',
                             b_y_axis_name='Pressure(psi)')

        x_min = mdf['FilteredPid'].min()
        x_max = mdf['FilteredPid'].max()
        vt = mdf['Vt'].mean() // 100
        # noinspection SpellCheckingInspection
        regressor, model_data, mae, mse, rmse = get_regression(mdf, "FilteredPid", "Pressure(psi)")

        x0 = np.arange(x_min, x_max, 0.5).reshape(-1, 1)
        yhat0 = regressor.predict(x0)
        equation = "$P_v=%.1f(x) = %.2f + %.2fx$" % (vt, model_data[0][0], model_data[1][0])

        mdf.plot.scatter(x="FilteredPid", y='Pressure(psi)', marker="+", ax=ax1, label="Pressure vs PID(A)")
        ax1.plot(x0, yhat0, label=equation)
        ax1.grid()
        ax1.legend()
        # noinspection SpellCheckingInspection
        ax1.set_title("Scatter: Pressure vs PID (mae=%.1f, mse=%.1f, rmse=%.1f)" % (mae, mse, rmse))
        ax0.set_title("Pressure and PID over time")
        fig.suptitle(name)
        plt.savefig(full_png_name)
        print("Generated", full_png_name)
        return full_csv_name


# noinspection DuplicatedCode
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Digest analysis')
    parser.add_argument('digest_csv_name_or_dir', type=str,
                        help='The digest csv filename or a directory contains all the file')
    parser.add_argument('--min_pressure_in_psi', type=int, default=20, help='The minimum pressure to plot in psi')
    parser.add_argument('--show_plot', type=bool, action=argparse.BooleanOptionalAction, default=False,
                        help='Show plot')
    parser.add_argument('--combine_pressure_data', type=bool, action=argparse.BooleanOptionalAction, default=False,
                        help='Combine pressure_data give the digest_csv_name_or_dir is a directory')
    args = parser.parse_args()
    if os.path.isfile(args.digest_csv_name_or_dir):
        DigestAnalysis.analyse_and_plot(args.digest_csv_name_or_dir, args.min_pressure_in_psi, args.show_plot)
    elif os.path.isdir(args.digest_csv_name_or_dir):
        if not args.combine_pressure_data:
            # generate pressure data and p lot
            flush_file_list = list(glob.glob(os.path.join(args.digest_csv_name_or_dir, "FLUSH-*run-*-pressure.csv")))
            contrast_file_list = list(
                glob.glob(os.path.join(args.digest_csv_name_or_dir, "CONTRAST-*run-*-pressure.csv")))
            for digest_csv_filename in flush_file_list + contrast_file_list:
                print(digest_csv_filename)
                DigestAnalysis.analyse_and_plot(digest_csv_filename, args.min_pressure_in_psi, args.show_plot)
        else:  # combine pressure data
            dpi = 120
            fig, axes = plt.subplots(
                figsize=[14, 8], nrows=1, ncols=2, clear=True, sharex=False, sharey=True, dpi=dpi)

            for idx, motor_str in enumerate(["CONTRAST", "FLUSH"]):
                # combine all the single and double-digit
                file_list = \
                    list(glob.glob(os.path.join(args.digest_csv_name_or_dir, "%s-*run-*-pressure.csv" % motor_str)))

                dfs = list()
                for csv_filename in file_list:
                    df = pd.read_csv(csv_filename)
                    print("load", csv_filename, df.shape)
                    dfs.append(df)
                df = pd.concat(dfs)

                # Make Vt ml/s
                df["Vt"] = df.Vt // 100

                # save it
                combined_name = os.path.join(args.digest_csv_name_or_dir, motor_str + "-combined-pressure.csv")
                df.to_csv(combined_name, index=False)
                print("Generated", combined_name, df.shape)

                # Plot it
                print("Unique Vt", df.Vt.unique())
                df.plot.scatter("FilteredPid", 'Pressure(psi)', c="Vt", marker=".", cmap="viridis", ax=axes[idx])
                axes[idx].grid()
                axes[idx].set_title(motor_str)

            png = os.path.join(args.digest_csv_name_or_dir, "combine-pressure.png")
            fig.suptitle(os.path.basename(args.digest_csv_name_or_dir))
            plt.savefig(png)
            plt.show()
            plt.clf()
            plt.close()


if __name__ == '__main__':
    main()
