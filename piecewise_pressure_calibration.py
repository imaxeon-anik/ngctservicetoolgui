import json

import pandas as pd
import os
import glob
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

from pid_analysis import get_regression
PSI_TO_KPA_SCALAR = 6.894757  # https://en.wikipedia.org/wiki/Pound_per_square_inch
dpi = 150
scale_1616 = 1 << 16
scale_248 = 1 << 8


# noinspection DuplicatedCode
class PiecewisePressureModel:
    """
        Given PID space as x, pressure as P
        Each integer speed 0 to 10 inclusive has a line of best fit P(x) = ax + b
            L[V] is map from an integer speed V to the corresponding line
        Given a speed v between 0 and 10ml/s, it can be in between 2 lines at speed floor(v) and floor(v)+1

        """

    def __init__(self, pid_at_10mlps=3100):
        # intercept, slope = -707.85, -1.21
        # self._lines = [(intercept, slope)] * 11  # include 0ml/s and 10ml/s
        self._is_psi_unit = True
        self._lines = [
            (-100.81257907281679, -1.2921468146614765),
            (-402.0465141363584, -1.2475644797934058),
            (-703.2804491999001, -1.2029821449253353),
            (-1004.5143842634419, -1.158399810057265),
            (-1350.6549145776048, -1.169907735460198),
            (-1595.5377783400245, -1.101311603855882),
            (-1841.9669395539260, -1.0721231523911094),
            (-2057.1934317952628, -1.0183720737647266),
            (-2252.0232694174074, -0.9646925756108021),
            (-2389.4086787742660, -0.9197862753570982),
            (-2480.5483956149310, -0.8496249229840678)]
        self.zero_loads_pids = self.generate_zero_load_pids(pid_at_10mlps)

    def get_zero_load_pid(self, speed_x100: int) -> int:
        if speed_x100 < 0:
            speed_x100 = -speed_x100

        if speed_x100 >= 100:
            idx = 9
            dv = speed_x100 - 900
        else:
            idx = speed_x100 // 100
            dv = speed_x100 % 100
        dp = self.zero_loads_pids[idx + 1] - self.zero_loads_pids[idx]
        return self.zero_loads_pids[idx] + dp * dv / 100

    @staticmethod
    def generate_zero_load_pids(pid_at_10mlps):
        pids = list(range(0, pid_at_10mlps+1, pid_at_10mlps//10))
        pids[-1] = pid_at_10mlps  # ensure the last one end with it
        pids.append(pid_at_10mlps//10)  # 11 items
        return pids

    def __str__(self) -> str:
        strs = [str(x) for x in self._lines]
        return ",\n".join(strs)

    def save_model(self, text_filename):
        import json
        fh = open(text_filename, "w")
        model = dict()
        model["is_psi"] = self._is_psi_unit
        model["lines"] = self._lines
        json.dump(model, fh, indent=1)

    @staticmethod
    def load_model_from_json(json_filename):
        with open(json_filename) as fh:
            data = json.load(fh)
            ret = PiecewisePressureModel()
            ret._is_psi_unit = data["is_psi"]
            for speed, (ix, mx) in enumerate(data["lines"]):
                ret.set_model_at_speed(speed, ix, mx)
            return ret

    def to_kpa_unit(self):
        if self._is_psi_unit:
            self._is_psi_unit = False
            self._lines = [(ix * PSI_TO_KPA_SCALAR, mx * PSI_TO_KPA_SCALAR) for ix, mx in self._lines]

    @staticmethod
    def float_to_int(float_val: float):
        return int(np.floor(float_val + 0.5))

    def to_mcu_command(self, motor_str: str) -> str:
        scale_1616 = 1 << 16
        # 24.8 offset and 16.16 slope
        params_string = ",".join([" %d,%d" % (self.float_to_int(ix * scale_248), self.float_to_int(mx * scale_1616)) for ix, mx in self._lines])
        return "SET_PRESSURE_COEFFICIENTS %s %s" % (motor_str, params_string)

    def adjust_pressure_offset(self, speed_int: int, pressure_offset_in_psi, extrapolate_to_all_speed=True):
        if speed_int > 10 or speed_int < 1:
            raise Exception("invalid speed value. expect 1 to 10")
        if extrapolate_to_all_speed:
            slope = pressure_offset_in_psi / speed_int
            for i in range(len(self._lines)):
                ix, mx = self._lines[i]
                self._lines[i] = ix + slope * i, mx
                print(i, "adjust offset", ix, mx, " -> ", self._lines[i])
        else:
            ix, mx = self._lines[speed_int]
            self._lines[speed_int] = ix + pressure_offset_in_psi, mx
            print(speed_int, "adjust offset", ix, mx, " -> ", self._lines[speed_int])

    def to_mcu_c_struct(self, motor_str) -> str:
        params_string = "\n    ".join(
            ["%6d,%10d, // intercept_%04d(24.8), slope_%02d(16.16) = %f, %f" % (self.float_to_int(ix * scale_248), self.float_to_int(mx * scale_1616), idx, idx, ix, mx) for idx, (ix, mx) in
             enumerate(self._lines)])
        result = "\n//Pressure unit %s\n" % ("psi" if self._is_psi_unit else "kpa")
        result += "const Fixed_1616 pressure_default_%s[(MAX_INT_SPEED_MLPS + 1) * 2] = {\n    %s\n};\n" % (motor_str.lower(), params_string)
        return result

    def set_model_at_speed(self, speed, intercept, slope) -> bool:
        if speed < 0 or speed > 10:
            return False
        speed = int(speed)
        self._lines[speed] = intercept, slope
        return True

    def calculate_pressure_in_psi(self, speed_x100, pid):
        if speed_x100 < 0:
            speed_x100 = -speed_x100
        idx = speed_x100 // 100
        dv = speed_x100 % 100
        if idx >= 10:
            idx = 9
            dv += 100  # adding extra interpolate factor

        l1 = self._lines[idx]
        l2 = self._lines[idx + 1]

        p1 = l1[0] + l1[1] * pid
        p2 = l2[0] + l2[1] * pid

        v2_v1 = -100  # Vt[idx] - Vt[idx + 1]
        pressure = p1 + dv * (p1 - p2) / v2_v1
        return pressure

    def calculate_pressure_at_speed_index_in_psi(self, speed_x100, pid, ref_from: int, ref_to: int):
        if speed_x100 < 0 or speed_x100 > 1000:
            raise Exception("speed_index must be between 0 and 1000 inclusive")
        if ref_from < 0 or ref_from > 9:
            raise Exception("from-model must be between 0 and 9 inclusive")
        if ref_to < 1 or ref_to > 10:
            raise Exception("to-model must be between 1 and 10 inclusive")
        if ref_from >= ref_to:
            raise Exception("Reference model must be different and from-model must be less than to-model")

        l1 = self._lines[int(ref_from)]
        l2 = self._lines[int(ref_to)]

        dv = speed_x100 - ref_from * 100

        p1 = l1[0] + l1[1] * pid
        p2 = l2[0] + l2[1] * pid
        v2_v1 = (ref_from - ref_to) * 100
        pressure = p1 + dv * (p1 - p2) / v2_v1
        return pressure

    @staticmethod
    def create_model_from_df(df: pd.DataFrame):
        speeds = df['Vt'].unique()
        print("the available speed are", speeds)
        regression_dict = dict()
        for vt in speeds:
            mdf = df[df['Vt'] == vt]
            regressor, model_data, mae, mse, rmse = get_regression(mdf, "FilteredPid", "Pressure(psi)", verbose=False)
            # print("speed", Vt, "error", mae, mse, rmse, "Model", model_data)
            row = vt, mae, mse, rmse, model_data[0][0], model_data[1][0]
            regression_dict[vt] = row

        ppm = PiecewisePressureModel()
        data = sorted(regression_dict.values(), key=lambda a: a[0])
        if len(data) <= 1:
            raise Exception("Not enough calibration data for interpolating")

        for row in data:
            vt, mae, mse, rmse, intercept, slope = row
            ppm.set_model_at_speed(vt, intercept, slope)
            print(row)

        # Filling up missing data:
        missing_vts = list()
        for vt in range(11):
            if vt not in regression_dict:
                missing_vts.append(vt)
                first_vt = data[0][0]
                next_vt = data[1][0]
                if first_vt > vt:
                    ppm.generate_missing_speed(first_vt, next_vt, vt)
                else:
                    start_idx = 0
                    for idx, row in enumerate(data):
                        if row[0] <= vt:  # current speed less than the want speed
                            start_idx = idx
                        else:
                            break
                    if start_idx == len(data) - 1:    # last one, back trace 1
                        start_idx -= 1
                    if start_idx < 0:
                        start_idx = 0
                    first_vt = data[start_idx][0]
                    next_vt = data[start_idx + 1][0]
                    ppm.generate_missing_speed(first_vt, next_vt, vt)

        print("Missing Vt", missing_vts)

        err_df = pd.DataFrame(columns=["Vt", "mae", "mse", "rmse", "intercept", "slope"], data=data)
        return ppm, err_df

    def generate_missing_speed(self, first_vt: int, next_vt: int, vt: int):
        pid1 = -800
        pid2 = -300
        p1 = self.calculate_pressure_at_speed_index_in_psi(vt * 100, pid1, first_vt, next_vt)
        p2 = self.calculate_pressure_at_speed_index_in_psi(vt * 100, pid2, first_vt, next_vt)
        delta_pid = pid2 - pid1
        delta_p = p2 - p1
        slope = delta_p / delta_pid
        intercept = p1 + slope * (-pid1)
        print("Generate missing data for", vt, "line:", intercept, slope)
        self.set_model_at_speed(vt, intercept, slope)

    def compute_pressure_for_df(self, df: pd.DataFrame):
        vas = df["Va"].to_numpy()
        pids = df["FilteredPid"].to_numpy()
        predicted = [
            self.calculate_pressure_in_psi(va, pid) for va, pid in zip(vas, pids)
        ]
        df["y_hat"] = predicted
        df["err"] = df["y_hat"] - df['Pressure(psi)']

    @staticmethod
    def load_calibration_data(digest_csv_filename) -> pd.DataFrame:
        """ reading CONTRAST-combined-pressure.csv, or FLUSH-combined-pressure.csv"""
        df = pd.read_csv(digest_csv_filename)
        print("Loading", digest_csv_filename, df.shape)
        # print("fields", self.df.columns)
        df['Pressure(psi)'] = df.PressureADC / 4095.0 * (500.00 + 14.7) - 14.7  # sensor conversion 0-5V = -14-500psi
        df['Pressure(kpa)'] = PSI_TO_KPA_SCALAR * df['Pressure(psi)']
        df['T(s)'] = df['T(ms)'] / 1000
        df['T'] = df['T(s)']  # for plotting

        # use realtime pid filtering
        df['FilteredPid'] = df['RealtimePid']

        test_filter = False
        if test_filter:
            # 'FilteredPid', 'RealtimePid'
            df.plot.scatter('FilteredPid', 'RealtimePid')
            plt.title(digest_csv_filename)
            plt.show()
            plt.clf()
            plt.close()
            plt.title(digest_csv_filename)
            df["err"] = df['FilteredPid'] - df['RealtimePid']
            df.plot.scatter("Va", 'err')
            plt.show()
            plt.clf()
            plt.close()

        return df

    @staticmethod
    def adjust_zero_pressure_pid(df: pd.DataFrame, pid_at_10mlps: int) -> pd.DataFrame:
        """Assume linear model"""
        # compute zero-load pid
        temp = PiecewisePressureModel(pid_at_10mlps)
        speeds = df["Va"].to_numpy()
        zero_load_pids = [
            temp.get_zero_load_pid(v) for v in speeds
        ]

        # take away from the regression
        df["OrgFilteredPid"] = df["FilteredPid"]
        df['ZeroLoadPid'] = zero_load_pids
        # df["FilteredPid"] = df["FilteredPid"] - df['ZeroLoadPid']
        df["DeltaFilteredPid"] = df["FilteredPid"] - df['ZeroLoadPid']
        return df

    @staticmethod
    def create_model_from_calibrated_csv(csv_filename, base_speed=0, base_pressure=0):
        print("Processing", csv_filename)
        base_name = os.path.splitext(os.path.basename(csv_filename))[0]
        # pressure = model.calculate_pressure_in_psi(200, -625)
        df = PiecewisePressureModel.load_calibration_data(csv_filename)
        print("df", df.shape, df.columns)

        # WIP todo check
        # df = PiecewisePressureModel.adjust_zero_pressure_pid(df, pid_at_10mlps=-3150)
        model, err_df = PiecewisePressureModel.create_model_from_df(df)

        if base_speed > 0:
            model.adjust_pressure_offset(base_speed, base_pressure, extrapolate_to_all_speed=True)

        print("Model", str(model))
        print("error.df", err_df)

        model.compute_pressure_for_df(df)
        # box plots and model and pressure combine
        # noinspection DuplicatedCode,PyTypeChecker
        fig, (ax, ax1) = plt.subplots(figsize=[14, 8], nrows=1, ncols=2, clear=True, sharex=False, sharey=False, dpi=dpi)
        cmap = plt.cm.rainbow
        by_col = df['Vt'].unique()
        by_min = min(by_col)
        by_max = max(by_col)
        # print(by_col, by_min, by_max)
        norm = colors.BoundaryNorm(np.arange(by_min, by_max + 1, 1), cmap.N)
        df.plot.scatter("FilteredPid", 'Pressure(psi)', cmap=cmap, norm=norm, c="Vt", marker=".", ax=ax)
        model.plot_model_predicted_pressure(ax, max_pressure=400, min_pid=-4095, show_hlines=True)
        # df.plot.scatter("FilteredPid", 'ZeroLoadPid')

        df.boxplot('err', by='Vt', ax=ax1)
        # ax.set_title(str(model))
        fig.suptitle(base_name)
        ax.set_title(base_name)
        ax1.set_title("Pressure error (psi)")
        # ax.grid()
        motor_str = base_name.split("-")[0]  # expect "CONTRAST" or "FLUSH

        base = os.path.splitext(csv_filename)[0]
        png = base + "-predicted.png"
        json_filename = base + "-model.json"
        mcu_filename = base + "-model-mcu.txt"
        error_filename = base + "-predicted-error.csv"
        plt.savefig(png)
        plt.show()
        plt.clf()
        plt.close()
        print("Generated", png)
        model.save_model(json_filename)
        print("Generated", json_filename)
        err_df.to_csv(error_filename, index=False)
        print("Generated", error_filename)

        print("Convert the model to kpa unit")
        model.to_kpa_unit()         # converting the model to kpa unit

        with open(mcu_filename, "w") as fh:
            fh.write("// " + csv_filename + "\n")
            fh.write("// " + model.to_mcu_command(motor_str))
            fh.write(model.to_mcu_c_struct(motor_str))

            print("Generate", mcu_filename)
        print("MCU model:", model.to_mcu_command(motor_str))

        # sample:
        for speed, pid in ([200, -600], [200, -700], [250, -700],
                           [1000, -3000], [1000, -3100], [950, -3100],
                           [499, -1503], [500, -1503],
                           [300, -916]):
            pressure = model.calculate_pressure_in_psi(speed, pid)
            print("v, pid, p", speed, pid, pressure)

    # noinspection SpellCheckingInspection
    def plot_model_predicted_pressure(self, ax,
                                      max_pressure=300, min_pid=-3500, min_pressure_psi=0,
                                      is_dashing=False, show_hlines=False) -> dict:
        """
        @returns a dictionary of color index by speed.
        """
        # generate predicted data that the output pressures in [0, 300 psi] for different speeds
        pids = np.arange(0, min_pid, -10)
        speed_color_dict = dict()
        for speed in np.arange(1, 11, 1) * 100:
            yh = [self.calculate_pressure_in_psi(speed, pid) for pid in pids]
            yh = np.array(yh)
            # noinspection DuplicatedCode
            first_max_index = (yh > max_pressure).argmax(axis=0)  # index of the first value greate than max pressure
            filtered_yh = yh[:first_max_index]
            filtered_pids = pids[:first_max_index]
            indices = np.bitwise_and(filtered_yh > min_pressure_psi, filtered_yh <= max_pressure)
            valid_pids = filtered_pids[indices]
            valid_yh = filtered_yh[indices]
            if len(valid_yh):
                if is_dashing:
                    ax.plot(valid_pids, valid_yh, dashes=[6, 4], label="$P_h(V=%d)$" % speed)
                else:
                    ax.plot(valid_pids, valid_yh, label="$P_h(V=%d)$" % speed, color="gray")
                last_color = ax.get_lines()[-1].get_color()
                speed_color_dict[speed] = last_color
        if show_hlines:
            ax.axhline(300, dashes=[2, 6])
            ax.axhline(400, dashes=[2, 6], c='red')
        ax.set_ylabel("Pressure(psi)")
        ax.set_xlabel("pid")
        ax.grid()
        return speed_color_dict


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Piecewise pressure calibration')
    parser.add_argument('digest_csv_name_or_dir', type=str, help='The digest csv filename or a directory')
    # noinspection DuplicatedCode
    parser.add_argument('--min_pressure_in_psi', type=int, default=20, help='The minimum pressure to plot in psi')
    parser.add_argument('--base_speed', type=int, default=0, help='The speed ml/s to adjust the intercept term, only trigger if the speed > 0')
    parser.add_argument('--base_pressure', type=int, default=0, help='The pressure at the base speed to adjust all the intercept term')

    parser.add_argument('--show_plot', type=bool, action=argparse.BooleanOptionalAction, default=False, help='Show plot')
    parser.add_argument('--combine_pressure_data', type=bool, action=argparse.BooleanOptionalAction, default=False,
                        help='Combine pressure_data give the digest_csv_name_or_dir is a directory')
    args = parser.parse_args()
    if os.path.isfile(args.digest_csv_name_or_dir):
        PiecewisePressureModel.create_model_from_calibrated_csv(args.digest_csv_name_or_dir, args.base_speed, args.base_pressure)
    elif os.path.isdir(args.digest_csv_name_or_dir):
        flush_file_list = list(glob.glob(os.path.join(args.digest_csv_name_or_dir, "CONTRAST-combined-pressure.csv")))
        contrast_file_list = list(glob.glob(os.path.join(args.digest_csv_name_or_dir, "FLUSH-combined-pressure.csv")))
        for digest_csv_filename in flush_file_list + contrast_file_list:
            PiecewisePressureModel.create_model_from_calibrated_csv(digest_csv_filename, args.base_speed, args.base_pressure)


def test_it():
    model = PiecewisePressureModel()
    x = list(range(1005))
    y = [model.get_zero_load_pid(a) for a in x]
    plt.plot(x, y)
    plt.show()


if __name__ == '__main__':
    main()
    # test_it()
