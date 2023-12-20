import time

from ServiceMotorController import MotorController, get_datetime_string
import os
import traceback
from DigestAnalysis import DigestAnalysis
from piecewise_pressure_calibration import PiecewisePressureModel
import pandas as pd

FLUSH = 0
CONTRAST = 1
ALL = 2

class PressureCalibration:
    def __init__(self, mcu: MotorController,
                 start_pressure_x10:int,
                 max_pressure_psi_x10: int,
                 ramp_pressure_psi_x10: int,
                 min_pressure_to_analyse_in_psi=50,
                 verbose=True):
        self.mcu = mcu
        self.verbose = verbose
        self.start_pressure_psi_x10 = start_pressure_x10
        self.max_pressure_psi_x10 = max_pressure_psi_x10
        self.ramp_pressure_psi_x10 = ramp_pressure_psi_x10
        self.min_calibration_volume_needed_x10 = 110
        self.calibrate_motor = "FLUSH"
        self.start_pressure_adc = 0.0
        self.max_fill_vol_ml_x10 = 1900
        self.current_speed_mlps_x10 = 0
        self.min_pressure_to_analyse_in_psi = min_pressure_to_analyse_in_psi
        self.motor_cal_before_calibration = False
        self.result_dict = dict()
        self.result_dict["FLUSH"] = []
        self.result_dict["CONTRAST"] = []

    def do_pressure_ramp(self, motor_index, speed_x10: int):
        self.mcu.update_digest()
        if self.mcu.motor_is_active(FLUSH) or self.mcu.motor_is_active(CONTRAST):
            print("ERROR: Fail to start either motors are active")
            return False

        if not self.mcu.motor_is_engaged(FLUSH):
            self.mcu.ensure_plunger_engaged(FLUSH)

        if not self.mcu.motor_is_engaged(CONTRAST):
            self.mcu.ensure_plunger_engaged(CONTRAST)

        if self.mcu.get_pressure_adc_in_psi() * 10 > self.start_pressure_psi_x10:
            print("ERROR initial condition over-pressure ADC pressure %.01f initial cal pressure %.01f" %
                  (self.mcu.get_pressure_adc_in_psi(), self.start_pressure_psi_x10/10.0))
            return False

        #Work out which motor needs to be calibrated first
        flush_vol_x10 = self.mcu.get_syringe_volume(FLUSH)
        contrast_vol_x10 = self.mcu.get_syringe_volume(CONTRAST)
        total_vol_x10 = flush_vol_x10 + contrast_vol_x10
        if total_vol_x10 > 2000:
            print("ERROR total volume exceeds 200ml")
            return False
        if total_vol_x10 < self.min_calibration_volume_needed_x10:
            print("ERROR total volume less than %d .01ml" % self.min_calibration_volume_needed_x10,
                  total_vol_x10 / 10, contrast_vol_x10, flush_vol_x10)
            return False
        self.max_fill_vol_ml_x10 = total_vol_x10
        print("Set max_fill_vol_ml_x10 to", self.max_fill_vol_ml_x10)

        self.calibrate_motor = motor_index

        self_current_speed_mlps_x10 = abs(speed_x10)
        self.start_pressure_adc = self.mcu.get_pressure_adc_in_psi()

        if self.motor_cal_before_calibration:
            self.mcu.find_plunger(2, 100)
            self.mcu.wait_until()

        flush_vol_x10 = self.mcu.get_syringe_volume(FLUSH)
        contrast_vol_x10 = self.mcu.get_syringe_volume(CONTRAST)

        self.mcu.pressure_ready(ALL, 100)
        self.mcu.wait_until()

        flush_vol_x10 = self.mcu.get_syringe_volume(FLUSH)
        contrast_vol_x10 = self.mcu.get_syringe_volume(CONTRAST)
        if total_vol_x10 > 2000:
            print("ERROR total volume exceeds 200ml")
            return False
        if total_vol_x10 < self.min_calibration_volume_needed_x10:
            print("ERROR total volume less than %d .01ml" % self.min_calibration_volume_needed_x10,
                  total_vol_x10 / 10, contrast_vol_x10, flush_vol_x10)
            return False

        pull_motor = CONTRAST if flush_vol_x10 > contrast_vol_x10 else FLUSH
        push_speed_x10 = self.current_speed_mlps_x10

        if push_speed_x10 < 40:
            ramp_dv_x10 = 220  # 22ml
        elif push_speed_x10 < 70:
            ramp_dv_x10 = 200  # 20ml
        else:
            ramp_dv_x10 = 180  # 18ml

        push_vol_x10 = flush_vol_x10 if flush_vol_x10 > contrast_vol_x10 else contrast_vol_x10
        total_time = push_vol_x10 / push_speed_x10
        speed_delta_x10 = ramp_dv_x10 // total_time  # integer division
        if speed_delta_x10 < 1:
            speed_delta_x10 = 1  # 0.1ml
        print("delta speed", speed_delta_x10, push_vol_x10, push_speed_x10)

        ramp_speed_x10 = push_speed_x10 - speed_delta_x10

        short_name = "%s-vt-%03d.csv" % (self.calibrate_motor, push_speed_x10)
        print(short_name)
        self.mcu.set_digest_csv_log(self.mcu.get_absolute_path(short_name))
        self.mcu.update_digest()
        time.sleep(0.5)

        stable_time_in_ms = 5000

        self.mcu.pull_pressure_ramp(pull_motor, -ramp_speed_x10, self.ramp_pressure_psi_x10,
                                    self.start_pressure_psi_x10, self.max_pressure_psi_x10, -push_speed_x10,
                                    stable_time_in_ms)
        self.mcu.wait_until(delay=0.001)
        digest_csv_filename = self.mcu.set_digest_csv_log("")
        print("Finished data capture. Analysing log", digest_csv_filename)
        if os.path.exists(digest_csv_filename):
            # analyse and plot digest pressure data
            csv_filename = DigestAnalysis.analyse_and_plot(digest_csv_filename,
                                                           min_pressure_in_psi=self.min_pressure_to_analyse_in_psi,
                                                           show_plot=False)
            # keep a list of csv filename
            self.result_dict[self.calibrate_motor].append(csv_filename)

    def disengage_all(self):
        flush_vol_x10 = self.mcu.get_syringe_volume(FLUSH)
        contrast_vol_x10 = self.mcu.get_syringe_volume(CONTRAST)
        if flush_vol_x10 < contrast_vol_x10:
            first_disengage = FLUSH
            second_disengage = CONTRAST
        else:
            first_disengage = CONTRAST
            second_disengage = FLUSH

        self.mcu.disengage(first_disengage)
        self.mcu.wait_until()
        self.mcu.disengage(second_disengage)
        self.mcu.wait_until()

        self.mcu.disengage(ALL)
        self.mcu.wait_until()

    def combine_pressure_data(self) -> list:
        output = list()
        for motor_str, filelist in self.result_dict.items():
            if len(filelist) == 0:
                continue
            print("Motor", motor_str, len(filelist))
            dfs = list()
            for csv_filename in filelist:
                df = pd.read_csv(csv_filename)
                print("    loaded", csv_filename, df.shape)
                dfs.append(df)
            df = pd.concat(dfs)

            # Make Vt ml/s  -- todo why it was done with "df.Vt // 100"?
            df["Vt"] = df.Vt / 100

            # save it
            combined_name = self.mcu.create_filename(motor_str + "-combined-pressure.csv")
            df.to_csv(combined_name, index=False)
            print("Generated", combined_name, df.shape)
            output.append(combined_name)
        return output

    @staticmethod
    def create_model_from_calibrated_csv(combined_filename: str):
        PiecewisePressureModel.create_model_from_calibrated_csv(combined_filename, 0, 0)

    @staticmethod
    # noinspection DuplicatedCode
    def main(main_com: str,
             output_dir: str,
             start_mlps_x10=10,
             end_mlps_x10=100,
             step_mlps_x10=30,
             start_pressure_x10=100,
             max_pressure_psi_x10=2500,
             ramp_pressure_psi_x10=2500,
             rest_time_in_seconds=60
             ):

        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        name = "Pressure-" + get_datetime_string()
        output_dir = os.path.join(output_dir, name)
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        mcu = MotorController(main_com, "", output_dir)

        def test_script():
            # test digest
            mcu.set_digest_csv_log(mcu.get_absolute_path("mcu-digest-test.csv"))
            for i in range(10):
                mcu.update_digest()
            mcu.close()
            return

        mcu.update_digest()
        # no need to start a thread
        # mcu.start_monitor_reading_thread(None)    # this is needed to read the data - no thread

        pc = PressureCalibration(mcu,
                                 start_pressure_x10,
                                 max_pressure_psi_x10,
                                 ramp_pressure_psi_x10,
                                 min_pressure_to_analyse_in_psi=50,
                                 verbose=True)

        mcu.find_plunger(2)
        mcu.wait_until()

        if not mcu.ensure_plunger_engaged(CONTRAST):
            print("ERROR: failed to engage the CONTRAST plunger")
            mcu.close()
            return

        if not mcu.ensure_plunger_engaged(FLUSH):
            print("ERROR: failed to engage FLUSH plunger")
            mcu.close()
            return

        if not mcu.motor_is_engaged(CONTRAST):
            print("ERROR: abort due to missing contrast plunger")
            mcu.close()
            return

        if not mcu.motor_is_engaged(FLUSH):
            print("ERROR:abort due to missing flush plunger")
            mcu.close()
            return

        flush_vol_x10 = mcu.get_syringe_volume(FLUSH)
        contrast_vol_x10 = mcu.get_syringe_volume(CONTRAST)
        if flush_vol_x10 < contrast_vol_x10:
            calibrated_motors = ["FLUSH", "CONTRAST"]
        else:
            calibrated_motors = ["CONTRAST", "FLUSH"]

        try:
            for speed_x10 in range(start_mlps_x10, end_mlps_x10+1, step_mlps_x10):
                for motor in calibrated_motors:
                    pc.do_pressure_ramp(motor, speed_x10)
                print("Let the motor rest (%ds)" % rest_time_in_seconds)
                import tqdm
                for _ in tqdm.trange(rest_time_in_seconds):
                    time.sleep(1)

        except Exception as e:
            print("ERROR", e)
            traceback.print_exc()   # printing stack trace
        mcu.close()

        # combine the pressure data in to a single file for a motor
        combined_names = pc.combine_pressure_data()

        # then create the pressure model
        for filename in combined_names:
            pc.create_model_from_calibrated_csv(filename)



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='NextGenCT Pressure Calibration')
    parser.add_argument('main_com', type=str, help='The main USB serial COM port')
    # noinspection DuplicatedCode
    parser.add_argument('--start_mlps_x10', type=int, default=20, help='First flow rate in x10 ml/s')
    parser.add_argument('--end_mlps_x10', type=int, default=100, help='Final flow rate in x10 ml/s')
    parser.add_argument('--step_mlps_x10', type=int, default=20, help='Step between flow rate x10 ml/s')
    parser.add_argument('--start_pressure_x10', type=int, default=100, help='Initial pressure before the 2nd motor move  is x10 psi unit')
    parser.add_argument('--max_pressure_psi_x10', type=int, default=2500, help='Maximum Pressure is x10 psi unit')
    parser.add_argument('--ramp_pressure_psi_x10', type=int, default=2500, help='Pressure is x10 psi unit')
    parser.add_argument('--rest_time_in_seconds', type=int, default=60, help='Let motors rest between flow-rate in seconds')

    parser.add_argument('--output_dir', type=str, default="./output", help='The output directory')
    args = parser.parse_args()
    PressureCalibration.main(
        args.main_com,
        args.output_dir,
        start_mlps_x10=args.start_mlps_x10,
        end_mlps_x10=args.end_mlps_x10,
        step_mlps_x10=args.step_mlps_x10,
        start_pressure_x10=args.start_pressure_x10,
        max_pressure_psi_x10=args.max_pressure_psi_x10,
        ramp_pressure_psi_x10=args.ramp_pressure_psi_x10,
        rest_time_in_seconds=args.rest_time_in_seconds
    )
