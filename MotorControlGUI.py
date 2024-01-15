import os
import time
import tkinter as tk
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
from threading import Thread
import csv

from ServiceMotorController import MotorController


class DigestThread(Thread):
    def __init__(self, mcu):
        Thread.__init__(self)
        self.mcu = mcu
        self.daemon = True
        self._stop = False
        self.start()

    def run(self):
        while True:
            # self.mcu.loop_digest(delay=0.1)
            time.sleep(0.1)
            if self._stop:
                self.join()
                return
            self.mcu.do_digest()

    def set_stop(self):
        self._stop = True


class MotorControllerApplication(tk.Frame):
    def __init__(self, master, mcu: MotorController, out_dir, verbose=False):
        super().__init__(master)
        self.master = master
        self.grid(row=0, column=0, sticky='NESW')
        self._verbose = verbose

        self._command_var = tk.StringVar(self)

        self._timer_counter = 0
        self._used_debug_windows = False  # True to create a separate debug windows
        # self.pack()
        self._command_var = tk.StringVar(self)
        self._volume_var = tk.DoubleVar(self, value=50)
        self._speed_var = tk.DoubleVar(self, value=8)
        self._debug_M1 = tk.BooleanVar(self, value=False)
        self._debug_M2 = tk.BooleanVar(self, value=False)

        self.digest_inject_progress_var = tk.StringVar(self, value="")
        self.digest_inject_complete_var = tk.StringVar(self, value="")
        self.digest_inject_pressure_var = tk.StringVar(self, value="I.P")
        self.digest_flush_pressure_var = tk.StringVar(self, value="S.P")
        self.digest_contrast_pressure_var = tk.StringVar(self, value="C.P")

        self.digest_flush_status_var = tk.StringVar(self, value="status")
        self.digest_flush_volume_var = tk.StringVar(self, value="volume")
        self.digest_flush_flow_var = tk.StringVar(self, value="flow")
        self.digest_flush_current_var = tk.StringVar(self, value="current")
        self.digest_flush_plunger_var = tk.StringVar(self, value="plunger")
        self.digest_flush_plunger_adc_var = tk.StringVar(self, value="plungerADC")
        self.digest_flush_actual_speed_var = tk.StringVar(self, value="Actual.Flow")

        self.digest_contrast_status_var = tk.StringVar(self, value="status")
        self.digest_contrast_volume_var = tk.StringVar(self, value="volume")
        self.digest_contrast_flow_var = tk.StringVar(self, value="flow")
        self.digest_contrast_current_var = tk.StringVar(self, value="current")
        self.digest_contrast_plunger_var = tk.StringVar(self, value="plunger")
        self.digest_contrast_plunger_adc_var = tk.StringVar(self, value="plungerADC")
        self.digest_contrast_actual_speed_var = tk.StringVar(self, value="Actual.Flow")

        self.digest_pressure_adc = tk.StringVar(self, value="pressureADC")
        self.digest_battery_level_var = tk.StringVar(self, value="battery")
        self.digest_power_source_var = tk.StringVar(self, value="power")
        self.digest_diagnostic_var = tk.StringVar(self, value="diagnostic")

        self.digest_flush_air_flow = tk.StringVar(self, value="Air.Flow")
        self.digest_contrast_air_flow = tk.StringVar(self, value="Air.Flow")

        self.digest_flush_air_vol = tk.StringVar(self, value="Air.Vol")
        self.digest_contrast_air_vol = tk.StringVar(self, value="Air.Vol")
        self.digest_active_alarms = tk.StringVar(self, value="Active.alarms")
        self.digest_active_hardware_signals = tk.StringVar(self, value="Active.hardware")

        self.mcu = mcu
        self.out_dir = out_dir
        if not os.path.exists(self.out_dir):
            os.mkdir(self.out_dir)

        self.start_time = self.mcu.unix_timestamp()

        # if len(log_filename) == 0:
        #     # create MCU with the link name and current system timestamp
        self.log_filename = self.mcu.get_filename_from_current_timestamp() + ".txt"
        with open(self.log_filename, mode='a') as csv_file:
            csv_file.write("flush_pressure,contrast_pressure,flush_adc,contrast_adc,"
                           "flush_speed,contrast_speed,time, "
                           "flush_actual_speed, contrast_actual_speed,"
                           "flush_volume, contrast_volume, "
                           "flush_flow, contrast_flow, "
                           "flush_current, contrast_current, "
                           ""
                           "\n")
        # else:
        #     log_filename = os.path.join(self._output_dir, log_filename)
        # self._log_file = open(log_filename, "w")
        # print("Set output directory:", self.out_dir)
        # print("MCU log filename", log_filename)

        # self._binary_queue = deque(maxlen=1000)

        self._create_widgets()
        # self.master.after(20, self._update_binary_queue_timer)
        # self.master.after(100, self._do_digest)

    def _on_quit(self):
        # self._on_cycle_test_stop()
        # self.mcu.stop_monitor_reading_thread()
        self.master.destroy()

    # def read_all_binary_queue_data(self):
    #     samples = []
    #     while len(self._binary_queue) > 0:
    #         data = self._binary_queue.popleft()
    #         samples.append(data)
    #     return samples

    def s_push_c_pull(self, motor, speed_gap=10, start_pressure=50, max_pressure=1000, delta_pressure=200,
                      stable_duration=1000):
        self.do_digest()

        if (self.mcu.last_digest_data.flush_syringe_state or
                self.mcu.last_digest_data.contrast_syringe_state):
            print("ERROR: Either motors are still busy")
            return False
        print(self.mcu.last_digest_data.contrast_plunger_state)
        if self.mcu.last_digest_data.contrast_plunger_state != 1:
            print("does not have a contrast plunger")
            return False

        if self.mcu.last_digest_data.flush_plunger_state != 1:
            print("does not have a flush plunger")
            return False

        if self.mcu.get_syringe_volume(motor) <= self.mcu.get_syringe_volume(motor % 2 + 1):
            print("ERROR: need push motor to have more volume than pull motor", self.mcu.get_syringe_volume(motor),
                  self.mcu.get_syringe_volume(motor % 2 + 1))
            return False
        push_speed = int(self._speed_var.get()*10)
        if push_speed < 10:
            print("ERROR: speed must be greater than 1ml/s")
            return False

        if push_speed <= speed_gap:
            print("WARNING adjust speed gap to be 0.1ml less than the push speed")

        down_speed = speed_gap - push_speed

        volume = self.mcu.get_syringe_volume(motor) / 10

        self.mcu.piston(motor % 2 + 1, volume, push_speed/10)
        time.sleep(0.1)
        self.mcu.pull_pressure_ramp(motor, down_speed, delta_pressure, start_pressure, max_pressure, down_speed,
                                    stable_duration)

    # def on_enable_debug(self, motor):
    #     enable = 0
    #     self.mcu.enable_debug(motor)
    #     if self.plot_frame:
    #         self.plot_frame.set_animation(enable)

    def _create_widgets(self):
        frame_width = 150
        frame_height = 50

        all_command_frame = tk.LabelFrame(self, width=frame_width, text="Commands:", height=frame_height)
        all_command_frame.grid(row=0, column=0, padx=10, pady=2, sticky='EW')

        left_frame = tk.LabelFrame(all_command_frame, width=frame_width, text="Motor Command:", height=frame_height)
        left_frame.grid(row=0, column=0, padx=10, pady=2, sticky='EW')

        # row 0
        right_frame = tk.LabelFrame(all_command_frame, width=frame_width, text="Custom Commands", height=frame_height)
        right_frame.grid(row=0, column=1, padx=10, pady=2, sticky='EW')


        # tk.Label(top_frame, text="Command:").grid(row=0, column=0)
        # self._command_entry = tk.Entry(right_frame, textvariable=self._command_var, width=50)
        # self._command_entry.grid(row=0, column=0, columnspan=6)

        tk.Button(right_frame, text="️S.Push,C.Pull",
                  command=lambda: self.s_push_c_pull(0, 200, 50, 1000)).grid(row=2, column=0)
        tk.Button(right_frame, text="C.Push,S.Pull",
                  command=lambda: self.s_push_c_pull(1, 200, 50, 1000)).grid(row=2, column=1)

        tk.Button(
            right_frame, text="Arm(6 phases)",
            command=lambda: self.mcu.inject_arm(500, 6, 40, 50,
                                                50, 200, 100,
                                                50, 200, 0, 50,
                                                200, 5, 50, 200,
                                                95, 50, 200, 50,
                                                50, 200, 0)
        ).grid(row=3, column=0)
        #
        tk.Button(
            right_frame, text="Plot data",
            command=lambda: self.injector_plot()
        ).grid(row=3,column=6)

        tk.Button(
            right_frame, text="Inject Start",
            command=lambda: self.mcu.inject_start()
        ).grid(row=3, column=1)
        #
        tk.Button(
            right_frame, text="Digest",
            command=self._on_digest
        ).grid(row=3, column=2)
        #
        tk.Button(
            right_frame, text="Air Reset",
            command=lambda: self.mcu.air_reset()
        ).grid(row=3, column=3)
        #
        tk.Button(
            right_frame, text="Reset/restart", fg="red",
            command=lambda: self.mcu.reset()
        ).grid(row=3, column=4)
        #
        tk.Button(right_frame, text="QUIT", fg="red", command=self._on_quit).grid(row=3, column=5)
        #
        # # FCU related
        tk.Button(
            right_frame, text="FCM HOME", fg="blue",
            command=lambda: self.mcu.move_fcm(1, 1, 1, 1)
        ).grid(row=4, column=0)
        #
        tk.Button(
            right_frame, text="FCM CLOSED", fg="blue",
            command=lambda: self.mcu.move_fcm(2, 2, 2, 2)
        ).grid(row=4, column=1)
        #
        tk.Button(
            right_frame, text="FCM OPEN", fg="blue",
            command=lambda: self.mcu.move_fcm(3, 3, 3, 3)
        ).grid(row=4, column=2)

        tk.Button(
            right_frame, text="Clear Digest",
            command=lambda: self.clear_digest()
        ).grid(row=4, column=3)

        tk.Button(
            right_frame, text="Prime flush",
            command=lambda: self.mcu.prime(0, 100, 100)
        ).grid(row=4, column=4)

        tk.Button(
            right_frame, text="Prime contrast",
            command=lambda: self.mcu.prime(1, 100, 100)
        ).grid(row=4, column=5)

        tk.Button(
            right_frame, text="Prime all",
            command=lambda: self.mcu.prime(2, 100, 100)
        ).grid(row=4, column=6)

        # right command frame ---------------------------------------------
        speed_frame = tk.Frame(self, width=frame_width, height=frame_height)
        speed_frame.grid(row=4, column=0)

        tk.Button(
            left_frame, text="⏹Stop", fg="red",
            command=lambda: self.mcu.stop(True)).grid(row=0, column=0)

        tk.Button(
            left_frame, text="M_CAL.S", fg="orange",
            command=lambda: self.mcu.m_cal(0, True)).grid(row=0, column=1)

        tk.Button(
            left_frame, text="⌂Home.S",
            command=lambda: self.mcu.disengage(0, True)).grid(row=0, column=2)

        tk.Button(
            left_frame, text="⬆Up.S",
            command=lambda: self.mcu.motor_up(
                0, 1, self._volume_var.get()*10, self._speed_var.get()*10, True),
                ).grid(row=0, column=3)

        tk.Button(
            left_frame, text="⬇Down.S",
            command=lambda: self.mcu.motor_up(
                0, -1, self._volume_var.get() * 10, self._speed_var.get() * 10, True)
        ).grid(row=0, column=4)

        tk.Button(
            left_frame, text="🔎Plunger.S",
            command=lambda: self.mcu.find_plunger(0, self._speed_var.get()*10, True)
        ).grid(row=0, column=5)

        tk.Button(
            left_frame, text="M_CAL.C", fg="orange",
            command=lambda: self.mcu.m_cal(1, True)).grid(row=1, column=1)

        tk.Button(
            left_frame, text="⌂Home.C",
            command=lambda: self.mcu.disengage(1, True)).grid(row=1, column=2)

        tk.Button(
            left_frame, text="⬆Up.C",
            command=lambda: self.mcu.motor_up(
                1, 1, int(self._volume_var.get() * 10), int(self._speed_var.get() * 10), True),
        ).grid(row=1, column=3)

        tk.Button(
            left_frame, text="⬇Down.C",
            command=lambda: self.mcu.motor_up(
                1, -1, self._volume_var.get() * 10, self._speed_var.get() * 10, True)
        ).grid(row=1, column=4)

        tk.Button(
            left_frame, text="🔎Plunger.C",
            command=lambda: self.mcu.find_plunger(1, self._speed_var.get()*10, True)
        ).grid(row=1, column=5)

        tk.Button(
            left_frame, text="Pull2Push.S",
            command=lambda: self.mcu.pull2push(0, True)
        ).grid(row=0, column=6)

        tk.Button(
            left_frame, text="Pull2Push.C",
            command=lambda: self.mcu.pull2push(1, True)
        ).grid(row=1, column=6)

        tk.Button(
            left_frame, text="Push2Pull.S",
            command=lambda: self.mcu.push2pull(0, True)
        ).grid(row=0, column=7)

        tk.Button(
            left_frame, text="Push2Pull.C",
            command=lambda: self.mcu.push2pull(1, True)
        ).grid(row=1, column=7)

        tk.Button(
            left_frame, text="FILL.S",
            command=lambda: self.mcu.fill(0, 10 * self._speed_var.get(), 500, True)
        ).grid(row=0, column=8)

        tk.Button(
            left_frame, text="FILL.C",
            command=lambda: self.mcu.fill(1, 10 * self._speed_var.get(), 500, True)
        ).grid(row=1, column=8)

        tk.Button(
            left_frame, text="Purge Flush",
            command=lambda: self.mcu.purge(0, int(10*self._speed_var.get()), delta_pressure_in_kpa=30)
        ).grid(row=2, column=1)

        tk.Button(
            left_frame, text="Purge Contrast",
            command=lambda: self.mcu.purge(1, int(10*self._speed_var.get()), delta_pressure_in_kpa=30)
        ).grid(row=2, column=2)

        tk.Button(
            left_frame, text="Purge All",
            command=lambda: self.mcu.purge(2, int(10*self._speed_var.get()), delta_pressure_in_kpa=30)
        ).grid(row=2, column=3)
        # noinspection PyShadowingNames

        def create_entry(parent, text, text_var, row, col, entry_width=9, read_only=True):
            tk.Label(parent, text=text).grid(row=row, column=col, sticky=tk.E)
            if read_only:
                tk.Entry(parent, width=entry_width, textvariable=text_var, state="readonly").grid(row=row, column=col+1)
            else:
                tk.Entry(parent, width=entry_width, textvariable=text_var).grid(row=row, column=col + 1)

        # noinspection PyShadowingNames
        def create_dual_entries(parent, text, text_var1, text_var2, row, col, entry_width=15):
            tk.Label(parent, text=text).grid(row=row, column=col, sticky=tk.E)
            tk.Entry(parent, width=entry_width, textvariable=text_var1, state="readonly").grid(row=row, column=col + 1)
            tk.Entry(parent, width=entry_width, textvariable=text_var2, state="readonly").grid(row=row, column=col + 2)

        injector_frame = tk.LabelFrame(self, width=frame_width, text="Injector:")
        injector_frame.grid(row=6, column=0, columnspan=3, padx=10, pady=2, sticky='EW')
        create_entry(injector_frame, "Injector", self.digest_inject_progress_var, 0, 0)
        create_entry(injector_frame, "complete", self.digest_inject_complete_var, 0, 2)
        create_entry(injector_frame, "Pressure", self.digest_inject_pressure_var, 0, 4)
        create_entry(injector_frame, "S.Pressure", self.digest_flush_pressure_var, 0, 6)
        create_entry(injector_frame, "C.Pressure", self.digest_contrast_pressure_var, 0, 8)

        create_entry(injector_frame, "Battery", self.digest_battery_level_var, 1, 0)
        create_entry(injector_frame, "Power", self.digest_power_source_var, 1, 2)
        create_entry(injector_frame, "Diagnostic", self.digest_diagnostic_var, 1, 4)
        create_entry(injector_frame, "P.ADC", self.digest_pressure_adc, 1, 6)

        # Injector frame: [row 7] ---------------------------------------------
        syringe_frame = tk.LabelFrame(self, width=frame_width, text="Syringes:")
        syringe_frame.grid(row=7, column=0, padx=10, pady=2, sticky='EW')

        tk.Label(syringe_frame, text="Flush").grid(row=0, column=1, sticky=tk.W)
        tk.Label(syringe_frame, text="Contrast").grid(row=0, column=2, sticky=tk.W)
        create_dual_entries(syringe_frame, "Volume x10", self.digest_flush_volume_var, self.digest_contrast_volume_var,
                            1, 0)
        create_dual_entries(syringe_frame, "Flow x100", self.digest_flush_flow_var, self.digest_contrast_flow_var, 2, 0)
        create_dual_entries(syringe_frame, "Current", self.digest_flush_current_var, self.digest_contrast_current_var,
                            3, 0)
        create_dual_entries(syringe_frame, "Plunger", self.digest_flush_plunger_var, self.digest_contrast_plunger_var,
                            4, 0)
        create_dual_entries(syringe_frame, "Plunger ADC", self.digest_flush_plunger_adc_var,
                            self.digest_contrast_plunger_adc_var, 5, 0)
        create_dual_entries(syringe_frame, "Status", self.digest_flush_status_var, self.digest_contrast_status_var, 6,
                            0)
        # create_dual_entries(syringe_frame, "PID", self.digest_flush_PID_var, self.digest_contrast_PID_var, 7, 0)
        # create_dual_entries(syringe_frame, "RT PID", self.digest_flush_RT_PID_var,
        # self.digest_contrast_RT_PID_var, 8, 0)
        # create_dual_entries(syringe_frame, "RT.Flow x100",
        # self.digest_flush_actual_speed_var, self.digest_contrast_actual_speed_var, 9, 0)

        create_dual_entries(syringe_frame, "Flow(air)x100 ml/s", self.digest_flush_air_flow,
                            self.digest_contrast_air_flow, 7, 0)
        create_dual_entries(syringe_frame, "Vol(air)x100 ml", self.digest_flush_air_vol, self.digest_contrast_air_vol,
                            8,
                            0)

    def _on_digest(self):
        print(self.do_digest())
        print(f"active_alarms: {self.digest_active_alarms.get()}")
        print(f"active_hardware: {self.digest_active_hardware_signals.get()}")

    def do_digest(self):
        try:
            data = self.mcu.update_digest()

            self.digest_inject_progress_var.set(data.inject_progress)
            self.digest_inject_complete_var.set(data.inject_complete_state)
            self.digest_inject_pressure_var.set(data.inject_pressure_kpa)
            self.digest_flush_pressure_var.set(data.flush_pressure_kpa)
            self.digest_contrast_pressure_var.set(data.contrast_pressure_kpa)

            self.digest_contrast_status_var.set(self.mcu.syringe_status(data.contrast_syringe_state))
            self.digest_contrast_volume_var.set(data.contrast_volume_x10)
            self.digest_contrast_flow_var.set(data.contrast_flow_x100)
            self.digest_contrast_current_var.set(data.contrast_motor_current_adc)
            self.digest_contrast_plunger_var.set(self.mcu.plunger_status(data.contrast_plunger_state))

            self.digest_flush_status_var.set(self.mcu.syringe_status(data.flush_syringe_state))
            self.digest_flush_volume_var.set(data.flush_volume_x10)
            self.digest_flush_flow_var.set(data.flush_flow_x100)
            self.digest_flush_current_var.set(data.flush_motor_current_adc)
            self.digest_flush_plunger_var.set(self.mcu.plunger_status(data.flush_plunger_state))

            self.digest_flush_plunger_adc_var.set(data.flush_plunger_adc)
            self.digest_contrast_plunger_adc_var.set(data.contrast_plunger_adc)

            self.digest_power_source_var.set(data.power_source)
            self.digest_diagnostic_var.set(data.mcu_diagnostic)
            self.digest_battery_level_var.set(data.battery_level)
            self.digest_pressure_adc.set(data.pressure_adc)

            self.digest_flush_actual_speed_var.set(data.flush_actual_speed_x100)
            self.digest_contrast_actual_speed_var.set(data.contrast_actual_speed_x100)

            self.digest_flush_air_vol.set(data.flush_air_volume_x100)
            self.digest_contrast_air_vol.set(data.contrast_air_volume_x100)

            self.digest_flush_air_flow.set(data.flush_air_speed_x100)
            self.digest_contrast_air_flow.set(data.contrast_air_speed_x100)
            self.digest_active_alarms.set(self.mcu.find_alarm_bitmap(data.alarm_bitmap))
            self.digest_active_hardware_signals.set(self.mcu.find_hardware_bitmap(data.alarm_bitmap))
        except:
            print("", end='')
            return

        with open(self.log_filename, mode='a', newline='') as csv_file:
            plot_writer = csv.writer(csv_file, delimiter=',')

            plot_variables = [self.digest_flush_pressure_var.get(), self.digest_contrast_pressure_var.get(),
                                  self.digest_flush_plunger_adc_var.get(), self.digest_contrast_plunger_adc_var.get(),
                                  self.digest_flush_actual_speed_var.get(), self.digest_contrast_actual_speed_var.get(),
                              self.mcu.unix_timestamp() - self.start_time]
            plot_writer.writerow(plot_variables)

        return data


    def clear_digest(self):
        f = open(self.log_filename, "w+", newline='')
        self.start_time = self.mcu.unix_timestamp()
        f.write("flush_pressure,contrast_pressure,flush_adc,contrast_adc,flush_speed,contrast_speed,time\n")
        f.close()

    def injector_plot(self):
        df = pd.read_csv(self.log_filename)
        df["flush_speed"] = df["flush_speed"] / 100
        df["contrast_speed"] = df["contrast_speed"] / 100
        df1 = pd.DataFrame(df, columns=['flush_pressure', 'contrast_pressure', 'time'])
        df2 = pd.DataFrame(df, columns=['flush_speed', 'contrast_speed', 'time'])
        df3 = pd.DataFrame(df, columns=['flush_adc', 'contrast_adc', 'time'])
        fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)

        # plt.grid()
        # plt.show()

        ax1 = df1.plot("time", ax=axes[0])
        ax1.set_ylabel("Pressure (kpa)")
        ax1.grid()

        ax2 = df2.plot("time", ax=axes[1])
        ax2.set_ylabel("Speed (ml/s)")
        ax2.grid()

        ax3 = df3.plot("time", ax=axes[2])
        ax3.set_ylabel("ADC (V)")
        ax3.grid()

        plt.show()
    #     flush_pressure_index = 0
    #     contrast_pressure_index = 1
    #     flush_adc_index = 2
    #     contrast_adc_index = 3
    #     flush_speed_index = 4
    #     contrast_speed_index = 5
    #     time_index = 6
    #
    #     flush_pressure = []
    #     contrast_pressure = []
    #     seconds = []
    #     with open(self.log_filename, mode='r') as csv_file:
    #         file = csv.reader(csv_file)
    #         for lines in file:
    #             flush_pressure.append(int(lines[flush_pressure_index]))
    #             contrast_pressure.append(int(lines[contrast_pressure_index]))
    #             seconds.append(float(lines[time_index]))

    #TODO




def main(main_com: str, debug_com: str, output_dir: str):
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    # log_filename = "mcu_link_%s.txt" % get_datetime_string()

    mcu = MotorController(main_com, debug_com, output_dir)
    mcu.set_main_com_verbose(True)
    root = tk.Tk()
    app = MotorControllerApplication(root, mcu, output_dir)
    digest_thread = DigestThread(app)

    def on_app_quit():
        print("WM_DELETE_WINDOW -> on_app_quit()")
        # mcu.stop_monitor_reading_thread()
        digest_thread.set_stop()
        root.destroy()

    # need to terminate the threads with stop_monitor_reading_thread()
    # on exit or the process does not terminate.
    root.protocol("WM_DELETE_WINDOW", on_app_quit)

    app.mainloop()
    del app


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Salient EVO Motor controller UI')
    parser.add_argument('--main_com', type=str, default="",
                        help='The FTDI USB serial COM port. Default find first FTDI COM')
    parser.add_argument('--debug_com', type=str, default="",
                        help='The USB serial COM port (for download speed). Default find the first USB device')
    parser.add_argument('--output_dir', type=str, default="./output", help='The output directory')
    parser.add_argument('--list', action='store_true', help='List the available serial com ports')
    args = parser.parse_args()
    if args.list:
        import serial.tools.list_ports as list_ports

        for i, info in enumerate(sorted(list_ports.comports())):
            print("info", info)
    else:
        main(
            args.main_com,
            args.debug_com,
            os.path.abspath(args.output_dir)
        )
