import os
import time
import tkinter as tk
from collections import deque

from Test_MotorController_JZ import MotorController, get_datetime_string
from pid_analysis import PIDAnalyser
from serial_port_read_pid import parse_motor_pid_binary_no_struct_size, get_binary_pid_header, \
    get_pid_name_tuple, get_pid_data_dict


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
        self.digest_flush_PID_var = tk.StringVar(self, value="PID")
        self.digest_flush_RT_PID_var = tk.StringVar(self, value="RT_PID")
        self.digest_flush_actual_speed_var = tk.StringVar(self, value="Actual.Flow")

        self.digest_contrast_status_var = tk.StringVar(self, value="status")
        self.digest_contrast_volume_var = tk.StringVar(self, value="volume")
        self.digest_contrast_flow_var = tk.StringVar(self, value="flow")
        self.digest_contrast_current_var = tk.StringVar(self, value="current")
        self.digest_contrast_plunger_var = tk.StringVar(self, value="plunger")
        self.digest_contrast_plunger_adc_var = tk.StringVar(self, value="plungerADC")
        self.digest_contrast_PID_var = tk.StringVar(self, value="PID")
        self.digest_contrast_actual_speed_var = tk.StringVar(self, value="Actual.Flow")
        self.digest_contrast_RT_PID_var = tk.StringVar(self, value="RT_PID")

        self.digest_pressure_adc = tk.StringVar(self, value="pressureADC")
        self.digest_battery_level_var = tk.StringVar(self, value="battery")
        self.digest_power_source_var = tk.StringVar(self, value="power")
        self.digest_diagnostic_var = tk.StringVar(self, value="diagnostic")

        self.digest_flush_air_flow = tk.StringVar(self, value="Air.Flow")
        self.digest_contrast_air_flow = tk.StringVar(self, value="Air.Flow")

        self.digest_flush_air_vol = tk.StringVar(self, value="Air.Vol")
        self.digest_contrast_air_vol = tk.StringVar(self, value="Air.Vol")

        self.mcu = mcu
        self.out_dir = out_dir
        if not os.path.exists(self.out_dir):
            os.mkdir(self.out_dir)

        self._binary_queue = deque(maxlen=1000)

        self._debug_headers = get_binary_pid_header().split(",")

        self._create_widgets()
        self.master.after(20, self._update_binary_queue_timer)
        self.master.after(100, self._do_digest)

    def _on_quit(self):
        # self._on_cycle_test_stop()
        self.mcu.stop_monitor_reading_thread()
        self.master.destroy()

    def read_all_binary_queue_data(self):
        samples = []
        while len(self._binary_queue) > 0:
            data = self._binary_queue.popleft()
            samples.append(data)
        return samples

    def _update_binary_queue_timer(self):
        samples = self.read_all_binary_queue_data()
        if len(samples):
            self._timer_counter += 1
            new_data = np.array(samples).reshape((len(samples), -1))
            self._debug_data = new_data  # Keep the last read debug data
            # print("new length", len(self._debug_data), len(new_data))

            data = samples[-1]  # get the last sample
            pid_info = get_pid_name_tuple(data)
            index = pid_info.MotorIndex
            data_dict = get_pid_data_dict(data)

            # Update the UI variables in motors_dicts
            for k, str_var in self.motors_dicts[index].items():
                value = data_dict[k]
                str_var.set(str(value))
        self.master.after(10, self._update_binary_queue_timer)

    def s_push_c_pull(self, motor, speed_gap=10, start_pressure=50, max_pressure=1000, delta_pressure=200, stable_duration=1000):
        self._do_digest()

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

    def on_enable_debug(self, motor):
        enable = 0
        self.mcu.enable_debug(motor)
        if self.plot_frame:
            self.plot_frame.set_animation(enable)

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

        tk.Button(right_frame, text="️S.Push,C.Pull", command=lambda: self.s_push_c_pull(0, 200, 50, 1000)).grid(row=2, column=0)
        tk.Button(right_frame, text="C.Push,S.Pull", command=lambda: self.s_push_c_pull(1, 200, 50, 1000)).grid(row=2, column=1)

        tk.Button(
            right_frame, text="Arm(6 phases)",
            command=lambda: self.mcu.inject_arm(500, 6, 40, 50, 50, 200, 100, 50, 200, 0, 50, 200, 5, 50, 200, 95, 50, 200, 50, 50, 200, 0)
        ).grid(row=3, column=0)
        #
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
            command=lambda: self.mcu.move_fcm(2, 2,2,2)
        ).grid(row=4, column=1)
        #
        tk.Button(
            right_frame, text="FCM OPEN", fg="blue",
            command=lambda: self.mcu.move_fcm(3,3,3,3)
        ).grid(row=4, column=2)

        # right command frame ---------------------------------------------
        speed_frame = tk.Frame(self, width=frame_width, height=frame_height)
        speed_frame.grid(row=4, column=0)

        tk.Button(
            left_frame, text="⏹Stop", fg="red",
            command=lambda: self.mcu.stop(True)).grid(row=0, column=0)

        tk.Button(
            left_frame, text="MCAL.S", fg="orange",
            command=lambda: self.mcu.MCAL(0 ,True)).grid(row=0, column=1)

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
            left_frame, text="MCAL.C", fg="orange",
            command=lambda: self.mcu.MCAL(1, True)).grid(row=1, column=1)

        tk.Button(
            left_frame, text="⌂Home.C",
            command=lambda: self.mcu.disengage(1, True)).grid(row=1, column=2)

        tk.Button(
            left_frame, text="⬆Up.C",
            command=lambda: self.mcu.motor_up(
                1, 1, self._volume_var.get() * 10, self._speed_var.get() * 10, True),
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
            command=lambda: self.mcu.fill(0, 10 * self._speed_var.get(), True)
        ).grid(row=0, column=8)

        tk.Button(
            left_frame, text="FILL.C",
            command=lambda: self.mcu.fill(1, 10 * self._speed_var.get(), True)
        ).grid(row=1, column=8)

        # pid_frame = tk.LabelFrame(self, width=frame_width, text="PID Data")
        # pid_frame.grid(row=4, column=0, padx=10, pady=2, sticky='EW')
        #
        # debug_motor1_frame = tk.LabelFrame(pid_frame, width=frame_width, text="Contrast:")
        # debug_motor1_frame.grid(row=0, column=0, padx=10, pady=2, sticky='EW')
        # debug_motor2_frame = tk.LabelFrame(pid_frame, width=frame_width, text="Flush:")
        # debug_motor2_frame.grid(row=0, column=1, padx=10, pady=2, sticky='EW')
        # motor_frames = debug_motor1_frame, debug_motor2_frame
        # self.motors_dicts = dict(), dict()


        # noinspection PyShadowingNames
        # def add_motor_entry(motor_index: int, label_name: str, data_name: str, tk_frame: tk.LabelFrame, row: int,
        #                     col: int,
        #                     width: int):
        #     tk.Label(tk_frame, text=label_name).grid(row=row, column=col, sticky=tk.E)
        #     str_var = tk.StringVar(self, value=data_name)
        #     tk.Entry(tk_frame, textvariable=str_var, width=width, state='readonly').grid(row=row, column=col + 1)
        #     self.motors_dicts[motor_index][data_name] = str_var
        #
        # for index, frame in enumerate(motor_frames):
        #     row = 0
        #     add_motor_entry(index, "Time:",             "T",                    frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "Lag:",              "lag",                  frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "AngleOffset:",      "AngleOffset",          frame, row=row, col=4, width=9)
        #
        #     row += 1
        #     add_motor_entry(index, "Speed(mL/s):",      "Va",                   frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "Position(mL):",     "Pa",                   frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "EncoderIndex:",     "EncoderIndex",         frame, row=row, col=4, width=9)
        #
        #     row += 1
        #     add_motor_entry(index, "Motor State:",      "MotorState",           frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "ProgramEnd:",       "ProgramEnd",           frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "Home:",             "Home",                 frame, row=row, col=4, width=9)
        #     add_motor_entry(index, "HasIndex:",         "HasIndex",             frame, row=row, col=6, width=9)
        #
        #     row += 1
        #     add_motor_entry(index, "PlungerADC:",       "PlungerADC",           frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "t1_adc:",           "t1_adc",               frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "current:",          "current",              frame, row=row, col=4, width=9)
        #
        #     row += 1
        #     add_motor_entry(index, "PID_e",             "pid_e",                frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "PID_i:",            "pid_i",                frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "PID_d:",            "pid_d",                frame, row=row, col=4, width=9)
        #     add_motor_entry(index, "pid:",              "pid",                  frame, row=row, col=6, width=9)
        #
        #     row += 1
        #     add_motor_entry(index, "last_u:",           "last_u",               frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "Pr:",               "Pr",                   frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "effort:",           "effort",               frame, row=row, col=4, width=9)
        #     add_motor_entry(index, "FilteredPid:",      "FilteredPid",          frame, row=row, col=6, width=9)
        #     row += 1
        #     add_motor_entry(index, "T1:",               "T1",                   frame, row=row, col=0, width=9)
        #     add_motor_entry(index, "Pressure:",         "pressure",             frame, row=row, col=2, width=9)
        #     add_motor_entry(index, "P.Pressure:",       "PredictedPressureKpa", frame, row=row, col=4, width=9)
        #     add_motor_entry(index, "reserved:",         "reserved",             frame, row=row, col=6, width=9)

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
        # create_dual_entries(syringe_frame, "RT PID", self.digest_flush_RT_PID_var, self.digest_contrast_RT_PID_var, 8, 0)
        # create_dual_entries(syringe_frame, "RT.Flow x100", self.digest_flush_actual_speed_var, self.digest_contrast_actual_speed_var, 9, 0)

        create_dual_entries(syringe_frame, "Flow(air)x100 ml/s", self.digest_flush_air_flow,
                            self.digest_contrast_air_flow, 7, 0)
        create_dual_entries(syringe_frame, "Vol(air)x100 ml", self.digest_flush_air_vol, self.digest_contrast_air_vol,
                            8,
                            0)

    def _on_digest(self):
        print(self._do_digest())


    def _do_digest(self):
        data = self.mcu.update_digest(verbose=self._verbose)

        # digest OK Digest(alarms='', InjectProgress='IDLE', InjectCompleteState='NORMAL', InjectPressureKpa='0',
        # ContrastSyringeStatus='Completed', ContrastVolume='0', ContrastFlow='0', ContrastMotorCurrent='0',
        # ContrastPlungerState='Disengaged', FlushSyringeStatus='Completed', FlushVolume='-2750', FlushFlow='0',
        # FlushMotorCurrent='112', FlushPlungerState='Disengaged',
        # BatteryLevel='Critical', PowerSource='AC', mcu_diagnostic='')
        # for bit in range(14):
        # data.hardware_bitmap
        # self.mcu.hardware_bitmap_decode
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

        self.digest_flush_PID_var.set(data.flush_realtime_pid)
        self.digest_contrast_PID_var.set(data.contrast_realtime_pid)

        self.digest_flush_RT_PID_var.set(data.flush_motor_pid)
        self.digest_contrast_RT_PID_var.set(data.contrast_motor_pid)

        self.digest_flush_actual_speed_var.set(data.flush_actual_speed_x100)
        self.digest_contrast_actual_speed_var.set(data.contrast_actual_speed_x100)

        self.digest_flush_air_vol.set(data.flush_air_volume_x100)
        self.digest_contrast_air_vol.set(data.contrast_air_volume_x100)

        self.digest_flush_air_flow.set(data.flush_air_speed_x100)
        self.digest_contrast_air_flow.set(data.contrast_air_speed_x100)

        #self.master.after(200, self._do_digest)
        return data


def main(main_com: str, debug_com: str, output_dir: str):
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    log_filename = "mcu_link_%s.txt" % get_datetime_string()

    mcu = MotorController(main_com, debug_com, output_dir, log_filename)
    mcu.set_main_com_verbose(True)
    root = tk.Tk()
    app = MotorControllerApplication(root, mcu, output_dir)

    def on_app_quit():
        print("WM_DELETE_WINDOW -> on_app_quit()")
        mcu.stop_monitor_reading_thread()
        root.destroy()

    # need to terminate the threads with stop_monitor_reading_thread()
    # on exit or the process does not terminate.
    root.protocol("WM_DELETE_WINDOW", on_app_quit)

    app.mainloop()
    del app


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Salient EVO Motor controller UI')
    parser.add_argument('--main_com', type=str, default="",  help='The FTDI USB serial COM port. Default find first FTDI COM')
    parser.add_argument('--debug_com', type=str, default="", help='The USB serial COM port (for download speed). Default find the first USB device')
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
