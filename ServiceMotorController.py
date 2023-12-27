# MotorController
import math

from serial import Serial, SerialException
import serial.tools.list_ports as list_ports
import time
from collections import defaultdict, deque
import json
import threading
from sys import exit
import os
from datetime import datetime
import traceback

from fcm.fcm_pb_transport import FcmPbTransport
from mcu_pb_transport import McuPbTransport
from transport_stream import FcmUsbStream, McuUsbStream
import mcu_commands_pb2 as CommandMsg
import cli_common_pb2 as MsgValues

response_codes = {"OK", "FAIL", "INVALID_COMMAND", "INVALID_PARAMETER", "INVALID_STATE", "REPEATED_OUTPUT"}
logger = None
McuTypes = json.load(open('mcu_cli_specification.json', 'r'))

digest_headers = \
    (
        "alarm_bitmap",    				# Each bit corresponds to a specific alarm status where being '1' indicates an active alarm, and '0' indicates an inactive alarm. The Alarm enumeration defines meaningful names for each alarm's index in the bitmap.
        "fcm_pinch_alarm_bitmap",    	# FCM Pinched Motor Alarms - see FCM CLI's Alarm for more information to decode the FCM alarm bitmap
        "hardware_signal_bitmap",    	# MCU button/sensor pressed/on/detected state bitmap. Each bit corresponds to a specific button status where being '1' indicates an active alarm, and '0' indicates an inactive alarm. The Button enumeration defines meaningful names for each alarm's index in the bitmap.
        "contrast_syringe_type",    	# The contrast syringe type.
        "flush_syringe_type",    		# The flush syringe type
        "flush_fill_valve_pos",    		# The FCM's flush fill valve position
        "flush_inject_valve_pos",    	# The FCM's flush inject valve position
        "contrast_fill_valve_pos",    	# The FCM's contrast fill valve position
        "contrast_inject_valve_pos",    # The FCM's contrast inject valve position
        "flush_fill_valve_state",    	# The FCM's flush fill valve state
        "flush_inject_valve_state",    	# The FCM's flush inject valve state
        "contrast_fill_valve_state",    # The FCM's contrast fill valve state
        "contrast_inject_valve_state",  # The FCM's contrast inject valve state
        "inject_phase",    				# The current injection phase. Only valid during an injection
        "inject_phase_time_ms",    		# Current injection phase time in milliseconds.
        "inject_phase_volume_delivered_x10",    # , 0.1 ml, Current injection phase volume already delivered in 0.1 ml
        "inject_progress",    			# The inject progress (status)
        "inject_complete_state",    	# The inject complete state
        "inject_pressure_kpa",    		# The predicted inject pressure in kpa unit
        "contrast_pressure_kpa",    	# The predicted contrast pressure in kpa unit
        "flush_pressure_kpa",    		# The predicted flush pressure in kpa unit
        "contrast_syringe_state",    	# The contrast syringe status
        "contrast_volume_x10",    		# The contrast volume in .1 ml unit
        "contrast_flow_x100",    		# The contrast flow rate in .01 ml/s
        "contrast_motor_pid",    		# The contrast motor PID
        "contrast_actual_speed_x100",   # The contrast actual speed in .01 ml/s
        "contrast_motor_current_adc",   # The contrast motor current ADC (12 bits)
        "contrast_plunger_adc",    		# The contrast plunger ADC (12 bits)
        "contrast_plunger_state",    	# The contrast plunger state
        "flush_syringe_state",    		# The flush syringe status
        "flush_volume_x10",    			# The flush plunger volume in .1 ml unit
        "flush_flow_x100",    			# The flush flow rate in .01 ml/s unit
        "flush_motor_pid",    			# The flush motor PID value
        "flush_actual_speed_x100",    	# The flush plunger speed in .01 ml unit
        "flush_motor_current_adc",    	# The flush motor current ADC (12 bits)
        "flush_plunger_adc",    		# The flush plunger ADC (12 bits)
        "flush_plunger_state",    		# The flush plunger state
        "pressure_adc",    				# The pressure ADC reading (12 bits)
        "battery_level",    			# The battery level. Valid if the power source is battery
        "power_source",    				# The power source (AC or battery)
        "flush_air_speed_x100",    		# The estimated air speed in the flush syringe in .01 ml/s unit
        "flush_air_volume_x100",    	# The estimated air volume in the flush syringe in .01 ml unit
        "contrast_air_speed_x100",    	# The estimated air speed in the contrast syringe in .01 ml/s unit
        "contrast_air_volume_x100",    	# The estimated air volume in contrast syringe in .01 ml unit
        "contrast_realtime_pid",    	# Contrast realtime PID (smaller size filtering compared with contrast_motor_pid)
        "flush_realtime_pid",    		# flush realtime PID (smaller size filtering compared with flush_motor_pid)
        "mcu_diagnostic"    			# Free text subfields with : separator. <type>[:<field>]* Refer to Diagnostic Message for more details
    )


def get_datetime_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


class MotorController:
    SCI1_BAUD_RATE = 115200  # Main MCU/HCU rate
    SCI_TIME_OUT = 2  # seconds
    SCI5_BAUD_RATE = 115200  # SCI5 debug port rate
    USB_PDC_BAUD_RATE = 115200
    SCI5_TIME_OUT = 1  # seconds

    def __init__(self, main_com: str, debug_com: str, output_dir: str, log_filename=""):
        self._is_open = False
        self.status = None
        self._main_serial = None  # of type Serial() or on startup
        self._main_comm_queue = deque(maxlen=10000)
        self._main_comm_thread = None
        self._main_com_verbose = False

        # self._log_file = None
        self._digest_log = None
        self._digest_log_start_time = time.time_ns()

        self._monitor_serial = None  # of type Serial() or None if start_monitor_reading_thread() is not called.
        self._monitor_com_thread = None
        self._stop_monitor = False
        self._monitor_count = 0
        # self._monitor_log_file = None

        self._output_dir = os.path.abspath(output_dir)
        if not os.path.exists(self._output_dir):
            os.mkdir(self._output_dir)

        self.last_digest_data = None

        # dictionary of array of elapse time measure in seconds, key = command name
        self._response_time_dict = defaultdict(list)
        self.last_command_duration = 0  # duration of the response of the last command

        if len(log_filename) == 0:
            # create MCU with the link name and current system timestamp
            log_filename = self.get_filename_from_current_timestamp(prefix="MCU-%s" % main_com)
        else:
            log_filename = os.path.join(self._output_dir, log_filename)
        self._log_file = open(log_filename, "w")
        print("Set output directory:", self._output_dir)
        print("MCU log filename", log_filename)

        try:
            self.mcu_stream = McuUsbStream(logging=logger)
            self.fcm_stream = FcmUsbStream(logging=logger)
            self.mcu_stream.open(logging=logger)
            self.mcu_transport = McuPbTransport(logger, self.mcu_stream)
            self.fcm_transport = FcmPbTransport(logger, self.fcm_stream)
        except FileNotFoundError as e:
            print("Error")
            traceback.print_exception(e)
            exit(1)

        self._monitor_port_name = debug_com

    def get_filename_from_current_timestamp(self, prefix="", postfix="", is_dir=False) -> str:
        """ Get an absolute filename from the current timestamp (yyyymmdd-hhmmss) with optional prefix or post-fix,
        if the name is a directory it will be created
        """
        dt_str = get_datetime_string()

        if len(prefix):
            dt_str = prefix + dt_str

        if len(postfix):
            dt_str = dt_str + postfix

        out_name = os.path.join(self._output_dir, dt_str)
        if is_dir and not os.path.exists(out_name):
            os.mkdir(out_name)

        return out_name

    def get_port_by_name(self, name: str) -> str:
        # noinspection SpellCheckingInspection
        for i, (port, desc, hwid) in enumerate(sorted(list_ports.comports())):
            # print("usb", name, desc)
            if name in desc:
                # noinspection SpellCheckingInspection
                return port
        return ""

    @staticmethod
    def find_ftdi_port() -> str:
        """Find the first FTDI port and return the name"""
        arr = []
        for i, info in enumerate(sorted(list_ports.comports())):
            if info.manufacturer == "FTDI":
                arr.append(info.name)
                return info.name
        return ""

    @classmethod
    def find_mcu_usb_port(cls) -> str:
        for name in ("USB Serial Device",):  # add more name as needed
            return cls.get_port_by_name(name)

    def set_main_com_verbose(self, verbose: bool = True):
        self._main_com_verbose = verbose

    # threading code
    def read_line(self, wait: bool = True) -> str:
        """
        waiting for the main comm thread to read a line
        @todo using sync method instead of polling
        :return:
        """
        if self._monitor_com_thread:
            if wait:
                while len(self._main_comm_queue) == 0:
                    time.sleep(0.01)
                return self._main_comm_queue.popleft()
            else:
                if len(self._main_comm_queue):
                    return self._main_comm_queue.popleft()
                else:
                    return ""
        else:
            # attempt to read if the main com if the main thread has not started
            line = self._main_serial.read_until().decode()
            if len(line):
                line = line.rstrip()
                if self._log_file:
                    if self._main_com_verbose:
                        print("M:", line)
                    self._log_file.write(line + "\n")
                    self._log_file.flush()
            return line

    def _main_comm_thread_task(self):
        """
        Command will be sent directly, the response will be read via this thread and added to the queue
        :return:
        """
        print("Main communication thread start and reading from %s(FTDI)" % self._main_port_name)
        while True:
            line = self._main_serial.read_until().decode()
            if len(line):
                line = line.rstrip()
                self._main_comm_queue.append(line)
                print(line)
                if self._log_file:
                    if self._main_com_verbose:
                        print("M:", line)
                    self._log_file.write(line + "\n")
                    self._log_file.flush()
            if self._stop_monitor:
                break
        print("Main communication thread end")

    def start_monitor_reading_thread(self, monitor_file) -> bool:
        """
        Start the reading thread if it has not been started
        :param monitor_file:  file handle in binary mode
        :return:
        """
        self._monitor_count = 0
        if self._monitor_com_thread is not None:
            return False

        if self._monitor_port_name == "":
            # Attempt to find the monitor port (prefer high speed USB port)
            self._monitor_port_name = self.find_mcu_usb_port()
            if self._monitor_port_name == "":
                print("ERROR missing USB COM device. cannot start monitor thread.")
                exit(1)

            print("Use USB %s as the monitor COM port for speed" % self._monitor_port_name)

        if self._main_comm_thread is None:
            self._main_comm_thread = threading.Thread(target=self._main_comm_thread_task)
            self._main_comm_thread.start()

        if self._monitor_log_file:
            self._monitor_log_file.close()
            self._monitor_log_file = None

        if monitor_file is None:
            print("The monitor port is not used")
            self._stop_monitor = True
            return True

        # want timeout every second so we stop the thread quickly
        try:
            self._monitor_log_file = monitor_file  # open(output_filename, "wb")
            self._monitor_serial = Serial(self._monitor_port_name, baudrate=self.SCI5_BAUD_RATE,
                                          timeout=self.SCI5_TIME_OUT)
        except SerialException as e:
            print("Exception while open monitor port", e)
            traceback.print_exception(e)
            self._monitor_serial = None
            self._stop_monitor = True
            if self._main_comm_thread is not None:
                self._main_comm_thread.join()  # thread.join

            if self._monitor_log_file:
                self._monitor_log_file.close()
                self._monitor_log_file = None
            exit(2)

        print("SKIP: START: try to escape USB binary CLI to text CLI")
        # self._monitor_serial.write(bytearray("\r\n", encoding='utf-8'))  # try to clear/stop any of the previous PIDDebug command
        while 1:
            self._monitor_serial.write(b'\r')
            time.sleep(0.1)
            try:
                raw_data = self._monitor_serial.read_all()
                text = raw_data.decode()
                print("SKIP:", len(text), text)
            except UnicodeDecodeError as ex:
                print("Skipping binary data", ex)
                continue
            if "Switching CLI from binary to text mode" in text:
                break
            elif "INVALID_COMMAND" in text:
                break
        while 1:
            time.sleep(0.1)
            data = self._monitor_serial.read_all()
            if len(data):
                print("SKIP:", data)
            else:
                break
        self._monitor_serial.reset_input_buffer()
        print("SKIP: DONE!")
        self._main_serial.read_all()
        # self._monitor_com_thread = threading.Thread(target=self._monitor_thread_function)
        # self._monitor_com_thread.start()
        print("Debug port is ready")
        return True

    def stop_monitor_reading_thread(self):
        if self._monitor_com_thread:
            print("Stopping the monitor thread")
            self._stop_monitor = True
            self._monitor_com_thread.join()
            self._monitor_com_thread = None
            if self._monitor_log_file:
                self._monitor_log_file.close()
                self._monitor_log_file = None
            print("The monitor thread has stopped")
            self._main_comm_thread.join()
            print("The main thread has stopped")

    def find_alarm_bitmap(self, bitmap):
        alarm_list = []
        alarm_name_list = []
        alarm_str = bitmap
        all_alarms = MsgValues.Alarm.items()

        # The maximum number of alarms triggered is the distance of the msb bit in the alarm integer
        # Add one just in case an alarm is missed
        number_of_alarms = math.ceil(math.log2(alarm_str)) + 1
        for alarm in range(number_of_alarms):
            if alarm_str & (1 << alarm) != 0:
                alarm_list.append(alarm)
        # print(alarm_list)
        if len(all_alarms) > 0:
            i = 0
            for alarm, alarm_number in all_alarms:
                if alarm_number in alarm_list:
                    alarm_name_list.append(alarm)
                i += 1
            return ', '.join(alarm_name_list)
        else:
            return ""

    def find_hardware_bitmap(self, bits):
        data = list(filter(lambda t: t['name'] == "HardwareSignal", McuTypes['support_types']))[0]['categorical_values']
        bitmap = []
        bit_shift_count = 0
        # if len(data) > 0:
        #     for i in range(0, 32):
        #         print(i)
        #         if bits & 1 << i != 0:
        #             bitmap.append(data[i]['name'])
        for i in data:
            if bits & 1 << bit_shift_count != 0:
                bitmap.append(i['name'])
            bit_shift_count += 1
        return ', '.join(bitmap)

    def _get_digest(self):
        digest = CommandMsg.DIGEST_Command()
        digest.delay_in_ms = 0
        dr = self.mcu_transport.send_digest(digest)[0]
        return dr

    def _print_digest(self):
        print(self._get_digest())

    def update_digest(self):
        self.last_digest_data = self._get_digest()
        return self.last_digest_data

    def plunger_status(self, plunger_state):
        return ''.join(MsgValues.PlungerState.Name(int(plunger_state)).split("_")[1:])

    def syringe_status(self, syringe_state):
        return ''.join(MsgValues.SyringeState.Name(int(syringe_state)).split("_")[2:])

    def s_push_c_pull(self, motor):
        # motor index will push, other index will pull
        self.update_digest()

        if (
                self.last_digest_data.flush_syringe_state == MsgValues.SyringeState.PROCESSING
                or self.last_digest_data.contrast_syringe_state
                == MsgValues.SyringeState.PROCESSING):
            print("ERROR: Either motors are still busy")
            return False

        if self.last_digest_data.contrast_plunger_state != "ENGAGED":
            print("does not have a contrast plunger")
            return False

        if self.last_digest_data.flush_plunger_state != "ENGAGED":
            print("does not have a flush plunger")
            return False

        if self.get_syringe_volume(motor) <= self.get_syringe_volume(motor % 2 + 1):
            print("ERROR: need push motor to have more volume than pull motor", self.get_syringe_volume(motor),
                  self.get_syringe_volume(motor % 2 + 1))
            return False

    def stop(self, verbose: bool = True):
        if verbose:
            print("Stopping")

        command = CommandMsg.STOP_Command()
        command_data = self.mcu_transport.send_stop(command)
        return command_data

    def mcal(self, motor: int, verbose: bool = True):
        if verbose:
            if motor == 0:
                print("Executing MCAL_flush")
            else:
                print("Executing MCAL_contrast")
        command = CommandMsg.MCAL_Command()
        command.motor = motor
        command_data = self.mcu_transport.send_mcal(command)
        return command_data

    def disengage(self, motor: int, verbose: bool = True):
        if verbose:
            if motor == 0:
                print("Executing disengage_flush")
            if motor == 1:
                print("Executing disengage_contrast")
        command = CommandMsg.DISENGAGE_Command()
        command.motor = motor
        command_data = self.mcu_transport.send_disengage(command)
        return command_data

    def piston(self, motor, volume, speed):
        command = CommandMsg.PISTON_Command()
        command.motor = motor
        command.volume_x10 = int(volume)
        command.speed_x10 = int(speed)
        return self.mcu_transport.send_piston(command)

    def motor_up(self, motor: int, direction: int, volume: int, speed: int, verbose: bool = True):
        if verbose:
            if motor == 0:
                if direction > 0:
                    print("Executing flush_up")
                elif direction < 0:
                    print("Executing flush_down")
            if motor == 1:
                if direction > 0:
                    print("Executing contrast_up")
                elif direction < 0:
                    print("Executing contrast_down")

        command = CommandMsg.PISTON_Command()
        command.motor = motor
        command.speed_x10 = int(speed * direction)
        command.volume_x10 = int(volume)
        command_data = self.mcu_transport.send_piston(command)
        return command_data

    def find_plunger(self, motor: int, speed: int = 10, verbose=True):
        if verbose:
            if motor == 0:
                print("Executing find_plunger flush")
            elif motor == 1:
                print("Executing find_plunger contrast")
            elif motor == 2:
                print("Executing find_plunger all")

        command = CommandMsg.FIND_PLUNGER_Command()
        command.speed_x10 = int(speed)
        command.motor = motor
        return self.mcu_transport.send_piston(command)

    def sys(self, verbose: bool = True):
        if verbose:
            print("Executing sys")
        command = CommandMsg.SYS_Command()
        return self.mcu_transport.send_sys(command)

    def pull2push(self, motor: int, verbose: bool = True):
        if verbose:
            if motor == 0:
                print("Executing Pull2Push flush")
            else:
                print("Executing Pull2Push contrast")
        command = CommandMsg.PULL_TO_PUSH_Command()
        command.motor = motor
        return self.mcu_transport.send_pull_to_push(command)

    def push2pull(self, motor: int, verbose: bool = True):
        if verbose:
            if motor == 0:
                print("Executing Pushl2Pull flush")
            else:
                print("Executing Push2Pull contrast")
        command = CommandMsg.PUSH_TO_PULL_Command()
        command.motor = motor
        return self.mcu_transport.send_push_to_pull(command)

    def fill(self, motor: int, speed: int, volume: int = 1000, verbose: bool = True):
        if verbose:
            if motor == 0:
                print("Executing Fill flush")
            else:
                print("Executing Fill contrast")

        command = CommandMsg.FILL_Command()
        command.motor = motor
        command.speed_x10 = int(speed)
        command.volume_x10 = int(volume)

        return self.mcu_transport.send_fill(command)

    def get_syringe_volume(self, syringe: int):
        if syringe == 0:
            # print(self._get_digest().flush_volume_x10)
            return self._get_digest().flush_volume_x10
        else:
            return self._get_digest().contrast_volume_x10

    def pull_pressure_ramp(self, motor, ramp_flow_rate, ramp_delta_pressure, initial_pressure, max_pressure,
                           stable_flow, stable_time):
        command = CommandMsg.PULL_PRESSURE_RAMP_Command()
        command.motor = motor
        command.ramp_floor_rate_x10 = int(ramp_flow_rate)
        command.ramp_delta_pressure_psi_x10 = int(ramp_delta_pressure)
        command.initial_pressure_psi_x10 = int(initial_pressure)
        command.max_pressure_psi_x10 = int(max_pressure)
        command.stable_flow_rate_x10 = int(stable_flow)
        command.stable_time_in_ms = int(stable_time)
        return self.mcu_transport.send_pull_pressure_ramp(command)

    def inject_arm(self, max_pressure: int, phase_count: int, flow_reduction_percentage: int,
                   pressure_limit_sensitivity_percentage: int, phase_0_flow_rate_x10: int, phase_0_volume_x10: int,
                   phase_0_mix_pc: int, phase_1_flow_rate_x10: int, phase_1_volume_x10: int, phase_1_mix_pc: int,
                   phase_2_flow_rate_x10: int, phase_2_volume_x10: int, phase_2_mix_pc: int, phase_3_flow_rate_x10: int,
                   phase_3_volume_x10: int, phase_3_mix_pc: int, phase_4_flow_rate_x10: int, phase_4_volume_x10: int,
                   phase_4_mix_pc: int,
                   phase_5_flow_rate_x10: int, phase_5_volume_x10: int, phase_5_mix_pc: int):
        command = CommandMsg.INJECT_ARM_Command()
        command.max_pressure_in_kpa = max_pressure
        command.phase_count = phase_count
        command.flow_reduction_percentage = flow_reduction_percentage
        command.pressure_limit_sensitivity_percentage = pressure_limit_sensitivity_percentage
        command.phase_0_flow_rate_x10 = phase_0_flow_rate_x10
        command.phase_0_volume_x10 = phase_0_volume_x10
        command.phase_0_mix_pc = phase_0_mix_pc
        command.phase_1_flow_rate_x10 = phase_1_flow_rate_x10
        command.phase_1_volume_x10 = phase_1_volume_x10
        command.phase_1_mix_pc = phase_1_mix_pc
        command.phase_2_flow_rate_x10 = phase_2_flow_rate_x10
        command.phase_2_volume_x10 = phase_2_volume_x10
        command.phase_2_mix_pc = phase_2_mix_pc
        command.phase_3_flow_rate_x10 = phase_3_flow_rate_x10
        command.phase_3_volume_x10 = phase_3_volume_x10
        command.phase_3_mix_pc = phase_3_mix_pc
        command.phase_4_flow_rate_x10 = phase_4_flow_rate_x10
        command.phase_4_volume_x10 = phase_4_volume_x10
        command.phase_4_mix_pc = phase_4_mix_pc
        command.phase_5_flow_rate_x10 = phase_5_flow_rate_x10
        command.phase_5_volume_x10 = phase_5_volume_x10
        command.phase_5_mix_pc = phase_5_mix_pc
        return self.mcu_transport.send_inject_arm(command)

    def inject_start(self):
        print("Executing injection_start")
        command = CommandMsg.INJECT_START_Command()
        return self.mcu_transport.send_inject_start(command)

    def air_reset(self):
        print("Executing air_reset")
        command = CommandMsg.AD_CLEAR_Command()
        command.motor_index = 2
        return self.mcu_transport.send_ad_clear(command)

    def reset(self):
        print("Executing reset")
        command = CommandMsg.RESET_Command()
        return self.mcu_transport.send_reset(command)

    def enable_debug(self, motor):
        command = CommandMsg.PIDDEBUG_Command()
        command.motor = motor
        return self.mcu_transport.send_piddebug(command)

    def move_fcm(self, left_fill_pos: int, left_patient_pos: int, right_fill_pos: int, right_patient_pos: int):
        # print("Executing move_fcm")
        if left_fill_pos == 1 and left_patient_pos == 1 and right_patient_pos == 1 and right_fill_pos == 1:
            print("Executing move_fcm HOME")
        elif left_fill_pos == 2 and left_patient_pos == 2 and right_patient_pos == 2 and right_fill_pos == 2:
            print("Executing move_fcm CLOSED")
        elif left_fill_pos == 3 and left_patient_pos == 3 and right_patient_pos == 3 and right_fill_pos == 3:
            print("Executing move_fcm OPEN")
        command = CommandMsg.SEND_FCM_MOVE_Command()
        command.left_fill_pos = left_fill_pos
        command.left_patient_pos = left_patient_pos
        command.right_fill_pos = right_fill_pos
        command.right_patient_pos = right_patient_pos
        command_msg = self.mcu_transport.send_send_fcm_move(command)
        # print(command_msg)
        return command_msg
        # return self.mcu_transport.send_send_fcm_move(command)

    def motor_is_active(self, motor_index: int) -> bool:
        if motor_index == 0:
            return self.last_digest_data.flush_syringe_state == MsgValues.SyringeState.PROCESSING
        elif motor_index == 1:
            return self.last_digest_data.contrast_syringe_state == MsgValues.SyringeState.PROCESSING
        else:
            return False

    def motor_is_engaged(self, motor_index: int):
        if motor_index == 0:
            return self.last_digest_data.flush_syringe_state == MsgValues.SyringeState.ENGAGED
        elif motor_index == 1:
            return self.last_digest_data.contrast_syringe_state == MsgValues.SyringeState.ENGAGED
        else:
            return False

    def get_pressure_adc_in_psi(self) -> float:
        pressure_adc = int(self.last_digest_data.pressure_adc)
        pressure_psi = pressure_adc / 4095.0 * (500.00 + 14.7) - 14.7  # 0 = -14.7 psi, 4095 = 500psi pressure_adc
        return pressure_psi

    def wait_until(self, flush: bool = True, contrast: bool = True, delay=0.01):
        waiting = True
        while waiting:
            self.update_digest(delay)
            if flush and self.motor_is_active(0):
                waiting = True
            elif contrast and self.motor_is_active(1):
                waiting = True
            else:
                break

    def pressure_ready(self, motor: int, volume):
        print("Executing pressure_ready")
        command = CommandMsg.PRESSURE_READY_Command()
        command.motor = motor
        command.volume = volume
        return self.mcu_transport.send_pressure_ready(command)

    def set_digest_csv_log(self, csv_filename: str) -> str:
        """Set the new log file and return the old log filename if available"""
        last_log_name = ""
        if self._digest_log:
            last_log_name = self._digest_log.name
            self._digest_log.close()
            self._digest_log = None

        if len(csv_filename) == 0:
            return last_log_name
        print("Previous digest log:", last_log_name)
        print("Digest logging", csv_filename)
        self._digest_log_start_time = time.time_ns()
        self._digest_log = open(csv_filename, "w")
        self._digest_log.write("T(ms),")
        self._digest_log.write(",".join(digest_headers))
        self._digest_log.write("\n")
        self._digest_log.flush()     # need to flush or the script stop running -python foo???? why
        return last_log_name

    def get_absolute_path(self, filename):
        """return the full path by prepending the output directory"""
        return os.path.join(self._output_dir, filename)

    def create_filename(self, name):
        out_name = os.path.join(self._output_dir, name)
        return out_name

    def close(self):
        self.stop_monitor_reading_thread()

        if self._monitor_serial:
            print("Closing the monitor port")
            self._monitor_serial.close()
            self._monitor_serial = None

        if self._main_serial:
            print("Closing the port")
            self._main_serial.close()
            self._main_serial = None
        if self._log_file:
            self._log_file.close()
            self._log_file = None

        if self._digest_log:
            self._digest_log.close()
            self._digest_log = None

    def ensure_plunger_engaged(self, motor: int, max_retry=3) -> bool:
        self.find_plunger(motor)

        self.wait_until()
        for i in range(max_retry):
            self.update_digest()
            if self.motor_is_engaged(motor):
                return True
            self.push2pull(motor)
            self.wait_until()

            if self.motor_is_engaged(motor):
                return True

            self.pull2push(motor)
            self.wait_until()
            if self.motor_is_engaged(motor):
                return True
        return False
