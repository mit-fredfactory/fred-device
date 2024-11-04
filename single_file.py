"""FrED Device Single File"""
import os
import sys
import threading
import cv2
import math
import time
import yaml
import csv
import busio
import board
import atexit
import datetime
import digitalio
import numpy as np
import RPi.GPIO as GPIO
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import warnings
warnings.filterwarnings("ignore")
from typing import Tuple
from typing import Self
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Qt5Agg')  # Use the Qt5Agg backend for matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QGridLayout, QWidget, QDoubleSpinBox, QPushButton, QMessageBox, QLineEdit, QCheckBox, QDesktopWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
from gpiozero import RotaryEncoder


class Database():
    """Class to store the raw data and generate the CSV file"""
    time_readings = []

    temperature_readings = []
    temperature_setpoint = []
    temperature_error = []
    temperature_pid_output = []
    temperature_kp = []
    temperature_ki = []
    temperature_kd = []
    extruder_rpm = []

    diameter_readings = []
    diameter_setpoint = []
    spooler_setpoint = []
    spooler_rpm = []
    spooler_gain = []
    spooler_oscilation_period = []

    fan_duty_cycle = []

    @classmethod
    def generate_csv(cls, filename: str) -> None:
        """Generate a CSV file with the data"""
        filename = filename + ".csv"
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Temperature", "Temperature Setpoint",
                                "Temperature Error", "Temperature PID Output",
                                "Temperature Kp", "Temperature Ki", "Temperature Kd",
                                "Extruder RPM", "Diameter", "Diameter Setpoint",
                                "Spooler Setpoint", "Spooler RPM", "Spooler Gain",
                                "Spooler Oscillation Period", "Fan Duty Cycle"])
            for i in enumerate(cls.time_readings):
                writer.writerow([cls.time_readings[i], cls.temperature_readings[i],
                                cls.temperature_setpoint[i], cls.temperature_error[i],
                                cls.temperature_pid_output[i], cls.temperature_kp[i],
                                cls.temperature_ki[i], cls.temperature_kd[i],
                                cls.extruder_rpm[i], cls.diameter_readings[i],
                                cls.diameter_setpoint[i], cls.spooler_setpoint[i],
                                cls.spooler_rpm[i], cls.spooler_gain[i],
                                cls.spooler_oscilation_period[i], cls.fan_duty_cycle[i]])
        print(f"CSV file {filename} generated.")

    @staticmethod
    def get_calibration_data(field: str) -> float:
        """Get calibration data from the yaml file"""
        with open("calibration.yaml", "r", encoding="utf-8") as file:
            calibration_data = yaml.unsafe_load(file)
        return calibration_data[field]

    @staticmethod
    def update_calibration_data(field: str, value: str) -> None:
        """Update calibration data in the yaml file"""
        with open("calibration.yaml", "r") as file:
            calibration_data = yaml.unsafe_load(file)
        with open("calibration.yaml", "w") as file:
            calibration_data[field] = float(value)
            yaml.dump(calibration_data, file)

class UserInterface():
    """"Graphical User Interface Class"""
    def __init__(self) -> None:
        self.app = QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()

        self.motor_plot, self.temperature_plot, self.diameter_plot \
            = self.add_plots()

        self.target_diameter, self.diameter_gain, \
            self.diameter_oscilation_period = self.add_diameter_controls()

        self.motor_gain, self.motor_oscilation_period, \
            self.extrusion_motor_speed = self.add_motor_controls()

        self.target_temperature_label, self.target_temperature, \
            self.temperature_kp, self.temperature_ki, self.temperature_kd \
            = self.add_temperature_controls()

        self.fan_duty_cycle_label, self.fan_duty_cycle = self.add_fan_controls()

        # Editable text box for the CSV file name
        self.csv_filename = QLineEdit()
        self.csv_filename.setText("Enter a file name")
        self.layout.addWidget(self.csv_filename, 24, 6)

        self.spooling_control_state = False
        self.device_started = False
        self.start_motor_calibration = False

        self.fiber_camera = FiberCamera()
        if self.fiber_camera.diameter_coefficient == -1:
            self.show_message("Camera calibration data not found",
                              "Please calibrate the camera.")
            self.fiber_camera.diameter_coefficient = 0.00782324
        self.layout.addWidget(self.fiber_camera.raw_image, 2, 8, 11, 1)
        self.layout.addWidget(self.fiber_camera.processed_image, 13, 8, 11, 1)

        self.add_buttons()
        
        self.window.setLayout(self.layout)
        self.window.setWindowTitle("MIT FrED")
        self.window.setGeometry(100, 100, 1600, 1000)
        self.window.setFixedSize(1600, 1000)
        self.window.setAutoFillBackground(True)
        
        # ~~~~~~~~~~~ Start threading the GUI and motor control ~~~~~~~~~~~    
        
        #motor_thread = threading.Thread(target=motor_control_thread)
        #motor_thread.start()
        #threading.Lock()
        
        # Begin application
        
        #motor_thread.join()

    def add_plots(self):
        """Add plots to the layout"""
        font_style = "font-size: 16px; font-weight: bold;"
        binary_checkbox = QCheckBox("Binary")
        binary_checkbox.setStyleSheet(font_style)
        #binary_checkbox.stateChanged.connect(checkbox_state_changed) TODO

        motor_plot = self.Plot("DC Spooling Motor", "Speed (RPM)")
        temperature_plot = self.Plot("Temperature", "Temperature (C)")
        diameter_plot = self.Plot("Diameter", "Diameter (mm)")

        self.layout.addWidget(binary_checkbox, 10, 1)
        self.layout.addWidget(diameter_plot, 2, 0, 8, 4)
        self.layout.addWidget(motor_plot, 11, 0, 8, 4)
        self.layout.addWidget(temperature_plot, 19, 0, 8, 4)

        return motor_plot, temperature_plot, diameter_plot

    def add_diameter_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox,
                                             QDoubleSpinBox]:
        """Add UI spin boxes to control the diameter"""
        font_style = "font-size: %ipx; font-weight: bold;"
        target_diameter_label = QLabel("Target Diameter (mm)")
        target_diameter_label.setStyleSheet(font_style % 16)
        target_diameter = QDoubleSpinBox()
        target_diameter.setMinimum(Extruder.MINIMUM_DIAMETER)
        target_diameter.setMaximum(Extruder.MAXIMUM_DIAMETER)
        target_diameter.setValue(Extruder.DEFAULT_DIAMETER)
        target_diameter.setSingleStep(0.01)
        target_diameter.setDecimals(2)

        diameter_gain_label = QLabel("Diameter Gain Ku")
        diameter_gain_label.setStyleSheet(font_style % 14)
        diameter_gain = QDoubleSpinBox()
        diameter_gain.setMinimum(0.1)
        diameter_gain.setMaximum(2)
        diameter_gain.setValue(1.2)
        diameter_gain.setSingleStep(0.1)
        diameter_gain.setDecimals(1)

        diameter_oscilation_period_label = QLabel("Diameter Oscillation Period Tu")
        diameter_oscilation_period_label.setStyleSheet(font_style % 14)
        diameter_oscilation_period = QDoubleSpinBox()
        diameter_oscilation_period.setMinimum(0.1)
        diameter_oscilation_period.setMaximum(2)
        diameter_oscilation_period.setValue(0.8)
        diameter_oscilation_period.setSingleStep(0.1)
        diameter_oscilation_period.setDecimals(1)

        self.layout.addWidget(target_diameter_label, 2, 6)
        self.layout.addWidget(target_diameter, 3,6)
        self.layout.addWidget(diameter_gain_label, 4, 6)
        self.layout.addWidget(diameter_gain, 5, 6)
        self.layout.addWidget(diameter_oscilation_period_label, 6, 6)
        self.layout.addWidget(diameter_oscilation_period, 7, 6)
        return target_diameter, diameter_gain, diameter_oscilation_period

    def add_motor_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox,
                                          QDoubleSpinBox]:
        """Add UI spin boxes to control the motors"""
        font_style = "font-size: %ipx; font-weight: bold;"
        motor_gain_label = QLabel("DC Motor Gain Ku")
        motor_gain_label.setStyleSheet(font_style % 14)
        motor_gain = QDoubleSpinBox()
        motor_gain.setMinimum(0.0)
        motor_gain.setMaximum(2.0)
        motor_gain.setValue(0.4)
        motor_gain.setSingleStep(0.1)
        motor_gain.setDecimals(1)

        motor_oscilation_period_label = QLabel("DC Motor Oscillation Period Tu")
        motor_oscilation_period_label.setStyleSheet(font_style % 14)
        motor_oscilation_period = QDoubleSpinBox()
        motor_oscilation_period.setMinimum(0.0)
        motor_oscilation_period.setMaximum(2.0)
        motor_oscilation_period.setValue(0.9)
        motor_oscilation_period.setSingleStep(0.1)
        motor_oscilation_period.setDecimals(1)

        extrusion_motor_speed_label = QLabel("Extrusion Motor Speed (RPM)")
        extrusion_motor_speed_label.setStyleSheet(font_style % 16)
        extrusion_motor_speed = QDoubleSpinBox()
        extrusion_motor_speed.setMinimum(0.0)
        extrusion_motor_speed.setMaximum(2.0)
        extrusion_motor_speed.setValue(1.2)
        extrusion_motor_speed.setSingleStep(0.1)
        extrusion_motor_speed.setDecimals(1)

        self.layout.addWidget(motor_gain_label, 8, 6)
        self.layout.addWidget(motor_gain, 9, 6)
        self.layout.addWidget(motor_oscilation_period_label, 10, 6)
        self.layout.addWidget(motor_oscilation_period, 11, 6)
        self.layout.addWidget(extrusion_motor_speed_label, 12, 6)
        self.layout.addWidget(extrusion_motor_speed, 13, 6)
        return motor_gain, motor_oscilation_period, extrusion_motor_speed

    def add_temperature_controls(self) -> Tuple[QLabel, QSlider, QDoubleSpinBox,
                                                  QDoubleSpinBox, QDoubleSpinBox]:
        """Add UI controls for the temperature"""
        font_style = "font-size: %ipx; font-weight: bold;"
        target_temperature_label = QLabel("Temperature (C)")
        target_temperature_label.setStyleSheet(font_style % 16)
        target_temperature = QSlider(Qt.Horizontal)
        target_temperature.setMinimum(65)
        target_temperature.setMaximum(105)
        target_temperature.setValue(95)
        target_temperature.valueChanged.connect(self.update_temperature_slider_label)

        temperature_kp_label = QLabel("Temperature Kp")
        temperature_kp_label.setStyleSheet(font_style % 14)
        temperature_kp = QDoubleSpinBox()
        temperature_kp.setMinimum(0.0)
        temperature_kp.setMaximum(2.0)
        temperature_kp.setValue(1.4)
        temperature_kp.setSingleStep(0.1)
        temperature_kp.setDecimals(1)

        temperature_ki_label = QLabel("Temperature Ki")
        temperature_ki_label.setStyleSheet(font_style % 14)
        temperature_ki = QDoubleSpinBox()
        temperature_ki.setMinimum(0.0)
        temperature_ki.setMaximum(2.0)
        temperature_ki.setValue(0.2)
        temperature_ki.setSingleStep(0.1)
        temperature_ki.setDecimals(1)

        temperature_kd_label = QLabel("Temperature Kd")
        temperature_kd_label.setStyleSheet(font_style % 14)
        temperature_kd = QDoubleSpinBox()
        temperature_kd.setMinimum(0.0)
        temperature_kd.setMaximum(2.0)
        temperature_kd.setValue(0.8)
        temperature_kd.setSingleStep(0.1)
        temperature_kd.setDecimals(1)

        self.layout.addWidget(target_temperature_label, 14, 6)
        self.layout.addWidget(target_temperature, 15, 6)
        self.layout.addWidget(temperature_kp_label, 16, 6)
        self.layout.addWidget(temperature_kp, 17, 6)
        self.layout.addWidget(temperature_ki_label, 18, 6)
        self.layout.addWidget(temperature_ki, 19, 6)
        self.layout.addWidget(temperature_kd_label, 20, 6)
        self.layout.addWidget(temperature_kd, 21, 6)

        return target_temperature_label, target_temperature, temperature_kp, \
            temperature_ki, temperature_kd

    def add_fan_controls(self) -> Tuple[QLabel, QSlider]:
        """Add UI controls for the fan"""
        font_style = "font-size: %ipx; font-weight: bold;"
        fan_duty_cycle_label = QLabel("Fan Duty Cycle (%)")
        fan_duty_cycle_label.setStyleSheet(font_style % 14)
        fan_duty_cycle = QSlider(Qt.Horizontal)
        fan_duty_cycle.setMinimum(0)
        fan_duty_cycle.setMaximum(100)
        fan_duty_cycle.setValue(30)
        fan_duty_cycle.valueChanged.connect(self.update_fan_slider_label)

        self.layout.addWidget(fan_duty_cycle_label, 22, 6)
        self.layout.addWidget(fan_duty_cycle, 23, 6)

        return fan_duty_cycle_label, fan_duty_cycle

    def add_buttons(self):
        """Add buttons to the layout"""
        font_style = "background-color: green; font-size: 14px; font-weight: bold;"
        spooling_control = QPushButton("Start/stop spooling close loop control")
        spooling_control.setStyleSheet(font_style)
        spooling_control.clicked.connect(self.spooling_control_toggle)
        start_device = QPushButton("Start device")
        start_device.setStyleSheet(font_style)
        start_device.clicked.connect(self.set_start_device)
        calibrate_motor = QPushButton("Calibrate motor")
        calibrate_motor.setStyleSheet(font_style)
        calibrate_motor.clicked.connect(self.set_calibrate_motor)
        calibrate_camera = QPushButton("Calibrate camera")
        calibrate_camera.setStyleSheet(font_style)
        calibrate_camera.clicked.connect(self.set_calibrate_camera)
        download_csv = QPushButton("Download CSV File")
        download_csv.setStyleSheet(font_style)
        download_csv.clicked.connect(self.set_download_csv)

        self.layout.addWidget(spooling_control, 10, 0)
        self.layout.addWidget(start_device, 1, 0)
        self.layout.addWidget(calibrate_motor, 1, 1)
        self.layout.addWidget(calibrate_camera, 1, 2)
        self.layout.addWidget(download_csv, 24, 3)

    def start_gui(self) -> None:
        """Start the GUI"""
        timer = QTimer()
        timer.timeout.connect(self.fiber_camera.camera_loop)
        timer.start(100)

        self.window.show()
        self.app.exec_()

    def update_temperature_slider_label(self, value) -> None:
        """Update the temperature slider label"""
        self.target_temperature_label.setText(f"Temperature: {value} C")
    
    def update_fan_slider_label(self, value) -> None:
        """Update the fan slider label"""
        self.fan_duty_cycle_label.setText(f"Fan Duty Cycle: {value} %")

    def spooling_control_toggle(self) -> None:
        """Toggle the spooling control"""
        self.spooling_control_state = not self.spooling_control_state
        if self.spooling_control_state:
            QMessageBox.information(self.app.activeWindow(), "Spooling Control",
                                    "Spooling control started.")
        else:
            QMessageBox.information(self.app.activeWindow(), "Spooling Control",
                                    "Spooling control stopped.")

    def set_start_device(self) -> None:
        """Set start device flag"""
        QMessageBox.information(self.app.activeWindow(), "Device Start",
                                "Device is starting.")
        self.device_started = True

    def set_calibrate_motor(self) -> None:
        """Set calibrate motor flag"""
        QMessageBox.information(self.app.activeWindow(), "Motor Calibration",
                                "Motor is calibrating.")
        self.start_motor_calibration = True

    def set_calibrate_camera(self) -> None:
        """Call calibrate camera"""
        QMessageBox.information(self.app.activeWindow(), "Camera Calibration",
                                "Camera is calibrating.")
        self.fiber_camera.calibrate()
        QMessageBox.information(self.app.activeWindow(),
                                "Calibration", "Camera calibration completed. "
                                "Please restart the program.")    

    def set_download_csv(self) -> None:
        """Call download csv from database"""
        QMessageBox.information(self.app.activeWindow(), "Download CSV",
                                "Downloading CSV file.")
        Database.generate_csv(self.csv_filename.text())

    def show_message(self, title: str, message: str) -> None:
        """Show a message box"""
        QMessageBox.information(self.app.activeWindow(), title, message)

    class Plot(FigureCanvas):
        """Base class for plots"""
        def __init__(self, title: str, y_label: str) -> None:
            self.figure = Figure()
            self.axes = self.figure.add_subplot(111)
            # 1x1 grid, first subplot: https://stackoverflow.com/a/46986694
            super(UserInterface.Plot, self).__init__(self.figure)

            self.axes.set_title(title)
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel(y_label)

            self.progress_line, = self.axes.plot([], [], lw=2, label=title)
            self.setpoint_line, = self.axes.plot([], [], lw=2, color='r',
                                                 label=f'Target {title}')
            self.axes.legend()

            self.x_data = []
            self.y_data = []
            self.setpoint_data = []

        def update_plot(self, x: float, y: float, setpoint: float) -> None:
            """Update the plot"""
            self.x_data.append(x)
            self.y_data.append(y)
            self.setpoint_data.append(setpoint)

            self.progress_line.set_data(self.x_data, self.y_data)
            self.setpoint_line.set_data(self.x_data, self.setpoint_data)

            self.axes.relim()
            self.axes.autoscale_view()
            self.draw()

class Thermistor:
    """Constants and util functions for the thermistor"""
    REFERENCE_TEMPERATURE = 298.15 # K
    RESISTANCE_AT_REFERENCE = 100000 # Î©
    BETA_COEFFICIENT = 3977 # K
    VOLTAGE_SUPPLY = 3.3 # V
    RESISTOR = 10000 # Î©
    READINGS_TO_AVERAGE = 10

    @classmethod
    def get_temperature(cls, voltage: float) -> float:
        """Get the average temperature from the voltage using Steinhart-Hart 
        equation"""
        resistance = (cls.VOLTAGE_SUPPLY / voltage) * cls.RESISTOR / voltage
        ln = math.log(resistance / cls.RESISTANCE_AT_REFERENCE)
        temperature = (1 / ((ln / cls.BETA_COEFFICIENT) +
                     (1 / cls.REFERENCE_TEMPERATURE))) - 273.15
        Database.temperature_readings.append(temperature)
        average_temperature = 0
        if len(Database.temperature_readings) > cls.READINGS_TO_AVERAGE:
            # Get last constant readings 
            average_temperature = (sum(Database.temperature_readings
                                      [-cls.READINGS_TO_AVERAGE:]) / 
                                      cls.READINGS_TO_AVERAGE)
        else:
            average_temperature = (sum(Database.temperature_readings) /
                                   len(Database.temperature_readings))
        return average_temperature


class Extruder():
    """Controller of the extrusion process: the heater and stepper motor"""
    HEATER_PIN = 6
    DIRECTION_PIN = 16
    STEP_PIN = 12
    MICROSTEP_PIN_A = 17
    MICROSTEP_PIN_B = 27
    MICROSTEP_PIN_C = 22
    DEFAULT_DIAMETER = 0.35
    MINIMUM_DIAMETER = 0.3
    MAXIMUM_DIAMETER = 0.6
    STEPS_PER_REVOLUTION = 200
    RESOLUTION = {'1': (0, 0, 0),
                  '1/2': (1, 0, 0),
                  '1/4': (0, 1, 0),
                  '1/8': (1, 1, 0),
                 '1/16': (0, 0, 1),
                 '1/32': (1, 0, 1)}
    FACTOR = {'1': 1,
                   '1/2': 2,
                   '1/4': 4,
                   '1/8': 8,
                   '1/16': 16,
                   '1/32': 32}
    DEFAULT_MICROSTEPPING = '1/4'
    DEFAULT_RPM = 0.6 # TODO: Delay is not being used, will be removed temporarily
    SAMPLE_TIME = 0.1
    MAX_OUTPUT = 1
    MIN_OUTPUT = 0
    
    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.speed = 0.0
        self.duty_cycle = 0.0
        self.channel_0 = None
        GPIO.setup(Extruder.HEATER_PIN, GPIO.OUT)
        GPIO.setup(Extruder.DIRECTION_PIN, GPIO.OUT)
        GPIO.setup(Extruder.STEP_PIN, GPIO.OUT)
        GPIO.setup(Extruder.MICROSTEP_PIN_A, GPIO.OUT)
        GPIO.setup(Extruder.MICROSTEP_PIN_B, GPIO.OUT)
        GPIO.setup(Extruder.MICROSTEP_PIN_C, GPIO.OUT)

        self.motor_step(0)
        self.initialize_thermistor()
        self.set_microstepping(Extruder.DEFAULT_MICROSTEPPING)

        self.current_diameter = 0.0
        self.diameter_setpoint = Extruder.DEFAULT_DIAMETER
        
        # Control parameters
        self.previous_time = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

    def initialize_thermistor(self):
        """Initialize the SPI for thermistor temperature readings"""
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

        # Create the cs (chip select)
        cs = digitalio.DigitalInOut(board.D8)

        # Create the mcp object
        mcp = MCP.MCP3008(spi, cs)

        # Create analog inputs connected to the input pins on the MCP3008
        self.channel_0 = AnalogIn(mcp, MCP.P0)

    def set_microstepping(self, mode: str) -> None:
        """Set the microstepping mode"""
        GPIO.output(Extruder.MICROSTEP_PIN_A, Extruder.RESOLUTION[mode][0])
        GPIO.output(Extruder.MICROSTEP_PIN_B, Extruder.RESOLUTION[mode][1])
        GPIO.output(Extruder.MICROSTEP_PIN_C, Extruder.RESOLUTION[mode][2])

    def motor_step(self, direction: int) -> None:
        """Step the motor in the given direction"""
        GPIO.output(Extruder.DIRECTION_PIN, direction)

    def stepper_control_loop(self) -> None:
        """Move the stepper motor constantly"""
        setpoint_rpm = self.gui.extrusion_motor_speed.value()
        delay = (60 / setpoint_rpm / Extruder.STEPS_PER_REVOLUTION /
                 Extruder.FACTOR[Extruder.DEFAULT_MICROSTEPPING])
        GPIO.output(Extruder.DIRECTION_PIN, 1)
        GPIO.output(Extruder.STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(Extruder.STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        Database.extruder_rpm.append(setpoint_rpm)

    def temperature_control_loop(self, current_time: float) -> None:
        """Closed loop control of the temperature of the extruder for desired diameter"""
        if current_time - self.previous_time <= Extruder.SAMPLE_TIME:
            return
        try:
            target_temperature = self.gui.target_temperature.value()
            kp = self.gui.temperature_kp.value()
            ki = self.gui.temperature_ki.value()
            kd = self.gui.temperature_kd.value()

            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            temperature = Thermistor.get_temperature(self.channel_0.voltage)
            error = target_temperature - temperature
            self.integral += error * delta_time
            derivative = (error - self.previous_error) / delta_time
            self.previous_error = error
            output = kp * error + ki * self.integral + kd * derivative
            if output > Extruder.MAX_OUTPUT:
                output = Extruder.MAX_OUTPUT
            elif output < Extruder.MIN_OUTPUT:
                output = Extruder.MIN_OUTPUT
            GPIO.output(Extruder.HEATER_PIN,
                        GPIO.HIGH if output > 0 else GPIO.LOW)
            self.gui.temperature_plot.update_plot(current_time, temperature,
                                                    target_temperature)
            Database.temperature_setpoint.append(target_temperature)
            Database.temperature_error.append(error)
            Database.temperature_pid_output.append(output)
            Database.temperature_kp.append(kp)
            Database.temperature_ki.append(ki)
            Database.temperature_kd.append(kd)
        except Exception as e:
            print(f"Error in temperature control loop: {e}")
            self.gui.show_message("Error", "Error in temperature control loop",
                                  "Please restart the program.")
            

class Spooler():
    """DC Motor Controller for the spooling process"""
    ENCODER_A_PIN = 24
    ENCODER_B_PIN = 23
    PWM_PIN = 5

    PULSES_PER_REVOLUTION = 1176
    READINGS_TO_AVERAGE = 10
    SAMPLE_TIME = 0.1
    DIAMETER_PREFORM = 7
    DIAMETER_SPOOL = 15.2

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.encoder = None
        self.pwm = None
        self.slope = Database.get_calibration_data("motor_slope")
        self.intercept = Database.get_calibration_data("motor_intercept")
        if self.slope == -1 or self.intercept == -1:
            self.gui.show_message( "Motor calibration data not found",
                                    "Please calibrate the motor.")
        GPIO.setup(Spooler.PWM_PIN, GPIO.OUT)
        self.initialize_encoder()
        
        # Control parameters
        self.previous_time = 0.0
        self.integral_diameter = 0.0
        self.previous_error_diameter = 0.0
        self.previous_steps = 0
        self.integral_motor = 0.0
        self.previous_error_motor = 0.0

    def initialize_encoder(self) -> None:
        """Initialize the encoder and SPI"""
        self.encoder = RotaryEncoder(Spooler.ENCODER_A_PIN,
                                     Spooler.ENCODER_B_PIN, max_steps=0)

    def start(self, frequency: float, duty_cycle: float) -> None:
        """Start the DC Motor PWM"""
        self.pwm = GPIO.PWM(Spooler.PWM_PIN, frequency)
        self.pwm.start(duty_cycle)

    def stop(self) -> None:
        """Stop the DC Motor PWM"""
        if self.pwm:
            self.pwm.stop()

    def update_duty_cycle(self, duty_cycle: float) -> None:
        """Update the DC Motor PWM duty cycle"""
        self.pwm.ChangeDutyCycle(duty_cycle)

    def get_average_diameter(self) -> float:
        """Get the average diameter of the fiber"""
        if len(Database.diameter_readings) < Spooler.READINGS_TO_AVERAGE:
            return (sum(Database.diameter_readings) /
                    len(Database.diameter_readings))
        else:
            return (sum(Database.diameter_readings[-Spooler.READINGS_TO_AVERAGE:])
                    / Spooler.READINGS_TO_AVERAGE)

    def diameter_to_rpm(self, diameter: float) -> float:
        """Convert the fiber diameter to RPM of the spooling motor"""
        stepper_rpm = self.gui.extrusion_motor_speed.value()
        return 25/28 * 11 * stepper_rpm * (Spooler.DIAMETER_PREFORM**2 /
                                        (Spooler.DIAMETER_SPOOL * diameter**2))

    def rpm_to_duty_cycle(self, rpm: float) -> float:
        """Convert the RPM to duty cycle"""
        return self.slope * rpm + self.intercept

    def motor_control_loop(self, current_time: float) -> None:
        """Closed loop control of the DC motor for desired diameter"""
        if current_time - self.previous_time <= Spooler.SAMPLE_TIME:
            return
        try:
            target_diameter = self.gui.target_diameter.value()
            current_diameter = self.get_average_diameter()

            diameter_ku = self.gui.diameter_gain.value()
            diameter_tu = self.gui.diameter_oscilation_period.value()
            diameter_kp = 0.6 * diameter_ku
            diameter_ti = diameter_tu / 2
            diameter_td = diameter_tu / 8
            diameter_ki = diameter_kp / diameter_ti
            diameter_kd = diameter_kp * diameter_td

            motor_ku = self.gui.motor_gain.value()
            motor_tu = self.gui.motor_oscilation_period.value()
            motor_kp = 0.6 * motor_ku
            motor_ti = motor_tu / 2
            motor_td = motor_tu / 8
            motor_ki = motor_kp / motor_ti
            motor_kd = motor_kp * motor_td

            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            error = target_diameter - current_diameter
            self.integral_diameter += error * delta_time
            self.integral_diameter = max(min(self.integral_diameter, 0.5), -0.5)
            derivative = (error - self.previous_error_diameter) / delta_time
            self.previous_error_diameter = error
            output = (diameter_kp * error + diameter_ki * self.integral_diameter
                      + diameter_kd * derivative)
            setpoint_rpm = self.diameter_to_rpm(target_diameter)
            setpoint_rpm = max(min(setpoint_rpm, 0), 60)

            # Control the motor
            delta_steps = self.encoder.steps - self.previous_steps
            self.previous_steps = self.encoder.steps
            current_rpm = (delta_steps / Spooler.PULSES_PER_REVOLUTION * 
                           60 / delta_time)
            error = setpoint_rpm - current_rpm
            self.integral_motor += error * delta_time
            self.integral_motor = max(min(self.integral_motor, 100), -100)
            derivative = (error - self.previous_error_motor) / delta_time
            self.previous_error_motor = error
            output = (motor_kp * error + motor_ki * self.integral_motor +
                        motor_kd * derivative)
            output_duty_cycle = self.rpm_to_duty_cycle(output) 
            output_duty_cycle = max(min(output_duty_cycle, 100), 0)
            self.update_duty_cycle(output_duty_cycle)

            # Update plots
            self.gui.motor_plot.update_plot(current_time, current_rpm,
                                            setpoint_rpm)
            self.gui.diameter_plot.update_plot(current_time, current_diameter,
                                                  target_diameter)

            # Add data to the database
            Database.diameter_setpoint.append(target_diameter)
            Database.spooler_setpoint.append(setpoint_rpm)
            Database.spooler_rpm.append(current_rpm)
            Database.spooler_gain.append(diameter_ku)
            Database.spooler_oscilation_period.append(diameter_tu)
        except Exception as e:
            print(f"Error in motor control loop: {e}")
            self.gui.show_message("Error", "Error in motor control loop",
                                  "Please restart the program.")

    def calibrate(self) -> None:
        """Calibrate the DC Motor"""
        rpm_values = []
        duty_cycles = []
        num_samples = 5 

        try:
            for duty_cycle in range(20, 101, 10):  # Sweep duty cycle from 0% to 100% in increments of 10%
                rpm_samples = []
                for _ in range(num_samples):
                    self.update_duty_cycle(duty_cycle)
                    time.sleep(2)
                    # Measure RPM
                    oldtime = time.perf_counter()
                    oldpos = self.encoder.steps
                    time.sleep(Spooler.SAMPLE_TIME)
                    newtime = time.perf_counter()
                    newpos = self.encoder.steps
                    dt = newtime - oldtime
                    ds = newpos - oldpos
                    rpm = ds / Spooler.PULSES_PER_REVOLUTION / dt * 60
                    rpm_samples.append(rpm)
                avg_rpm = sum(rpm_samples) / num_samples
                duty_cycles.append(duty_cycle)
                rpm_values.append(avg_rpm)
                print(f"Duty Cycle: {duty_cycle}% -> Avg RPM: {avg_rpm:.2f}")

            # Fit a curve to the data
            coefficients = np.polyfit(rpm_values, duty_cycles, 1)
            self.slope = coefficients[0]
            self.intercept = coefficients[1]
            Database.update_calibration_data("motor_slope", str(self.slope))
            Database.update_calibration_data("motor_intercept", str(self.intercept))

        except KeyboardInterrupt:
            print("\nData collection stopped\n\n")

        self.gui.show_message("Calibration", "Motor calibration completed. "
                               "Please restart the program.")
        self.stop()
        self.previous_steps = self.encoder.steps

class Fan():
    """Controller for the fan"""
    PIN = 13
    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.duty_cycle = 0.0
        self.pwm = None
        GPIO.setup(Fan.PIN, GPIO.OUT)

    def start(self, frequency: float, duty_cycle: float) -> None:
        """Start the fan PWM"""
        self.pwm = GPIO.PWM(Fan.FAN_PIN, frequency)
        self.pwm.start(duty_cycle)

    def stop(self) -> None:
        """Stop the fan PWM"""
        if self.pwm:
            self.pwm.stop()

    def update_duty_cycle(self, duty_cycle: float) -> None:
        """Update speed"""
        self.pwm.ChangeDutyCycle(duty_cycle)

    def control_loop(self) -> None:
        """Set the desired speed"""
        self.update_duty_cycle(self.gui.fan_duty_cycle.value())

class FiberCamera(QWidget):
    """Proceess video from camera to obtain the fiber diameter and display it"""
    use_binary_for_edges = True
    def __init__(self) -> None:
        super().__init__()
        self.raw_image = QLabel()
        self.processed_image = QLabel()
        self.capture = cv2.VideoCapture(0)
        self.line_value_updated = pyqtSignal(float)  # Create a new signal
        self.diameter_coefficient = Database.get_calibration_data(
            "diameter_coefficient")

    def camera_loop(self) -> None:
        """Loop to capture and process frames from the camera"""
        success, frame = self.capture.read()
        assert success, "Failed to capture frame"  # Check if frame is captured

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # To RGB for GUI
        height, _, _ = frame.shape
        frame = frame[height//4:3*height//4, :]  # Keep the middle section
        edges, binary_frame = self.get_edges(frame)
        # Get diameter from the binary image
        # TODO: Tune and set to constants for fiber line detection
        detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50,
                                         minLineLength=80, maxLineGap=20)
        fiber_diameter = self.get_fiber_diameter(detected_lines)
        # Plot lines on the frame
        frame = self.plot_lines(frame, detected_lines)
        # Emit the line_value_updated signal with the new line_value
        #self.line_value_updated.emit(line_value)
        Database.diameter_readings.append(fiber_diameter)

        # Display the frame with lines
        image_for_gui = QImage(frame, frame.shape[1], frame.shape[0],
                                QImage.Format_RGB888)
        self.raw_image.setPixmap(QPixmap(image_for_gui))

        # Binary Image
        image_for_gui = QImage(binary_frame, binary_frame.shape[1],
                               binary_frame.shape[0], QImage.Format_Grayscale8)
        self.processed_image.setPixmap(QPixmap(image_for_gui))

    def get_edges(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Filter the frame to enhance the edges"""
        # Divide the image into 4 horiizontal sections, and keep the middle section
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gaussian_blurred = cv2.GaussianBlur(gray_frame, (5, 5), 0) 
        threshold_value, binary_frame = cv2.threshold(
            gaussian_blurred, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #print(f'Threshold value: {threshold_value}')

        if FiberCamera.use_binary_for_edges is False:
            edges = cv2.Canny(gray_frame, 100, 250, apertureSize=3)
        else:
            edges = cv2.Canny(binary_frame, 100, 250, apertureSize=3)
        return edges, binary_frame

    def get_fiber_diameter(self, lines):
        """Get the fiber diameter from the edges detected in the image"""
        leftmost_min = sys.maxsize
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0
        if lines is None or len(lines) <= 1:
            return 0
        for line in lines:
            x0, _, x1, _ = line[0]
            # Find if local leftmost is less than the previous leftmost 
            leftmost_min = min(leftmost_min, x0, x1)
            leftmost_max = max(leftmost_max, min(x0, x1))
            rightmost_min = min(rightmost_min, max(x0, x1))
            rightmost_max = max(rightmost_max, x0, x1)

        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min))
                / 2 * self.diameter_coefficient )

    def plot_lines(self, frame, lines):
        """Plot the detected lines on the frame"""
        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
        return frame

    def calibrate(self):
        """Calibrate the camera"""
        num_samples = 20
        accumulated_diameter = 0
        average_diameter = 0
        valid_samples = 0

        for _ in range(num_samples):
            success, frame = self.capture.read()
            assert success, "Failed to capture frame"  # Check if frame is captured
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            edges, _ = self.get_edges(frame)
            detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 80,
                                         minLineLength=60, maxLineGap=10)
            fiber_diameter = self.get_fiber_diameter(detected_lines)
            if fiber_diameter is not None:
                accumulated_diameter += fiber_diameter
                valid_samples += 1

        if valid_samples > 0:
            average_diameter = accumulated_diameter / valid_samples
        
        print(f"Average width of wire: {average_diameter} mm")

        self.diameter_coefficient = 0.45/average_diameter
        print(f"Diameter_coeff: {self.diameter_coefficient} mm")

        Database.update_calibration_data("diameter_coefficient", 
                                         str(self.diameter_coefficient))

    def closeEvent(self, event):
        """Close the camera when the window is closed"""
        self.cap.release()
        event.accept()


def hardware_control(gui: UserInterface) -> None:
    """Thread to handle hardware control"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    fan = Fan(gui)
    spooler = Spooler(gui)
    extruder = Extruder(gui)
    fan.start(1000, 45)
    spooler.start(1000, 0)

    init_time = time.time()
    while True:
        current_time = time.time() - init_time
        if gui.start_motor_calibration:
            spooler.calibrate()
            gui.start_motor_calibration = False
        if gui.device_started:
            extruder.temperature_control_loop(current_time)
            extruder.stepper_control_loop()
            if gui.spooling_control_state:
                spooler.motor_control_loop(current_time)
            fan.control_loop()
        time.sleep(0.1)

if __name__ == "__main__":
    print("Starting FrED Device...")
    ui = UserInterface()
    hardware_thread = threading.Thread(target=hardware_control, args=(ui,))
    hardware_thread.start()
    threading.Lock()
    ui.start_gui()
    print("FrED Device Closed.")
