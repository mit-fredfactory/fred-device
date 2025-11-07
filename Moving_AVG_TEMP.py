# Test with moving average filter
# january 23rd, 2025

import sys
import time
import math
import RPi.GPIO as GPIO
import busio
import board
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QSlider, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from collections import deque

class Thermistor:
    REFERENCE_TEMPERATURE = 298.15
    RESISTANCE_AT_REFERENCE = 100000
    BETA_COEFFICIENT = 3977
    VOLTAGE_SUPPLY = 3.3
    RESISTOR = 10000

    @classmethod
    def get_temperature(cls, voltage):
        if voltage < 0.0001:
            return 0
        resistance = ((cls.VOLTAGE_SUPPLY - voltage) * cls.RESISTOR) / voltage
        ln = math.log(resistance / cls.RESISTANCE_AT_REFERENCE)
        temperature = (1 / ((ln / cls.BETA_COEFFICIENT) + (1 / cls.REFERENCE_TEMPERATURE))) - 273.15
        return temperature

class TemperatureController:
    HEATER_PIN = 6
    SAMPLE_TIME = 0.1

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.HEATER_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.HEATER_PIN, 1000)  # 1kHz frequency, cambio a 1
        self.pwm.start(0)
        self.channel_0 = self.initialize_thermistor()
        self.window_size = 10  # Number of samples for moving average
        self.temperature_buffer = deque(maxlen=self.window_size)

    def initialize_thermistor(self):
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D8)
        mcp = MCP.MCP3008(spi, cs)
        return AnalogIn(mcp, MCP.P0)

    def set_temperature(self, pwm_value):
        self.pwm.ChangeDutyCycle(pwm_value)

    def read_temperature(self):
        raw_temp = Thermistor.get_temperature(self.channel_0.voltage)
        self.temperature_buffer.append(raw_temp)
        return sum(self.temperature_buffer) / len(self.temperature_buffer)

class TemperatureApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.controller = TemperatureController()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Temperature Controller")
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        self.temp_label = QLabel("Current Temperature: 0.00 °C")
        layout.addWidget(self.temp_label)

        pwm_layout = QHBoxLayout()
        pwm_layout.addWidget(QLabel("PWM Value:"))
        self.pwm_slider = QSlider(Qt.Horizontal)
        self.pwm_slider.setRange(0, 100) 
        self.pwm_slider.valueChanged.connect(self.update_pwm)
        pwm_layout.addWidget(self.pwm_slider)
        self.pwm_value_label = QLabel("0")
        pwm_layout.addWidget(self.pwm_value_label)
        layout.addLayout(pwm_layout)

        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Temperature Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.times = []
        self.temperatures = []
        self.start_time = time.time()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_temperature)
        self.timer.start(100)  # Update every 100ms

    def update_temperature(self):
        current_temp = self.controller.read_temperature()
        current_time = time.time() - self.start_time

        self.temp_label.setText(f"Current Temperature: {current_temp:.2f} °C")

        self.times.append(current_time)
        self.temperatures.append(current_temp)
        self.ax.clear()
        self.ax.plot(self.times, self.temperatures)
        self.ax.set_title("Temperature Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.canvas.draw()

    def update_pwm(self, value):
        self.controller.set_temperature(value)
        self.pwm_value_label.setText(str(value))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TemperatureApp()
    window.show()
    sys.exit(app.exec_())