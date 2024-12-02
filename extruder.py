"""File to control the extrusion process"""
import time
import math
import RPi.GPIO as GPIO
import busio
import board
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

from database import Database
from user_interface import UserInterface
from collections import deque

class CircularBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = deque(maxlen = size)
        
    def add(self, item):
        self.buffer.append(item)
        
    def get_all(self):
        return list(self.buffer)

class Thermistor:
    """Constants and util functions for the thermistor"""
    REFERENCE_TEMPERATURE = 298.15 # K
    RESISTANCE_AT_REFERENCE = 100000 # Ω
    BETA_COEFFICIENT = 3977 # K
    VOLTAGE_SUPPLY = 3.3 # V
    RESISTOR = 10000 # Ω
    READINGS_TO_AVERAGE = 10

    @classmethod
    def get_temperature(cls, voltage: float) -> float:
        """Get the average temperature from the voltage using Steinhart-Hart 
        equation"""
        try:
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
        except Exception as e:
            print("0 division")
            print("Voltage: ", voltage)
            print("DB length: ", len(Database.temperature_readings))
            return 1

class Extruder:
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
        self.channel_0 = None
        self.buffer = CircularBuffer(8)
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

    def stepper_control_loop(self) -> float:
        """Move the stepper motor constantly"""
        try:
            setpoint_rpm = self.gui.extrusion_motor_speed.value()
            delay = (60 / setpoint_rpm / Extruder.STEPS_PER_REVOLUTION /
                    Extruder.FACTOR[Extruder.DEFAULT_MICROSTEPPING])
            GPIO.output(Extruder.DIRECTION_PIN, 1)
            GPIO.output(Extruder.STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(Extruder.STEP_PIN, GPIO.LOW)
            time.sleep(delay)
            Database.extruder_rpm.append(setpoint_rpm)
            return setpoint_rpm
        except Exception as e:
            print(f"Error in stepper control loop: {e}")
            self.gui.show_message("Error in stepper control loop",
                                    "Please restart the program.")

    def PWM_temperature_control(self, current_time: float) -> float:
        try:
            kp = self.gui.temperature_kp.value()
            ki = self.gui.temperature_ki.value() / 100
            kd = self.gui.temperature_kd.value()
            
            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            
            target_temp = self.gui.target_temperature.value()
            temp = Thermistor.get_temperature(self.channel_0.voltage)
        
            # Average of temperature readings
            self.buffer.add(temp)
            avg_temp = sum(self.buffer.get_all())/len(self.buffer.get_all())
        
            
            error = avg_temp - target_temp
                
            self.integral += error * delta_time
            
            derivative = (error - self.previous_error) / delta_time
            self.previous_error = error
            output = kp * error + ki * self.integral + kd * derivative

            out = -output//10 / 10
            if out > 1: out = 1
            
            
            self.gui.temperature_plot.update_plot(current_time, avg_temp, target_temp)
            return out
 
            
        except Exception as e:
            print(f"Error in PWM temperature control loop: {e}")
            self.gui.show_message("Error", "Error in PWM temperature control loop",
                                  "Please restart the program.")
                                  
    def turnON(self) -> None:
        GPIO.output(Extruder.HEATER_PIN, GPIO.HIGH)
        
    def turnOFF(self) -> None:
        GPIO.output(Extruder.HEATER_PIN, GPIO.LOW)
