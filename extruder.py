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
        if voltage < 0.0001:  # Prevenir división por cero
            return 0
        resistance = ((cls.VOLTAGE_SUPPLY - voltage) * cls.RESISTOR )/ voltage
        ln = math.log(resistance / cls.RESISTANCE_AT_REFERENCE)
        temperature = (1 / ((ln / cls.BETA_COEFFICIENT) + (1 / cls.REFERENCE_TEMPERATURE))) - 273.15
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

class Extruder:
    """Controller of the extrusion process: the heater and stepper motor"""
    HEATER_PIN = 6
    DIRECTION_PIN = 16
    STEP_PIN = 12
    DEFAULT_DIAMETER = 0.35
    MINIMUM_DIAMETER = 0.3
    MAXIMUM_DIAMETER = 0.6
    STEPS_PER_REVOLUTION = 200
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
        self.set_motor_direction(False)
        # PWM Setup
        self.pwm = GPIO.PWM(Extruder.STEP_PIN, 1000)  # Initial 1000Hz
        self.pwm.start(0)  # 50% duty cycle
    
        self.initialize_thermistor()
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

    def set_motor_direction(self, clockwise: bool) -> None:
        """Set motor direction"""
        GPIO.output(Extruder.DIRECTION_PIN, not clockwise)

    def set_motor_speed(self, rpm: float) -> None:
        """Set motor speed in RPM"""
        steps_per_second = (rpm * Extruder.STEPS_PER_REVOLUTION) / 60
        frequency = steps_per_second * 2  # Each cycle is two steps
        self.pwm.ChangeFrequency(frequency)
        self.pwm.ChangeDutyCycle(50)

    def stepper_control_loop(self) -> None:
        """Control stepper motor speed"""
        try:
            setpoint_rpm = self.gui.extrusion_motor_speed.value()
            self.pwm.ChangeDutyCycle(0)
            if setpoint_rpm > 0.0:
                self.set_motor_speed(setpoint_rpm)
            Database.extruder_rpm.append(setpoint_rpm)
        except Exception as e:
            print(f"Error in stepper control loop: {e}")
            self.gui.show_message("Error", "Stepper control loop error")

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
            self.gui.temperature_plot.update_plot(current_time, temperature,target_temperature)
            
            Database.temperature_timestamps.append(current_time)
            Database.temperature_delta_time.append(delta_time)
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
            
    def temperature_on_and_off_control(self, current_time: float) -> None:
        """On/off control of the temperature"""
        if current_time - self.previous_time <= Extruder.SAMPLE_TIME:
            return
            
        try:
            target_temperature = self.gui.temperature_step.value()
            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            temperature = Thermistor.get_temperature(self.channel_0.voltage)

            # Control on/off
            if temperature < target_temperature:
                GPIO.output(Extruder.HEATER_PIN, GPIO.HIGH)
                heater_output = 1
            else:
                GPIO.output(Extruder.HEATER_PIN, GPIO.LOW)
                heater_output = 0

            # Update plot
            self.gui.temperature_plot.update_plot(current_time, temperature, target_temperature)

            # Store data
            Database.temperature_timestamps.append(current_time)
            Database.temperature_delta_time.append(delta_time)
            Database.temperature_setpoint.append(target_temperature)
            Database.temperature_error.append(target_temperature - temperature)
            Database.temperature_pid_output.append(heater_output)
            Database.temperature_kp.append(0)
            Database.temperature_ki.append(0)
            Database.temperature_kd.append(0)

        except Exception as e:
            print(f"Error in temperature on/off control: {e}")
            self.gui.show_message("Error", "Error in temperature on/off control")
                 
