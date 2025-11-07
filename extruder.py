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
    """Utility class for thermistor temperature calculation."""
    REFERENCE_TEMPERATURE = 298.15  # Kelvin
    RESISTANCE_AT_REFERENCE = 100000  # Ohms
    BETA_COEFFICIENT = 3977  # Kelvin
    VOLTAGE_SUPPLY = 3.3  # Volts
    RESISTOR = 100000  # Ohms
    READINGS_TO_AVERAGE = 10

    @classmethod
    def get_temperature(cls, voltage: float) -> float:
        """Convert voltage to temperature using the Steinhart-Hart equation."""
        if voltage < 0.0001 or voltage >= cls.VOLTAGE_SUPPLY:
            return 0
        resistance = ((cls.VOLTAGE_SUPPLY - voltage) * cls.RESISTOR) / voltage
        ln = math.log(resistance / cls.RESISTANCE_AT_REFERENCE)
        temperature = (1 / ((ln / cls.BETA_COEFFICIENT) + (1 / cls.REFERENCE_TEMPERATURE))) - 273.15
        Database.temperature_readings.append(temperature)
        # Use the average of the last N readings for stability
        if len(Database.temperature_readings) > cls.READINGS_TO_AVERAGE:
            average_temperature = sum(Database.temperature_readings[-cls.READINGS_TO_AVERAGE:]) / cls.READINGS_TO_AVERAGE
        else:
            average_temperature = sum(Database.temperature_readings) / len(Database.temperature_readings)
        return average_temperature

class Extruder:
    """Controls both the heater and stepper motor for extrusion."""
    HEATER_PIN = 6
    DIRECTION_PIN = 16
    STEP_PIN = 20
    STEPS_PER_REVOLUTION = 200
    SAMPLE_TIME = 0.1
    MAX_OUTPUT = 100
    MIN_OUTPUT = 0

    def __init__(self, gui: UserInterface) -> None:
        """Initialize GPIO, PWM, and thermistor interface."""
        self.gui = gui
        self.channel_0 = None

        GPIO.setup(Extruder.HEATER_PIN, GPIO.OUT)
        GPIO.setup(Extruder.DIRECTION_PIN, GPIO.OUT)
        GPIO.setup(Extruder.STEP_PIN, GPIO.OUT)

        self.set_motor_direction(False)
        self.pwm = GPIO.PWM(Extruder.STEP_PIN, 1000)
        self.pwm.start(0)
        self.heater_pwm = GPIO.PWM(Extruder.HEATER_PIN, 1)
        self.heater_pwm.start(0)

        self.initialize_thermistor()
        self.previous_time = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

    def initialize_thermistor(self):
        """Set up SPI and ADC for thermistor readings."""
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D8)
        mcp = MCP.MCP3008(spi, cs)
        self.channel_0 = AnalogIn(mcp, MCP.P0)

    def set_motor_direction(self, clockwise: bool) -> None:
        """Set the direction of the stepper motor."""
        GPIO.output(Extruder.DIRECTION_PIN, not clockwise)

    def set_motor_speed(self, rpm: float) -> None:
        """Set the speed of the stepper motor in RPM."""
        steps_per_second = (rpm * Extruder.STEPS_PER_REVOLUTION) / 60
        frequency = steps_per_second
        self.pwm.ChangeFrequency(frequency)
        self.pwm.ChangeDutyCycle(50)

    def stepper_control_loop(self) -> None:
        """
        Control the stepper motor.
        If device_started is True, run motor at fixed speed; otherwise, stop.
        """
        try:
            if self.gui.device_started:
                setpoint_rpm = 30.0  # Fixed RPM for extrusion
                self.set_motor_speed(setpoint_rpm)
                Database.extruder_rpm.append(setpoint_rpm)
            else:
                self.pwm.ChangeDutyCycle(0)
        except Exception as e:
            print(f"Error in stepper control loop: {e}")

    def temperature_control_loop(self, current_time: float) -> None:
        """
        Closed-loop control for the heater.
        If device_started is True, run PID control; otherwise, turn off heater.
        """
        if not self.gui.device_started:
            self.heater_pwm.ChangeDutyCycle(0)
            return

        if current_time - self.previous_time <= Extruder.SAMPLE_TIME:
            return

        try:
            target_temperature = 95.0  # Fixed setpoint
            kp = 1.4
            ki = 0.2
            kd = 0.8

            delta_time = current_time - self.previous_time
            self.previous_time = current_time

            temperature = Thermistor.get_temperature(self.channel_0.voltage)
            error = target_temperature - temperature

            self.integral += error * delta_time
            derivative = (error - self.previous_error) / delta_time
            self.previous_error = error

            output = kp * error + ki * self.integral + kd * derivative
            output = max(min(output, Extruder.MAX_OUTPUT), Extruder.MIN_OUTPUT)
            self.heater_pwm.ChangeDutyCycle(output)

            # Log data for analysis
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

    def stop(self):
        """Stop both motor and heater immediately."""
        self.pwm.ChangeDutyCycle(0)
        self.heater_pwm.ChangeDutyCycle(0)
