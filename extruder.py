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

class Heater:
    """Class to control the heater via PWM"""

    def __init__(self, heater_pin: int) -> None:
        self.heater_pin = heater_pin

        GPIO.setup(self.heater_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.heater_pin, 1)  # 1kHz frequency
        self.pwm.start(0)

    def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the heater PWM"""
        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle > 100:
            duty_cycle = 100
        self.pwm.ChangeDutyCycle(duty_cycle)


class StepperMotor:
    """Class to control the stepper motor"""
    STEPS_PER_REVOLUTION = 200

    def __init__(self, step_pin: int, direction_pin: int, M0_pin: int, M1_pin: int, M2_pin: int) -> None:
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.M0_pin = M0_pin
        self.M1_pin = M1_pin
        self.M2_pin = M2_pin

        GPIO.setup(self.step_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.step_pin, 1000)  # Initial frequency
        self.pwm.start(0)

        GPIO.setup(self.direction_pin, GPIO.OUT)
        self.set_direction(False)

        GPIO.setup(self.M0_pin, GPIO.OUT)
        GPIO.output(self.M0_pin, False)
        GPIO.setup(self.M1_pin, GPIO.OUT)
        GPIO.output(self.M1_pin, False)
        GPIO.setup(self.M2_pin, GPIO.OUT)
        GPIO.output(self.M2_pin, False)

    def set_direction(self, clockwise: bool) -> None:
        """Set motor direction"""
        GPIO.output(self.direction_pin, not clockwise)

    def set_speed(self, rpm: float) -> None:
        """Set motor speed in RPM"""
        steps_per_second = (rpm * self.STEPS_PER_REVOLUTION) / 60
        frequency = steps_per_second   # Each cycle is two steps
        self.pwm.ChangeFrequency(frequency)
        self.pwm.ChangeDutyCycle(50)  # 50% duty cycle for full step


class Thermistor:
    """Constants and util functions for the thermistor"""
    REFERENCE_TEMPERATURE = 298.15 # K
    RESISTANCE_AT_REFERENCE = 100000 # Ω
    BETA_COEFFICIENT = 3977 # K
    VOLTAGE_SUPPLY = 3.3 # V
    RESISTOR = 100000 # Ω
    READINGS_TO_AVERAGE = 10

    def __init__(self):
        """Initialize the SPI for thermistor temperature readings"""
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) #intialize SPI bus
        cs = digitalio.DigitalInOut(board.D8) # Create the cs (chip select)
        mcp = MCP.MCP3008(spi, cs) # Create the mcp object
        self.channel_0 = AnalogIn(mcp, MCP.P0) # Create analog inputs connected to the input pins on the MCP3008

    def get_voltage(self) -> float:
        """Get the voltage from the thermistor"""
        return self.channel_0.voltage
     

    def get_temperature(self) -> float:
        """Get the average temperature from the voltage using Steinhart-Hart 
        equation"""
        voltage = self.get_voltage()
        if voltage < 0.0001 or voltage >= self.VOLTAGE_SUPPLY:  # Prevenir división por cero
            return 0
        
        resistance = ((self.VOLTAGE_SUPPLY - voltage) * self.RESISTOR )/ voltage
        ln = math.log(resistance / self.RESISTANCE_AT_REFERENCE)
        temperature = (1 / ((ln / self.BETA_COEFFICIENT) + (1 / self.REFERENCE_TEMPERATURE))) - 273.15

        return temperature

    # @classmethod
    # def get_temperature(cls, voltage: float) -> float:
    #     """Get the average temperature from the voltage using Steinhart-Hart 
    #     equation"""
    #     if voltage < 0.0001 or voltage >= cls.VOLTAGE_SUPPLY:  # Prevenir división por cero
    #         return 0
    #     resistance = ((cls.VOLTAGE_SUPPLY - voltage) * cls.RESISTOR )/ voltage
    #     ln = math.log(resistance / cls.RESISTANCE_AT_REFERENCE)
    #     temperature = (1 / ((ln / cls.BETA_COEFFICIENT) + (1 / cls.REFERENCE_TEMPERATURE))) - 273.15
    #     Database.temperature_readings.append(temperature)
    #     average_temperature = 0
    #     if len(Database.temperature_readings) > cls.READINGS_TO_AVERAGE:
    #         # Get last constant readings
    #         average_temperature = (sum(Database.temperature_readings
    #                                   [-cls.READINGS_TO_AVERAGE:]) /
    #                                   cls.READINGS_TO_AVERAGE)
    #     else:
    #         average_temperature = (sum(Database.temperature_readings) /
    #                                len(Database.temperature_readings))
    #     return average_temperature

class Extruder:
    """Controller of the extrusion process: the heater and stepper motor"""
    HEATER_PIN = 6
    DIRECTION_PIN = 16
    STEP_PIN = 20

    STEPS_PER_REVOLUTION = 200
    DEFAULT_RPM = 0.6 # TODO: Delay is not being used, will be removed temporarily
    SAMPLE_TIME = 0.1
    MAX_OUTPUT = 100
    MIN_OUTPUT = 0

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.speed = 0.0
        self.duty_cycle = 0.0
        # self.channel_0 = None
        
        self.heater = Heater(Extruder.HEATER_PIN)
        self.thermistor = Thermistor()
        self.stepper_motor = StepperMotor(Extruder.STEP_PIN, Extruder.DIRECTION_PIN, M0_pin=21, M1_pin=22, M2_pin=23)

        # Control parameters
        self.previous_time = 0.0
        self.previous_error = 0.0
        self.integral = 0.0


    def stepper_control_loop(self, current_time: float) -> None:
        """Control stepper motor speed"""
        try:
            setpoint_rpm = self.gui.extrusion_motor_speed.value()
            self.pwm.ChangeDutyCycle(0)
            if setpoint_rpm > 0.0:
                self.stepper_motor.set_speed(setpoint_rpm)
            Database.extruder_timestamps.append(current_time)
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

            temperature = self.thermistor.get_temperature()
            
            error = target_temperature - temperature
            self.integral += error * delta_time 
            derivative = (error - self.previous_error) / delta_time
            self.previous_error = error
            output = kp * error + ki * self.integral + kd * derivative
            if output > Extruder.MAX_OUTPUT:
                output = Extruder.MAX_OUTPUT
            elif output < Extruder.MIN_OUTPUT:
                output = Extruder.MIN_OUTPUT
            
            self.heater.set_duty_cycle(output)
            
            self.gui.temperature_plot.update_plot(current_time, temperature, target_temperature)
            
            Database.temperature_timestamps.append(current_time)
            Database.temperature_readings.append(temperature)
            # Database.temperature_movavg.append(temperature)
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
            
    
    # def temperature_open_loop_control(self, current_time: float) -> None:
    #     """Open loop PWM control of the heater"""
    #     if current_time - self.previous_time <= Extruder.SAMPLE_TIME:
    #         return
            
    #     try:
    #         pwm_value = self.gui.heater_open_loop_pwm.value()
    #         delta_time = current_time - self.previous_time
    #         self.previous_time = current_time
    #         temperature = Thermistor.get_temperature(self.channel_0.voltage)

    #         # Configurar PWM para el heater
    #         if not hasattr(self, 'heater_pwm'):
    #             self.heater_pwm = GPIO.PWM(Extruder.HEATER_PIN, 1)  # 1kHz frequency
    #             self.heater_pwm.start(0)

    #         # Actualizar duty cycle del PWM
    #         self.heater_pwm.ChangeDutyCycle(pwm_value)

    #         # Actualizar gráfica
    #         self.gui.temperature_plot.update_plot(current_time, temperature, 0)

    #         # Almacenar datos
    #         Database.temperature_timestamps.append(current_time)
    #         Database.temperature_delta_time.append(delta_time)
    #         Database.temperature_setpoint.append(0)  # No hay setpoint en lazo abierto
    #         Database.temperature_error.append(0)     # No hay error en lazo abierto
    #         Database.temperature_pid_output.append(pwm_value)
    #         Database.temperature_kp.append(0)
    #         Database.temperature_ki.append(0)
    #         Database.temperature_kd.append(0)

    #     except Exception as e:
    #         print(f"Error in temperature open loop control: {e}")
    #         self.gui.show_message("Error", "Error in temperature open loop control")
                 
