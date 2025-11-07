"""File to control the spooling process"""
import time
import numpy as np
import RPi.GPIO as GPIO
import spidev

from database import Database
from user_interface import UserInterface

class Spooler:
    """DC Motor Controller for the spooling process"""
    SLAVE_SELECT_ENC = 26
    PWM_PIN = 5

    PULSES_PER_REVOLUTION = 4704  # Updated for the new encoder
    READINGS_TO_AVERAGE = 10
    SAMPLE_TIME = 0.1
    DIAMETER_PREFORM = 7
    DIAMETER_SPOOL = 15.2

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.pwm = None
        self.slope = Database.get_calibration_data("motor_slope")
        self.intercept = Database.get_calibration_data("motor_intercept")
        self.motor_calibration = True
        if self.slope == -1 or self.intercept == -1:
            self.motor_calibration = False

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Spooler.SLAVE_SELECT_ENC, GPIO.OUT)
        GPIO.setup(Spooler.PWM_PIN, GPIO.OUT)
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.HIGH)

        self.initialize_encoder()

        # Control parameters
        self.previous_time = 0.0
        self.integral_diameter = 0.0
        self.previous_error_diameter = 0.0
        self.previous_position = 0
        self.integral_motor = 0.0
        self.previous_error_motor = 0.0

    def initialize_encoder(self) -> None:
        """Initialize the encoder and SPI"""
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 50000

        # Initialize encoder
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.LOW)
        self.spi.xfer2([0x88, 0x03])
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.HIGH)

        # Clear encoder count
        self.clear_encoder_count()

    def clear_encoder_count(self) -> None:
        """Clear the encoder count"""
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.LOW)
        self.spi.xfer2([0x98, 0x00, 0x00, 0x00, 0x00])
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.HIGH)

        time.sleep(0.0001)

        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.LOW)
        self.spi.xfer2([0xE0])
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.HIGH)

    def read_encoder(self) -> int:
        """Read the encoder position"""
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.LOW)
        self.spi.xfer2([0x60])
        count_1 = self.spi.xfer2([0x00])
        count_2 = self.spi.xfer2([0x00])
        count_3 = self.spi.xfer2([0x00])
        count_4 = self.spi.xfer2([0x00])
        GPIO.output(Spooler.SLAVE_SELECT_ENC, GPIO.HIGH)

        count_value = (count_1[0] << 24) + (count_2[0] << 16) + (count_3[0] << 8) + count_4[0]
        return count_value

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
        # 如果 extrusion_motor_speed 是控件就用 .value()，否则用 float
        stepper_rpm = 0.0
        if hasattr(self.gui, "extrusion_motor_speed"):
            try:
                stepper_rpm = self.gui.extrusion_motor_speed.value()
            except Exception:
                stepper_rpm = 0.0
        return 25/28 * 11 * stepper_rpm * (Spooler.DIAMETER_PREFORM**2 /
                                        (Spooler.DIAMETER_SPOOL * diameter**2))

    def rpm_to_duty_cycle(self, rpm: float) -> float:
        """Convert the RPM to duty cycle"""
        return self.slope * rpm + self.intercept

    def dc_motor_close_loop_control(self, current_time: float) -> None:
        """Closed loop control of the DC motor using PID"""
        if current_time - self.previous_time <= Spooler.SAMPLE_TIME:
            return

        try:
            if not self.motor_calibration:
                print("Motor calibration data not found. Please calibrate the motor.")
                self.motor_calibration = True

            # Read current position and calculate RPM
            current_position = self.read_encoder()
            delta_time = current_time - self.previous_time
            delta_position = current_position - self.previous_position
            current_rpm = (delta_position / Spooler.PULSES_PER_REVOLUTION) * (60 / delta_time)

            # Update previous values
            self.previous_position = current_position
            self.previous_time = current_time

            # setpoint_rpm 直接用 float，不要 .value()
            setpoint_rpm = self.gui.motor_setpoint
            # PID参数直接用默认值
            motor_kp = 0.4
            motor_ki = 0.2
            motor_kd = 0.05

            # Calculate error and PID terms
            error = setpoint_rpm - current_rpm
            self.integral_motor += error * delta_time
            self.integral_motor = max(min(self.integral_motor, 100), -100)  # Anti-windup
            derivative = (error - self.previous_error_motor) / delta_time
            self.previous_error_motor = error

            # Calculate PID output
            output = (motor_kp * error + motor_ki * self.integral_motor + motor_kd * derivative)

            # Convert to duty cycle and apply limits
            output_duty_cycle = self.rpm_to_duty_cycle(output)
            output_duty_cycle = max(min(output_duty_cycle, 100), 0)

            # Apply to motor
            self.update_duty_cycle(output_duty_cycle)

            # Update plots（可选）
            if hasattr(self.gui, "motor_plot"):
                self.gui.motor_plot.update_plot(current_time, current_rpm, setpoint_rpm)

            # Store data
            Database.spooler_timestamps.append(current_time)
            Database.spooler_delta_time.append(delta_time)
            Database.spooler_setpoint.append(setpoint_rpm)
            Database.spooler_rpm.append(current_rpm)
            Database.spooler_kp.append(motor_kp)
            Database.spooler_ki.append(motor_ki)
            Database.spooler_kd.append(motor_kd)

        except Exception as e:
            print(f"Error in motor close loop control: {e}")
            # 不建议在子线程弹窗

    def motor_control_loop(self, current_time: float) -> None:
        """Closed loop control of the DC motor for desired diameter"""
        if current_time - self.previous_time <= Spooler.SAMPLE_TIME:
            return
        try:
            if not self.motor_calibration:
                print("Motor calibration data not found. Please calibrate the motor.")
                self.motor_calibration = True

            # Read current position and calculate RPM
            current_position = self.read_encoder()
            delta_time = current_time - self.previous_time
            delta_position = current_position - self.previous_position
            current_rpm = (delta_position / Spooler.PULSES_PER_REVOLUTION) * (60 / delta_time)

            # Update previous values
            self.previous_position = current_position
            self.previous_time = current_time

            # 获取目标直径和当前直径
            target_diameter = self.gui.target_diameter.value()
            current_diameter = self.get_average_diameter()

            # PID参数（可自定义）
            diameter_kp = 0.1
            diameter_ki = 0.01
            diameter_kd = 0.01

            # 直径闭环
            error = target_diameter - current_diameter
            self.integral_diameter += error * delta_time
            self.integral_diameter = max(min(self.integral_diameter, 0.5), -0.5)
            derivative = (error - self.previous_error_diameter) / delta_time
            self.previous_error_diameter = error
            output = (diameter_kp * error + diameter_ki * self.integral_diameter
                      + diameter_kd * derivative)
            setpoint_rpm = self.diameter_to_rpm(target_diameter)
            setpoint_rpm = max(min(setpoint_rpm, 0), 60)

            # 电机闭环
            motor_kp = 0.4
            motor_ki = 0.2
            motor_kd = 0.05
            error_motor = setpoint_rpm - current_rpm
            self.integral_motor += error_motor * delta_time
            self.integral_motor = max(min(self.integral_motor, 100), -100)
            derivative_motor = (error_motor - self.previous_error_motor) / delta_time
            self.previous_error_motor = error_motor
            output_motor = (motor_kp * error_motor + motor_ki * self.integral_motor +
                            motor_kd * derivative_motor)
            output_duty_cycle = self.rpm_to_duty_cycle(output_motor)
            output_duty_cycle = max(min(output_duty_cycle, 100), 0)
            self.update_duty_cycle(output_duty_cycle)

            # Update plots（可选）
            if hasattr(self.gui, "motor_plot"):
                self.gui.motor_plot.update_plot(current_time, current_rpm, setpoint_rpm)
            if hasattr(self.gui, "diameter_plot"):
                self.gui.diameter_plot.update_plot(current_time, current_diameter, target_diameter)

            # Add data to the database
            Database.spooler_timestamps.append(current_time)
            Database.spooler_delta_time.append(delta_time)
        except Exception as e:
            print(f"Error in motor control loop: {e}")
            # 不建议在子线程弹窗

    def calibrate(self) -> None:
        """Calibrate the DC Motor"""
        rpm_values = []
        duty_cycles = []
        num_samples = 5
        try:
            for duty_cycle in range(20, 101, 10):  # Sweep duty cycle from 20% to 100% in increments of 10%
                rpm_samples = []
                for _ in range(num_samples):
                    self.update_duty_cycle(duty_cycle)
                    time.sleep(2)
                    # Measure RPM
                    start_time = time.perf_counter()
                    start_position = self.read_encoder()
                    time.sleep(Spooler.SAMPLE_TIME)
                    end_time = time.perf_counter()
                    end_position = self.read_encoder()
                    delta_time = end_time - start_time
                    delta_position = end_position - start_position
                    rpm = (delta_position / Spooler.PULSES_PER_REVOLUTION) * (60 / delta_time)
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

        print("Motor calibration completed. Please restart the program.")
        self.stop()

