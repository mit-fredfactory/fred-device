"""File for the fan controller"""
import RPi.GPIO as GPIO
from user_interface import UserInterface
from database import Database

class Fan:
    """Controller for the fan"""
    PIN = 13
    SAMPLE_TIME = 0.1  # seconds

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui

        GPIO.setup(Fan.PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(Fan.PIN, 1000)
        self.pwm.start(0)

        self.previous_time = 0.0

    def start(self, duty_cycle: float) -> None:
        """Start the fan PWM"""
        self.pwm.start(duty_cycle)

    def stop(self) -> None:
        """Stop the fan PWM"""
        if self.pwm:
            self.pwm.stop()

    def update_duty_cycle(self, duty_cycle: float) -> None:
        """Update speed"""
        self.pwm.ChangeDutyCycle(duty_cycle)

    def control_loop(self, current_time: float) -> None:
        """Set the desired speed"""
        if current_time - self.previous_time <= Fan.SAMPLE_TIME:
            return
        try:
            duty_cycle = self.gui.fan_duty_cycle.value()
            self.update_duty_cycle(duty_cycle)

            dt = current_time - self.previous_time
            self.previous_time = current_time

            Database.fan_timestamps.append(current_time)
            Database.fan_delta_time.append(dt)
            Database.fan_duty_cycle.append(duty_cycle)

        except Exception as e:
            print(f"Error in fan control loop: {e}")
            self.gui.show_message("Error in fan control loop",
                                    "Please restart the program.")
