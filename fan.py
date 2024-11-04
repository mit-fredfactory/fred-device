"""File for the fan controller"""
import RPi.GPIO as GPIO
from user_interface import UserInterface

class Fan:
    """Controller for the fan"""
    PIN = 13
    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.duty_cycle = 0.0
        self.pwm = None
        GPIO.setup(Fan.PIN, GPIO.OUT)
        print(self.gui.device_started)

    def start(self, frequency: float, duty_cycle: float) -> None:
        """Start the fan PWM"""
        self.pwm = GPIO.PWM(Fan.PIN, frequency)
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
        try:
            self.update_duty_cycle(self.gui.fan_duty_cycle.value())
        except Exception as e:
            print(f"Error in fan control loop: {e}")
            self.gui.show_message("Error in fan control loop",
                                    "Please restart the program.")
