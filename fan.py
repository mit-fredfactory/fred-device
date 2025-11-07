"""File for the fan controller"""
import RPi.GPIO as GPIO
from user_interface import UserInterface
from database import Database

class Fan:
    """Controller for the fan"""
    PIN = 13

    def __init__(self, gui: UserInterface) -> None:
        self.gui = gui
        self.duty_cycle = 30.0  # 默认风扇占空比为30%
        self.pwm = None
        GPIO.setup(Fan.PIN, GPIO.OUT)
        print(self.gui.device_started)

    def start(self, frequency: float, duty_cycle: float = 30.0) -> None:
        """Start the fan PWM"""
        self.pwm = GPIO.PWM(Fan.PIN, frequency)
        self.pwm.start(duty_cycle)
        self.duty_cycle = duty_cycle

    def stop(self) -> None:
        """Stop the fan PWM"""
        if self.pwm:
            self.pwm.stop()

    def update_duty_cycle(self, duty_cycle: float = None) -> None:
        """Update speed"""
        if duty_cycle is not None:
            self.duty_cycle = duty_cycle
        # 如果未指定 duty_cycle，则继续用当前 self.duty_cycle
        if self.pwm:
            self.pwm.ChangeDutyCycle(self.duty_cycle)
            Database.fan_duty_cycle.append(self.duty_cycle)

    def control_loop(self) -> None:
        """Set the desired speed (fixed at default 30%)"""
        try:
            # 这里直接用 self.duty_cycle，不再依赖 GUI 输入
            self.update_duty_cycle(self.duty_cycle)
        except Exception as e:
            print(f"Error in fan control loop: {e}")
            # 建议不要在子线程弹窗
            # self.gui.show_message("Error in fan control loop", "Please restart the program.")
