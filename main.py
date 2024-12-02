"""Main file to run the FrED device"""
""" https://drive.google.com/drive/folders/18odhWyNApQg9eVLjScSApWUiiRCTNP79?usp=sharing """

import threading
import time
import RPi.GPIO as GPIO
from database import Database
from user_interface import UserInterface
from fan import Fan
from spooler import Spooler
from extruder import Extruder
import math
import signal
import os

def custom_handler(signum, frame):
    print("\nDetected keyboard interrupt (Ctrl+C)")
    print("Switching temperature OFF and exiting...")
    GPIO.setup(6, GPIO.OUT)
    GPIO.output(6, GPIO.LOW)
    # Can add all the required GPIOs needed to turn off
    os._exit(0)
    
signal.signal(signal.SIGINT, custom_handler)

def hardware_control(gui: UserInterface) -> None:
    """Thread to handle hardware control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        fan = Fan(gui)
        spooler = Spooler(gui)
        extruder = Extruder(gui)
        fan.start(1000, 45)
        spooler.start(1000, 0)
    except Exception as e:
        print(f"Error in hardware control: {e}")
        gui.show_message("Error while starting the device",
                         "Please restart the program.")

    init_time = time.time()
    while True:
        try:
            current_time = time.time() - init_time
            Database.time_readings.append(current_time)
            if gui.start_motor_calibration:
                spooler.calibrate()
                gui.start_motor_calibration = False
            if gui.device_started:
                RPM = extruder.stepper_control_loop()
                
                duty_cycle = extruder.PWM_temperature_control(current_time)
                
                period = 10
                ON_time = math.ceil(period/2*duty_cycle+period/2)
                OFF_time = period - ON_time
                
                print("ON time: ", ON_time)
                print("OFF time: ", OFF_time)
                
                for i in range(ON_time):
                    extruder.turnON()
                    time.sleep(0.01)
                for i in range(OFF_time):
                    extruder.turnOFF()
                    time.sleep(0.01)
                
                
                if gui.spooling_control_state:
                    spooler.motor_speed(RPM)
                fan.control_loop()
            else:
                extruder.turnOFF()
                time.sleep(0.05)
        except Exception as e:
            print(f"Error in hardware control loop: {e}")
            gui.show_message("Error in hardware control loop",
                             "Please restart the program.")
            fan.stop()
            spooler.stop()
            extruder.stop()

if __name__ == "__main__":
    print("Starting FrED Device...")
    ui = UserInterface()
    time.sleep(2)
    hardware_thread = threading.Thread(target=hardware_control, args=(ui,))
    hardware_thread.start()
    threading.Lock()
    ui.start_gui()
    hardware_thread.join()
    print("FrED Device Closed.")
