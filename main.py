"""Main file to run the FrED device"""
import threading
import time
import RPi.GPIO as GPIO
from database import Database
from user_interface import UserInterface
from fan import Fan
from spooler import Spooler
from extruder import Extruder

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
                
            # DC Motor Control Logic
            if gui.dc_motor_open_loop_enabled and not gui.spooling_control_state:
                spooler.dc_motor_open_loop_control(current_time)
                
            # Heater Control Logic
            if gui.heater_on_off_enabled and not gui.device_started:
                extruder.temperature_on_and_off_control(current_time)
                extruder.stepper_control_loop()
            
            # Camera Feedback PLOT OPEN LOOP
            if gui.camera_feedback_enabled:
                gui.fiber_camera.camera_feedback(current_time)
                            
            elif gui.device_started:
                extruder.temperature_control_loop(current_time)
                
                extruder.stepper_control_loop()
                
                if gui.spooling_control_state and not gui.dc_motor_open_loop_enabled:
                    spooler.motor_control_loop(current_time)
                fan.control_loop()
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

