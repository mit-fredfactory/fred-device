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
        # Do NOT call gui.show_message here (from thread), just print/log the error
        return

    init_time = time.time()
    while True:
        try:
            current_time = time.time() - init_time
            Database.time_readings.append(current_time)
            if gui.start_motor_calibration:
                spooler.calibrate()
                gui.start_motor_calibration = False

            # Open loop heater (if enabled)
            if hasattr(gui, "heater_open_loop_enabled") and gui.heater_open_loop_enabled:
                extruder.temperature_open_loop_control(current_time)
            # Closed loop heater (default)
            elif gui.device_started:
                extruder.temperature_control_loop(current_time)
                extruder.stepper_control_loop()
                if hasattr(gui, "spooling_control_state") and gui.spooling_control_state:
                    spooler.motor_control_loop(current_time)
                fan.control_loop()
            time.sleep(0.05)
        except Exception as e:
            print(f"Error in hardware control loop: {e}")
            # Do NOT call gui.show_message here (from thread), just print/log the error
            try:
                fan.stop()
                spooler.stop()
                extruder.stop()
            except Exception:
                pass
            break

if __name__ == "__main__":
    print("Starting FrED Device...")
    ui = UserInterface()
    time.sleep(2)
    hardware_thread = threading.Thread(target=hardware_control, args=(ui,), daemon=True)
    hardware_thread.start()
    ui.start_gui()
    print("FrED Device Closed.")
