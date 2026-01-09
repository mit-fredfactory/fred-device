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
        
        fan.start(50) # Start fan at 50% duty cycle
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
            if gui.dc_motor_open_loop_enabled and not gui.dc_motor_close_loop_enabled:
                spooler.dc_motor_open_loop_control(current_time)                
            elif gui.dc_motor_close_loop_enabled and not gui.dc_motor_open_loop_enabled:
                spooler.dc_motor_close_loop_control(current_time)
            
            # Heater Control Logic
            if gui.heater_open_loop_enabled and not gui.device_started:  
                extruder.temperature_open_loop_control(current_time)     
                extruder.stepper_control_loop(current_time)  
         
            if gui.device_started:
                extruder.temperature_control_loop(current_time)
                extruder.stepper_control_loop(current_time)

            # Camera Feedback PLOT OPEN LOOP
            if gui.camera_feedback_enabled:
                gui.fiber_camera.camera_feedback(current_time)
                   
            # Fan controls is always on     
            fan.control_loop(current_time)

            time.sleep(0.05)
        except Exception as e:
            print(f"Error in hardware control loop: {e}")
            gui.show_message("Error in hardware control loop",
                             "Please restart the program.")
            fan.stop()
            spooler.stop()
            extruder.stop()


def cooling_control(gui: UserInterface) -> None:
    print("Starting COOLING Thread...")
    """Thread to handle COOLING control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        fan = Fan(gui)
        fan.start(50) # Start fan at 50% duty cycle
    except Exception as e:
        print(f"Error in COOLING control: {e}")
        gui.show_message("Error while starting the device",
                         "Please restart the program.")

    init_time = time.time()
    
    while True:
        try:
            current_time = time.time() - init_time
            # Database.time_readings.append(current_time)
                   
            # Fan controls is always on     
            fan.control_loop(current_time)

            # time.sleep(0.05)
        except Exception as e:
            print(f"Error in COOLING control loop: {e}")
            gui.show_message("Error in COOLING control loop",
                             "Please restart the program.")
            fan.stop()


def spooler_control(gui: UserInterface) -> None:
    print("Starting SPOOLER Thread...")
    """Thread to handle hardware control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        spooler = Spooler(gui)
        spooler.start(1000, 0)
    except Exception as e:
        print(f"Error in SPOOLER control: {e}")
        gui.show_message("Error while starting the device",
                         "Please restart the program.")

    init_time = time.time()
    while True:
        try:
            current_time = time.time() - init_time
            # Database.time_readings.append(current_time)

            if gui.start_motor_calibration:
                spooler.calibrate()
                gui.start_motor_calibration = False
                
            # DC Motor Control Logic
            if gui.dc_motor_open_loop_enabled and not gui.dc_motor_close_loop_enabled:
                spooler.dc_motor_open_loop_control(current_time)                
            elif gui.dc_motor_close_loop_enabled and not gui.dc_motor_open_loop_enabled:
                spooler.dc_motor_close_loop_control(current_time)


            # time.sleep(0.05)
        except Exception as e:
            print(f"Error in SPOOLER control loop: {e}")
            gui.show_message("Error in SPOOLER control loop",
                             "Please restart the program.")

            spooler.stop()

def extruder_control(gui: UserInterface) -> None:
    print("Starting EXTRUDER Thread...")
    """Thread to handle hardware control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        extruder = Extruder(gui)
    except Exception as e:
        print(f"Error in EXTRUDER control: {e}")
        gui.show_message("Error while starting the device",
                         "Please restart the program.")

    init_time = time.time()
    while True:
        try:
            current_time = time.time() - init_time
            # Database.time_readings.append(current_time)

            # Heater Control Logic
            if gui.heater_open_loop_enabled and not gui.device_started:  
                extruder.temperature_open_loop_control(current_time)     
                extruder.stepper_control_loop(current_time)  
         
            if gui.device_started:
                extruder.temperature_control_loop(current_time)
                extruder.stepper_control_loop(current_time)

            # time.sleep(0.05)
        except Exception as e:
            print(f"Error in EXTRUDER control loop: {e}")
            gui.show_message("Error in EXTRUDER control loop",
                             "Please restart the program.")
            extruder.stop()

def camera_control(gui: UserInterface) -> None:
    print("Starting CAMERA Thread...")
    """Thread to handle hardware control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    init_time = time.time()
    while True:
        try:
            current_time = time.time() - init_time
            Database.time_readings.append(current_time)

            # Camera Feedback PLOT OPEN LOOP
            if gui.camera_feedback_enabled:
                gui.fiber_camera.camera_feedback(current_time)
                   
            # time.sleep(0.05)
        except Exception as e:
            print(f"Error in CAMERA control loop: {e}")
            gui.show_message("Error in CAMERA control loop",
                             "Please restart the program.")
            
if __name__ == "__main__":
    print("Starting FrED Device...")
    ui = UserInterface()
    time.sleep(2)

    cooling_thread = threading.Thread(target=cooling_control, args=(ui,))
    cooling_thread.start()

    spooler_thread = threading.Thread(target=spooler_control, args=(ui,))
    spooler_thread.start()

    extruder_thread = threading.Thread(target=extruder_control, args=(ui,))
    extruder_thread.start()

    camera_thread = threading.Thread(target=camera_control, args=(ui,))
    camera_thread.start()
    # threading.Lock()

    # Start GUI (blocking)
    try:
        ui.start_gui()
    except KeyboardInterrupt:
        print("GUI stopped.")

    # Cleanup
    cooling_thread.join()
    spooler_thread.join()   
    extruder_thread.join()  
    camera_thread.join()
    print("FrED Device Closed.")


# if __name__ == "__main__":
#     print("Starting FrED Device...")
#     ui = UserInterface()
#     time.sleep(2)

#     hardware_thread = threading.Thread(target=hardware_control, args=(ui,))
#     hardware_thread.start()
#     threading.Lock()

#     # Start GUI (blocking)
#     try:
#         ui.start_gui()
#     except KeyboardInterrupt:
#         print("GUI stopped.")

#     # Cleanup
#     hardware_thread.join()
#     print("FrED Device Closed.")

