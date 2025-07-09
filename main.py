"""Main file to run the FrED device"""
import threading
import time
import json
import RPi.GPIO as GPIO
from database import Database
from user_interface import UserInterface
from fan import Fan
from spooler import Spooler
from extruder import Extruder
from mqtt_client import MQTTClient

CLIENT_ID = 'fred_device1'
MQTT_TOPIC = 'mit/fred/device1/'
BATCH_INTERVAL = 5.0 # seconds

buffer_lock = threading.Lock()


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
            if gui.dc_motor_open_loop_enabled and not gui.dc_motor_close_loop_enabled:
                spooler.dc_motor_open_loop_control(current_time)
                
            elif gui.dc_motor_close_loop_enabled and not gui.dc_motor_open_loop_enabled:
                spooler.dc_motor_close_loop_control(current_time)
            
            # Heater Control Logic
            if gui.heater_open_loop_enabled and not gui.device_started:  
                extruder.temperature_open_loop_control(current_time)     
                extruder.stepper_control_loop()
            
            # Camera Feedback PLOT OPEN LOOP
            if gui.camera_feedback_enabled:
                gui.fiber_camera.camera_feedback(current_time)
                            
            if gui.device_started:
                extruder.temperature_control_loop(current_time)
                extruder.stepper_control_loop()
                
            fan.control_loop()
            time.sleep(0.05)
        except Exception as e:
            print(f"Error in hardware control loop: {e}")
            gui.show_message("Error in hardware control loop",
                             "Please restart the program.")
            fan.stop()
            spooler.stop()
            extruder.stop()

def mqtt_control(mqtt_client: MQTTClient) -> None:
    prev_len_spooling = 0
    prev_len_heater = 0
    prev_len_cooling = 0

    while True:

        with buffer_lock:
            # Spooling + Extruder Motor
            curr_len_spooling = len(Database.spooler_timestamps)
            if curr_len_spooling > prev_len_spooling: # check if new data exists
                # create JSON message with arrays from lists
                batch_to_send_spooling = {
                    "timestamp":Database.spooler_timestamps[prev_len_spooling:curr_len_spooling],
                    "actual":Database.spooler_rpm[prev_len_spooling:curr_len_spooling],
                    "setpoint":Database.spooler_setpoint[prev_len_spooling:curr_len_spooling],
                    # "duty_cycle":Database.spooler_rpm[prev_len_spooling:curr_len_spooling],
                    "kp":Database.spooler_kp[prev_len_spooling:curr_len_spooling],
                    "ki":Database.spooler_ki[prev_len_spooling:curr_len_spooling],
                    "kd":Database.spooler_kd[prev_len_spooling:curr_len_spooling]
                    }
                batch_to_send_extruder_motor = {
                    "timestamp":Database.spooler_timestamps[prev_len_spooling:curr_len_spooling],
                    "actual":Database.extruder_rpm[prev_len_spooling:curr_len_spooling]
                    }
            else:
                batch_to_send_spooling = []
                # batch_to_send_extruder_motor = []

            # Heater
            curr_len_heater = len(Database.temperature_timestamps)
            if curr_len_heater > prev_len_heater: # check if new data exists
                # create JSON message with arrays from lists
                batch_to_send_heater = {
                    "timestamp":Database.temperature_timestamps[prev_len_heater:curr_len_heater],
                    "actual":Database.temperature_readings[prev_len_heater:curr_len_heater],
                    "setpoint":Database.temperature_setpoint[prev_len_heater:curr_len_heater],
                    "duty_cycle":Database.temperature_pid_output[prev_len_heater:curr_len_heater],
                    "kp":Database.temperature_kp[prev_len_heater:curr_len_heater],
                    "ki":Database.temperature_ki[prev_len_heater:curr_len_heater],
                    "kd":Database.temperature_kd[prev_len_heater:curr_len_heater],
                    }
            else:
                batch_to_send_heater = []

            # Diameter + Cooling
            curr_len_cooling = len(Database.camera_timestamps)
            if curr_len_cooling > prev_len_cooling: # check if new data exists
                # create JSON message with arrays from lists
                batch_to_send_cooling = {
                    "timestamp":Database.camera_timestamps[prev_len_cooling:curr_len_cooling],
                    "duty_cycle":Database.fan_duty_cycle[prev_len_cooling:curr_len_cooling]
                    }
                batch_to_send_diameter = {
                    "timestamp":Database.camera_timestamps[prev_len_cooling:curr_len_cooling],
                    "actual":Database.diameter_readings[prev_len_cooling:curr_len_cooling],
                    "setpoint":Database.diameter_setpoint[prev_len_cooling:curr_len_cooling]
                    }                
            else:
                batch_to_send_cooling = []
                batch_to_send_diameter = []

        # Spooling + Extruder Motor
        if batch_to_send_spooling:
            mqtt_payload_spooling = json.dumps(batch_to_send_spooling)
            mqtt_client.try_publish('spooling', mqtt_payload_spooling)   

            mqtt_payload_extruder_motor = json.dumps(batch_to_send_extruder_motor)
            mqtt_client.try_publish('extruder_motor', mqtt_payload_extruder_motor)

            prev_len_spooling = curr_len_spooling
        else:
            print("\nNo Spooling and Extruder Motor data to send this cycle.\n")
        
        # Heater
        if batch_to_send_heater:
            mqtt_payload_heater = json.dumps(batch_to_send_heater)
            mqtt_client.try_publish('extruder_heater', mqtt_payload_heater)   

            prev_len_heater = curr_len_heater
        else:
            print("\nNo Extruder Heater data to send this cycle.\n")
        
        # Diameter + Cooling
        if batch_to_send_cooling:
            mqtt_payload_cooling = json.dumps(batch_to_send_cooling)
            mqtt_client.try_publish('cooling', mqtt_payload_cooling)   

            mqtt_payload_diameter = json.dumps(batch_to_send_diameter)
            mqtt_client.try_publish('diameter', mqtt_payload_diameter)
               
            prev_len_cooling = curr_len_cooling
        else:
            print("\nNo Diameter and Cooling data to send this cycle.\n")
        
        time.sleep(5)


if __name__ == "__main__":
    
    print("Starting FrED Device...")
    ui = UserInterface()
    time.sleep(2)

    print("Starting MQTT Client...")
    mqtt_client = MQTTClient(topic = MQTT_TOPIC)
    mqtt_client._connect()
    time.sleep(2)


    hardware_thread = threading.Thread(target=hardware_control, args=(ui,))
    mqtt_thread = threading.Thread(target=mqtt_control, args=(mqtt_client,))

    hardware_thread.start()
    mqtt_thread.start()
    threading.Lock()

    # Start GUI (blocking)
    try:
        ui.start_gui()
    except KeyboardInterrupt:
        print("GUI stopped.")


    # Cleanup
    mqtt_client.disconnect()
    hardware_thread.join()
    print("FrED Device Closed.")
    


