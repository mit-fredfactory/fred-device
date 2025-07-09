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
    prev_len_cooling = 0
    # prev_len_fan_duty_cycle = 0
    # prev_len_fan_duty_cycle = 0
    # prev_len_fan_duty_cycle = 0

    while True:
        new_data_flag = False

        with buffer_lock:
            curr_len_cooling = len(Database.fan_duty_cycle)
            if curr_len_cooling > prev_len_cooling: # check if new data exists

                # create JSON message with arrays from lists
                batch_to_send_cooling = {
                    "timestamp":Database.camera_timestamps[prev_len_cooling:curr_len_cooling],
                    "duty_cycle":Database.fan_duty_cycle[prev_len_cooling:curr_len_cooling]
                    }

            else:
                batch_to_send = []

        if batch_to_send:
            mqtt_payload_cooling = json.dumps(batch_to_send_cooling)
            mqtt_client.try_publish(mqtt_payload_cooling, 'cooling')
            
            # result = client.publish(MQTT_TOPIC, payload) # original sending command
            # try:    
            #     mqtt_connection.publish(topic=MQTT_TOPIC, 
            #                             payload=message_json, 
            #                             qos=mqtt.QoS.AT_LEAST_ONCE)
            #     print(f"Published batch to MQTT: {num_data_in_batch} Images")
            # except:
            #     print("Failed to send message")    
            prev_len_cooling = curr_len_cooling
        else:
            print("\nNo data to send this cycle.\n")
        

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
    


