# ------------------------------------------------------------------------
# ------------------------------------------------------------------------
#
#               *** Example Code for Streaming Data to AWS IoT Core ***
#   
#   This sample uses the Message Broker for AWS IoT to send messages 
#   through an MQTT connection. On startup, the device connects to the server, 
#   aggregates incoming information for a user-defined amount of time (5 seconds)
#   and publishing messages to that topic at the end of every aggregation cycle.
#
#   Actions
#   1) Connects to IoT Core using provided credential (certificates)
#   2) Aggregates incoming information in batches
#   3) Publishes information via MQTT in aggregated arrays
#
#
#   User must define:
#   1) Client ID name
#   2) Topic where code will publish information
#   2) Interval at which information is aggregated then published (default 5 seconds)
#
#   Last Modified: 
#   04 July 2025
#   Kyle Saleeby
#   kylesaleeby@gatech.edu
#
#   Acknowledgements & Sources:
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
#   SPDX-License-Identifier: Apache-2.0.

# ------------------------------------------------------------------------
# ------------------------------------------------------------------------
from awscrt import mqtt, http
from awsiot import mqtt_connection_builder
import sys
import threading
import time
import json
import random
from utils.command_line_utils import CommandLineUtils



#      ******* USER DEFINED CONFIG ********

CLIENT_ID = 'fred_device1'
MQTT_TOPIC = 'mit/fred/device1'     # 'mit/fredfactory/device1', 'mit/laptop/<name>'
BATCH_INTERVAL = 5 # seconds

# Shared data buffer and lock
sensor1_buffer = []
sensor2_buffer = []
sensor3_buffer = []
buffer_lock = threading.Lock()

#  ************************************

# cmdData is the arguments/input from the command line placed into a single struct for
# use in this sample. This handles all of the command line parsing, validating, etc.
# See the Utils/CommandLineUtils for more information.
cmdData = CommandLineUtils.parse_sample_input_pubsub()
cmdData.input_clientId = CLIENT_ID # Update client id from script variable


received_count = 0
received_all_event = threading.Event()

# Callback when connection is accidentally lost.
def on_connection_interrupted(connection, error, **kwargs):
    print("Connection interrupted. error: {}".format(error))

# Callback when an interrupted connection is re-established.
def on_connection_resumed(connection, return_code, session_present, **kwargs):
    print("Connection resumed. return_code: {} session_present: {}".format(return_code, session_present))

    if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
        print("Session did not persist. Resubscribing to existing topics...")
        resubscribe_future, _ = connection.resubscribe_existing_topics()

        # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
        # evaluate result with a callback instead.
        resubscribe_future.add_done_callback(on_resubscribe_complete)

def on_resubscribe_complete(resubscribe_future):
    resubscribe_results = resubscribe_future.result()
    print("Resubscribe results: {}".format(resubscribe_results))

    for topic, qos in resubscribe_results['topics']:
        if qos is None:
            sys.exit("Server rejected resubscribe to topic: {}".format(topic))

# Callback when the subscribed topic receives a message
def on_message_received(topic, payload, dup, qos, retain, **kwargs):
    print("Received message from topic '{}': {}".format(topic, payload))
    global received_count
    received_count += 1
    if received_count == cmdData.input_count:
        received_all_event.set()

# Callback when the connection successfully connects
def on_connection_success(connection, callback_data):
    assert isinstance(callback_data, mqtt.OnConnectionSuccessData)
    print("Connection Successful with return code: {} session present: {}".format(callback_data.return_code, callback_data.session_present))

# Callback when a connection attempt fails
def on_connection_failure(connection, callback_data):
    assert isinstance(callback_data, mqtt.OnConnectionFailureData)
    print("Connection failed with error code: {}".format(callback_data.error))

# Callback when a connection has been disconnected or shutdown successfully
def on_connection_closed(connection, callback_data):
    print("Connection closed")

if __name__ == '__main__':
    # Create the proxy options if the data is present in cmdData
    proxy_options = None
    if cmdData.input_proxy_host is not None and cmdData.input_proxy_port != 0:
        proxy_options = http.HttpProxyOptions(
            host_name=cmdData.input_proxy_host,
            port=cmdData.input_proxy_port)

    # Create a MQTT connection from the command line data
    mqtt_connection = mqtt_connection_builder.mtls_from_path(
        endpoint=cmdData.input_endpoint,
        port=cmdData.input_port,
        cert_filepath=cmdData.input_cert,
        pri_key_filepath=cmdData.input_key,
        ca_filepath=cmdData.input_ca,
        on_connection_interrupted=on_connection_interrupted,
        on_connection_resumed=on_connection_resumed,
        client_id=cmdData.input_clientId,
        clean_session=False,
        keep_alive_secs=30,
        http_proxy_options=proxy_options,
        on_connection_success=on_connection_success,
        on_connection_failure=on_connection_failure,
        on_connection_closed=on_connection_closed)


    # Attempt to connect to IoT Core MQTT Broker, confirm if connection is successful
    if not cmdData.input_is_ci:
        print(f"Connecting to {cmdData.input_endpoint} with client ID '{cmdData.input_clientId}'...")
    else:
        print("Connecting to endpoint with client ID")
    connect_future = mqtt_connection.connect()

    connect_future.result() # Future.result() waits until a result is available
    print("Connected!")

    message_count = cmdData.input_count
    message_topic = cmdData.input_topic
    message_string = cmdData.input_message


    def ingest_data():
        """Simulates incoming data stream."""
        while True:
            new_sensor_data1 = random.randint(1, 100)
            new_sensor_data2 = random.randint(1, 100)
            new_sensor_data3 = random.randint(1, 100)
            
            with buffer_lock:
                sensor1_buffer.append(new_sensor_data1)
                sensor2_buffer.append(new_sensor_data2)
                sensor3_buffer.append(new_sensor_data3)
            print(f"Data generated: {new_sensor_data1, new_sensor_data2, new_sensor_data3}")
            time.sleep(random.uniform(0.1, 0.2))

    def send_batch():
        """Publishes batched data via MQTT every BATCH_INTERVAL seconds."""
        while True:
            time.sleep(BATCH_INTERVAL)
            with buffer_lock:
                if sensor1_buffer: # check if new data exists
                    
                    # create JSON message with arrays from lists
                    batch_to_send = {"sensor1":sensor1_buffer[:],
                                     "sensor2":sensor2_buffer[:],
                                     "sensor3":sensor3_buffer[:]}
                    
                    # clear lists
                    sensor1_buffer.clear()
                    sensor2_buffer.clear()
                    sensor3_buffer.clear()
                else:
                    batch_to_send = []

            if batch_to_send:
                message_json = json.dumps(batch_to_send)
                
                # result = client.publish(MQTT_TOPIC, payload) # original sending command
                try: 
                    mqtt_connection.publish(topic=MQTT_TOPIC, 
                                            payload=message_json, 
                                            qos=mqtt.QoS.AT_LEAST_ONCE)
                    print(f"\nPublished batch to MQTT: {message_json}\n")
                except:
                    print("Failed to send message")
                    
            else:
                print("\nNo data to send this cycle.\n")
                
    # Start everything
    ingest_thread = threading.Thread(target=ingest_data, daemon=True)
    batch_thread = threading.Thread(target=send_batch, daemon=True)

    ingest_thread.start()
    batch_thread.start()
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Disconnect
        print("Disconnecting...")
        disconnect_future = mqtt_connection.disconnect()
        disconnect_future.result()
        print("Disconnected!")
        print("Program stopped.")



