# mqtt_client.py
from awscrt import mqtt, http
from awsiot import mqtt_connection_builder
from utils.command_line_utils import CommandLineUtils
import json
import threading
import time


class MQTTClient:
    def __init__(self, topic: str, publish_interval: float = 5.0):
        self.topic = topic
        self.publish_interval = publish_interval
        self.lock = threading.Lock()
        self.last_publish_time = 0
        self.connection = None
        self.cmd_data = CommandLineUtils.parse_sample_input_pubsub()
        self.cmd_data.input_topic = topic
        self._connect()

    def _connect(self):
        proxy_options = None
        if self.cmd_data.input_proxy_host and self.cmd_data.input_proxy_port != 0:
            proxy_options = http.HttpProxyOptions(
                host_name=self.cmd_data.input_proxy_host,
                port=self.cmd_data.input_proxy_port)

        self.connection = mqtt_connection_builder.mtls_from_path(
            endpoint=self.cmd_data.input_endpoint,
            port=self.cmd_data.input_port,
            cert_filepath=self.cmd_data.input_cert,
            pri_key_filepath=self.cmd_data.input_key,
            ca_filepath=self.cmd_data.input_ca,
            client_id=self.cmd_data.input_clientId,
            clean_session=False,
            keep_alive_secs=30,
            http_proxy_options=proxy_options
        )

        print("Connecting to AWS IoT Core...")
        connect_future = self.connection.connect()
        connect_future.result()
        print("MQTT connection established.")

    def try_publish(self, mqtt_subtopic: str, payload_dict: dict):
        """Publish data if publish_interval has passed."""
        with self.lock:
            # current_time = time.time()
            # if current_time - self.last_publish_time >= self.publish_interval:
            try:
                message_json = json.dumps(payload_dict)
                self.connection.publish(
                    topic=self.topic + mqtt_subtopic,
                    payload=message_json,
                    qos=mqtt.QoS.AT_LEAST_ONCE
                )
                print(f"[MQTT] Published: {message_json}")
                # self.last_publish_time = current_time
            except Exception as e:
                print(f"[MQTT] Publish failed: {e}")

    def disconnect(self):
        if self.connection:
            print("Disconnecting MQTT...")
            disconnect_future = self.connection.disconnect()
            disconnect_future.result()
            print("MQTT disconnected.")
