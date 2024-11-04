# fred-device
Code for the Fiber Extrusion Device

## Modules

### `database.py`
This module handles data storage and CSV file generation for the FrED device. It stores raw data, such as temperature readings, diameter readings, and motor speeds, and generates a CSV file with the collected data. The module also provides functions for retrieving and updating calibration data from a YAML file.

#### Methods:
- `generate_csv(cls, filename: str) -> None`: Generate a CSV file with the data.
- `get_calibration_data(field: str) -> float`: Get calibration data from the yaml file.
- `update_calibration_data(field: str, value: str) -> None`: Update calibration data in the yaml file.

### `extruder.py`
This module controls the extrusion process of the FrED device. It manages the heater and stepper motor for the extruder, including temperature control using a thermistor and PID control. The module also provides functions for setting microstepping modes and stepping the motor in a given direction.

#### Methods:
- `get_temperature(cls, voltage: float) -> float`: Get the average temperature from the voltage using Steinhart-Hart equation.
- `set_microstepping(self, mode: str) -> None`: Set the microstepping mode.
- `motor_step(self, direction: int) -> None`: Step the motor in the given direction.
- `stepper_control_loop(self) -> None`: Move the stepper motor constantly.
- `temperature_control_loop(self, current_time: float) -> None`: Closed loop control of the temperature of the extruder for desired diameter.

### `fan.py`
This module controls the fan used in the FrED device. It provides functions for starting, stopping, and updating the fan's duty cycle. The module also includes a control loop for setting the desired fan speed based on user input.

#### Methods:
- `start(self, frequency: float, duty_cycle: float) -> None`: Start the fan PWM.
- `stop(self) -> None`: Stop the fan PWM.
- `update_duty_cycle(self, duty_cycle: float) -> None`: Update speed.
- `control_loop(self) -> None`: Set the desired speed.

### `fiber_camera.py`
This module processes video from the camera to obtain the fiber diameter and display it. It captures and processes frames from the camera, detects edges, and calculates the fiber diameter. The module also provides functions for calibrating the camera and updating the displayed images.

#### Methods:
- `camera_loop(self) -> None`: Loop to capture and process frames from the camera.
- `get_edges(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]`: Filter the frame to enhance the edges.
- `get_fiber_diameter(self, lines)`: Get the fiber diameter from the edges detected in the image.
- `plot_lines(self, frame, lines)`: Plot the detected lines on the frame.
- `calibrate(self)`: Calibrate the camera.

### `main.py`
This is the main file for running the FrED device. It initializes the user interface and starts the hardware control thread. The hardware control thread manages the fan, spooler, and extruder, and updates the device's state based on user input and sensor readings.

### `spooler.py`
This module controls the spooling process of the FrED device. It manages the DC motor for the spooler, including PID control for motor speed and diameter control. The module also provides functions for calibrating the motor and updating the duty cycle based on the desired RPM.

#### Methods:
- `initialize_encoder(self) -> None`: Initialize the encoder and SPI.
- `start(self, frequency: float, duty_cycle: float) -> None`: Start the DC Motor PWM.
- `stop(self) -> None`: Stop the DC Motor PWM.
- `update_duty_cycle(self, duty_cycle: float) -> None`: Update the DC Motor PWM duty cycle.
- `get_average_diameter(self) -> float`: Get the average diameter of the fiber.
- `diameter_to_rpm(self, diameter: float) -> float`: Convert the fiber diameter to RPM of the spooling motor.
- `rpm_to_duty_cycle(self, rpm: float) -> float`: Convert the RPM to duty cycle.
- `motor_control_loop(self, current_time: float) -> None`: Closed loop control of the DC motor for desired diameter.
- `calibrate(self) -> None`: Calibrate the DC Motor.

### `user_interface.py`
This module sets up the layout of the user interface for the FrED device. It provides functions for adding plots, controls, and buttons to the interface, as well as updating the displayed values based on user input and sensor readings. The module also includes functions for starting the GUI and handling user interactions.

#### Methods:
- `add_plots(self)`: Add plots to the layout.
- `add_diameter_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI spin boxes to control the diameter.
- `add_motor_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI spin boxes to control the motors.
- `add_temperature_controls(self) -> Tuple[QLabel, QSlider, QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI controls for the temperature.
- `add_fan_controls(self) -> Tuple[QLabel, QSlider]`: Add UI controls for the fan.
- `add_buttons(self)`: Add buttons to the layout.
- `start_gui(self) -> None`: Start the GUI.
- `update_temperature_slider_label(self, value) -> None`: Update the temperature slider label.
- `update_fan_slider_label(self, value) -> None`: Update the fan slider label.
- `spooling_control_toggle(self) -> None`: Toggle the spooling control.
- `set_start_device(self) -> None`: Set start device flag.
- `set_calibrate_motor(self) -> None`: Set calibrate motor flag.
- `set_calibrate_camera(self) -> None`: Call calibrate camera.
- `set_download_csv(self) -> None`: Call download csv from database.
- `show_message(self, title: str, message: str) -> None`: Show a message box.
