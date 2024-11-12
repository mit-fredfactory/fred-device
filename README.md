# $${\color{red}MIT}$$  $${\color{red}FrED}$$
Code for the Fiber Extrusion Device's Raspberry Pi
## Setup
Open a terminal in the Raspberry Pi (`Ctrl + Alt + T`) and type:
```bash
git clone https://github.com/mit-fredfactory/fred-device.git # This will clone the git folder
cd fred-device # Gets in the fred-device folder
```
You will find multiple files in the folder, go to the [modules](#modules) section for an overview on each file.

### Virtual environment
To set up a virtual environment, type:
```bash
python3 -m venv --system-site-packages fred-env # Creates a virtaul environment called "fred-env"
source fred-env/bin/activate # Activates the virtual environment
# Your virtual environment should appear in parenthesis, for example: "(fred-env) pi@pi"
```

### Activate Serial Peripheral Interface (SPI)
To toggle SPI on:
1. Click the Raspberry Pi logo on the top left corner.
2. Naviagte to "Preferences", then "Raspberry Configuration".
3. Click "Interfaces" in the pop-up menu.
4. Toggle SPI **on**. 

### Dependencies
To make the FrED work, you need to install some libraries in the Raspberry Pi: (@TODO by Edu team)
Install the following libraries:
```bash
# pip install is the command to install packages
pip install opencv-python PyYAML adafruit-blinka adafruit-circuitpython-mcp3xxx RPi.GPIO numpy matplotlib PyQt5 gpiozero
```

### Collaborating
If you wish to collaborate in making this code better first make sure to have a GitHub account, and set it up in the terminal:
```bash
git config --global user.name "your_username"
git config --global user.email "your@email.address"
```

The GitHub CLI package provides the most lightweight setup for GitHub in the Raspberry Pi, install it with the following commands by the [official guide](https://github.com/cli/cli/blob/trunk/docs/install_linux.md):
```bash
(type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) \
	&& sudo mkdir -p -m 755 /etc/apt/keyrings \
	&& wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
	&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
	&& sudo apt update \
	&& sudo apt install gh -y
```
```bash
sudo apt update
sudo apt install gh
```

Now you can login with your GitHub account in the Raspberry Pi ([official guide](https://docs.github.com/en/enterprise-cloud@latest/github-cli/github-cli/quickstart)):
```bash
gh auth login
```
Follow the screen prompts, selecting `Github.com` for login, and `HTTPS` as the method. Select to open a browser window and login with your account. The browser will ask for a 8 letter code that is currently showing in the terminal.

Now select the branch with your name or issue you are working on, so the code changes can be uploaded to the repository:
```bash
git checkout <your name>
```
**Break things**. You can change the code as you wish, don't worry if something breaks as the safe and working version will always be available at `main` branch. To upload your local changes to the online repository, run in the terminal:
```bash
git add .  # Add your changes to staging phase
git commit -m "Concise message with the changes made"  # Package all the current changes to be uploaded
git push  # Upload to the online repo
```

When you have a **working** new feature, bug fixed, or style changes, go to the browser in your computer, open this repo, enter the **Pull requests** tab, and click **New pull request**.

There you can select your branch as the `compare`, and keep `main` as the base. Type **very detailed** instructions of your change and **Create** it.

Wait for a reviewer to check the change, and you'll receive an email notification if it has been approved or if you have to make modifications.
For smoother review process aim to make **small pull requests**.

You can find further information about the basics of Git and GitHub in this [workshop](https://github.com/RoBorregos/GitCourse)

## Modules

![image](https://github.com/user-attachments/assets/8836b644-5aed-41a6-8425-1bcd4f87cab1)

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
- `get_edges(self, frame: img) -> Tuple[img, img]`: Filter the frame to enhance the edges.
- `get_fiber_diameter(self, lines: List[x0,y0,x1,y1]) -> float`: Get the fiber diameter from the edges detected in the image.
- `plot_lines(self, frame: img, lines: List[x0,y0,x1,y1]) -> None`: Plot the detected lines on the frame.
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
- `add_plots(self) -> Tuple[Plot,Plot,Plot]`: Add plots to the layout.
- `add_diameter_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI spin boxes to control the diameter.
- `add_motor_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI spin boxes to control the motors.
- `add_temperature_controls(self) -> Tuple[QLabel, QSlider, QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]`: Add UI controls for the temperature.
- `add_fan_controls(self) -> Tuple[QLabel, QSlider]`: Add UI controls for the fan.
- `add_buttons(self) -> None`: Add buttons to the layout.
- `start_gui(self) -> None`: Start the GUI.
- `update_temperature_slider_label(self, value) -> None`: Update the temperature slider label.
- `update_fan_slider_label(self, value) -> None`: Update the fan slider label.
- `spooling_control_toggle(self) -> None`: Toggle the spooling control.
- `set_start_device(self) -> None`: Set start device flag.
- `set_calibrate_motor(self) -> None`: Set calibrate motor flag.
- `set_calibrate_camera(self) -> None`: Call calibrate camera.
- `set_download_csv(self) -> None`: Call download csv from database.
- `show_message(self, title: str, message: str) -> None`: Show a message box.
