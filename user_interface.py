"""File to setup the layout of the User Interface"""
from typing import Tuple
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QDoubleSpinBox, QSlider, QPushButton, QMessageBox, QLineEdit, QCheckBox 
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from database import Database
from fiber_camera import FiberCamera
 
class UserInterface():
    """"Graphical User Interface Class"""
    def __init__(self) -> None:
        self.app = QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()

        self.motor_plot, self.temperature_plot, self.diameter_plot \
            = self.add_plots()

        self.target_diameter, self.diameter_gain, \
            self.diameter_oscilation_period = self.add_diameter_controls()

        self.motor_gain, self.motor_oscilation_period, \
            self.extrusion_motor_speed = self.add_motor_controls()

        self.target_temperature_label, self.target_temperature, \
            self.temperature_kp, self.temperature_ki, self.temperature_kd \
            = self.add_temperature_controls()

        self.fan_duty_cycle_label, self.fan_duty_cycle = self.add_fan_controls()

        # Editable text box for the CSV file name
        self.csv_filename = QLineEdit()
        self.csv_filename.setText("Enter a file name")
        self.layout.addWidget(self.csv_filename, 24, 8)

        self.spooling_control_state = False
        self.device_started = False
        self.start_motor_calibration = False

        self.fiber_camera = FiberCamera(self.target_diameter)
        if self.fiber_camera.diameter_coefficient == -1:
            self.show_message("Camera calibration data not found",
                              "Please calibrate the camera.")
            self.fiber_camera.diameter_coefficient = 0.00782324
        self.layout.addWidget(self.fiber_camera.raw_image, 2, 8, 11, 1)
        self.layout.addWidget(self.fiber_camera.processed_image, 13, 8, 11, 1)

        self.add_buttons()
        
        self.window.setLayout(self.layout)
        self.window.setWindowTitle("MIT FrED")
        self.window.setGeometry(100, 100, 1600, 1000)
        self.window.setFixedSize(1600, 1000)
        self.window.setAutoFillBackground(True)

    def add_plots(self):
        """Add plots to the layout"""
        font_style = "font-size: 16px; font-weight: bold;"
        binary_checkbox = QCheckBox("Binary")
        binary_checkbox.setStyleSheet(font_style)
        #binary_checkbox.stateChanged.connect(checkbox_state_changed) TODO

        motor_plot = self.Plot("DC Spooling Motor", "Speed (RPM)")
        temperature_plot = self.Plot("Temperature", "Temperature (C)")
        diameter_plot = self.Plot("Diameter", "Diameter (mm)")

        self.layout.addWidget(binary_checkbox, 10, 1)
        self.layout.addWidget(diameter_plot, 2, 0, 8, 4)
        self.layout.addWidget(motor_plot, 11, 0, 8, 4)
        self.layout.addWidget(temperature_plot, 19, 0, 8, 4)

        return motor_plot, temperature_plot, diameter_plot

    def add_diameter_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox,
                                             QDoubleSpinBox]:
        """Add UI spin boxes to control the diameter"""
        font_style = "font-size: %ipx; font-weight: bold;"
        target_diameter_label = QLabel("Target Diameter (mm)")
        target_diameter_label.setStyleSheet(font_style % 16)
        target_diameter = QDoubleSpinBox()
        target_diameter.setMinimum(0.3)
        target_diameter.setMaximum(0.6)
        target_diameter.setValue(0.35)
        target_diameter.setSingleStep(0.01)
        target_diameter.setDecimals(2)

        diameter_gain_label = QLabel("Diameter Gain Ku")
        diameter_gain_label.setStyleSheet(font_style % 14)
        diameter_gain = QDoubleSpinBox()
        diameter_gain.setMinimum(0.1)
        diameter_gain.setMaximum(2)
        diameter_gain.setValue(1.2)
        diameter_gain.setSingleStep(0.1)
        diameter_gain.setDecimals(1)

        diameter_oscilation_period_label = QLabel("Diameter Oscillation Period Tu")
        diameter_oscilation_period_label.setStyleSheet(font_style % 14)
        diameter_oscilation_period = QDoubleSpinBox()
        diameter_oscilation_period.setMinimum(0.1)
        diameter_oscilation_period.setMaximum(2)
        diameter_oscilation_period.setValue(0.8)
        diameter_oscilation_period.setSingleStep(0.1)
        diameter_oscilation_period.setDecimals(1)

        self.layout.addWidget(target_diameter_label, 2, 6)
        self.layout.addWidget(target_diameter, 3,6)
        self.layout.addWidget(diameter_gain_label, 4, 6)
        self.layout.addWidget(diameter_gain, 5, 6)
        self.layout.addWidget(diameter_oscilation_period_label, 6, 6)
        self.layout.addWidget(diameter_oscilation_period, 7, 6)
        return target_diameter, diameter_gain, diameter_oscilation_period

    def add_motor_controls(self) -> Tuple[QDoubleSpinBox, QDoubleSpinBox,
                                          QDoubleSpinBox]:
        """Add UI spin boxes to control the motors"""
        font_style = "font-size: %ipx; font-weight: bold;"
        motor_gain_label = QLabel("DC Motor Gain Ku")
        motor_gain_label.setStyleSheet(font_style % 14)
        motor_gain = QDoubleSpinBox()
        motor_gain.setMinimum(0.0)
        motor_gain.setMaximum(2.0)
        motor_gain.setValue(0.4)
        motor_gain.setSingleStep(0.1)
        motor_gain.setDecimals(1)

        motor_oscilation_period_label = QLabel("DC Motor Oscillation Period Tu")
        motor_oscilation_period_label.setStyleSheet(font_style % 14)
        motor_oscilation_period = QDoubleSpinBox()
        motor_oscilation_period.setMinimum(0.0)
        motor_oscilation_period.setMaximum(2.0)
        motor_oscilation_period.setValue(0.9)
        motor_oscilation_period.setSingleStep(0.1)
        motor_oscilation_period.setDecimals(1)

        extrusion_motor_speed_label = QLabel("Extrusion Motor Speed (RPM)")
        extrusion_motor_speed_label.setStyleSheet(font_style % 16)
        extrusion_motor_speed = QDoubleSpinBox()
        extrusion_motor_speed.setMinimum(0.0)
        extrusion_motor_speed.setMaximum(2.0)
        extrusion_motor_speed.setValue(1.2)
        extrusion_motor_speed.setSingleStep(0.1)
        extrusion_motor_speed.setDecimals(1)

        self.layout.addWidget(motor_gain_label, 8, 6)
        self.layout.addWidget(motor_gain, 9, 6)
        self.layout.addWidget(motor_oscilation_period_label, 10, 6)
        self.layout.addWidget(motor_oscilation_period, 11, 6)
        self.layout.addWidget(extrusion_motor_speed_label, 12, 6)
        self.layout.addWidget(extrusion_motor_speed, 13, 6)
        return motor_gain, motor_oscilation_period, extrusion_motor_speed

    def add_temperature_controls(self) -> Tuple[QLabel, QSlider, QDoubleSpinBox,
                                                  QDoubleSpinBox, QDoubleSpinBox]:
        """Add UI controls for the temperature"""
        font_style = "font-size: %ipx; font-weight: bold;"
        target_temperature_label = QLabel("Temperature (C)")
        target_temperature_label.setStyleSheet(font_style % 16)
        target_temperature = QSlider(Qt.Horizontal)
        target_temperature.setMinimum(65)
        target_temperature.setMaximum(105)
        target_temperature.setValue(95)
        target_temperature.valueChanged.connect(self.update_temperature_slider_label)

        temperature_kp_label = QLabel("Temperature Kp")
        temperature_kp_label.setStyleSheet(font_style % 14)
        temperature_kp = QDoubleSpinBox()
        temperature_kp.setMinimum(0.0)
        temperature_kp.setMaximum(2.0)
        temperature_kp.setValue(1.4)
        temperature_kp.setSingleStep(0.1)
        temperature_kp.setDecimals(5)

        temperature_ki_label = QLabel("Temperature Ki")
        temperature_ki_label.setStyleSheet(font_style % 14)
        temperature_ki = QDoubleSpinBox()
        temperature_ki.setMinimum(0.0)
        temperature_ki.setMaximum(2.0)
        temperature_ki.setValue(0.2)
        temperature_ki.setSingleStep(0.1)
        temperature_ki.setDecimals(5)

        temperature_kd_label = QLabel("Temperature Kd")
        temperature_kd_label.setStyleSheet(font_style % 14)
        temperature_kd = QDoubleSpinBox()
        temperature_kd.setMinimum(0.0)
        temperature_kd.setMaximum(2.0)
        temperature_kd.setValue(0.8)
        temperature_kd.setSingleStep(0.1)
        temperature_kd.setDecimals(5)

        self.layout.addWidget(target_temperature_label, 14, 6)
        self.layout.addWidget(target_temperature, 15, 6)
        self.layout.addWidget(temperature_kp_label, 16, 6)
        self.layout.addWidget(temperature_kp, 17, 6)
        self.layout.addWidget(temperature_ki_label, 18, 6)
        self.layout.addWidget(temperature_ki, 19, 6)
        self.layout.addWidget(temperature_kd_label, 20, 6)
        self.layout.addWidget(temperature_kd, 21, 6)

        return target_temperature_label, target_temperature, temperature_kp, \
            temperature_ki, temperature_kd

    def add_fan_controls(self) -> Tuple[QLabel, QSlider]:
        """Add UI controls for the fan"""
        font_style = "font-size: %ipx; font-weight: bold;"
        fan_duty_cycle_label = QLabel("Fan Duty Cycle (%)")
        fan_duty_cycle_label.setStyleSheet(font_style % 14)
        fan_duty_cycle = QSlider(Qt.Horizontal)
        fan_duty_cycle.setMinimum(0)
        fan_duty_cycle.setMaximum(100)
        fan_duty_cycle.setValue(30)
        fan_duty_cycle.valueChanged.connect(self.update_fan_slider_label)

        self.layout.addWidget(fan_duty_cycle_label, 22, 6)
        self.layout.addWidget(fan_duty_cycle, 23, 6)

        return fan_duty_cycle_label, fan_duty_cycle

    def add_buttons(self):
        """Add buttons to the layout"""
        font_style = "background-color: green; font-size: 14px; font-weight: bold;"
        spooling_control = QPushButton("Start/stop spooling close loop control")
        spooling_control.setStyleSheet(font_style)
        spooling_control.clicked.connect(self.spooling_control_toggle)
        start_device = QPushButton("Start device")
        start_device.setStyleSheet(font_style)
        start_device.clicked.connect(self.set_start_device)
        calibrate_motor = QPushButton("Calibrate motor")
        calibrate_motor.setStyleSheet(font_style)
        calibrate_motor.clicked.connect(self.set_calibrate_motor)
        calibrate_camera = QPushButton("Calibrate camera")
        calibrate_camera.setStyleSheet(font_style)
        calibrate_camera.clicked.connect(self.set_calibrate_camera)
        download_csv = QPushButton("Download CSV File")
        download_csv.setStyleSheet(font_style)
        download_csv.clicked.connect(self.set_download_csv)

        self.layout.addWidget(spooling_control, 10, 0)
        self.layout.addWidget(start_device, 1, 0)
        self.layout.addWidget(calibrate_motor, 1, 1)
        self.layout.addWidget(calibrate_camera, 1, 2)
        self.layout.addWidget(download_csv, 24, 6)

    def start_gui(self) -> None:
        """Start the GUI"""
        timer = QTimer()
        timer.timeout.connect(self.fiber_camera.camera_loop)
        timer.start(200)

        self.window.show()
        self.app.exec_()

    def update_temperature_slider_label(self, value) -> None:
        """Update the temperature slider label"""
        self.target_temperature_label.setText(f"Temperature: {value} C")
    
    def update_fan_slider_label(self, value) -> None:
        """Update the fan slider label"""
        self.fan_duty_cycle_label.setText(f"Fan Duty Cycle: {value} %")

    def spooling_control_toggle(self) -> None:
        """Toggle the spooling control"""
        self.spooling_control_state = not self.spooling_control_state
        if self.spooling_control_state:
            QMessageBox.information(self.app.activeWindow(), "Spooling Control",
                                    "Spooling control started.")
        else:
            QMessageBox.information(self.app.activeWindow(), "Spooling Control",
                                    "Spooling control stopped.")

    def set_start_device(self) -> None:
        """Set start device flag"""
        QMessageBox.information(self.app.activeWindow(), "Device Start",
                                "Device is starting.")
        self.device_started = True

    def set_calibrate_motor(self) -> None:
        """Set calibrate motor flag"""
        QMessageBox.information(self.app.activeWindow(), "Motor Calibration",
                                "Motor is calibrating.")
        self.start_motor_calibration = True

    def set_calibrate_camera(self) -> None:
        """Call calibrate camera"""
        QMessageBox.information(self.app.activeWindow(), "Camera Calibration",
                                "Camera is calibrating.")
        self.fiber_camera.calibrate()
        QMessageBox.information(self.app.activeWindow(),
                                "Calibration", "Camera calibration completed. "
                                "Please restart the program.")    

    def set_download_csv(self) -> None:
        """Call download csv from database"""
        QMessageBox.information(self.app.activeWindow(), "Download CSV",
                                "Downloading CSV file.")
        Database.generate_csv(self.csv_filename.text())

    def show_message(self, title: str, message: str) -> None:
        """Show a message box"""
        QMessageBox.information(self.app.activeWindow(), title, message)

    class Plot(FigureCanvas):
        """Base class for plots"""
        def __init__(self, title: str, y_label: str) -> None:
            self.figure = Figure()
            self.axes = self.figure.add_subplot(111)
            # 1x1 grid, first subplot: https://stackoverflow.com/a/46986694
            super(UserInterface.Plot, self).__init__(self.figure)

            self.axes.set_title(title)
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel(y_label)

            self.progress_line, = self.axes.plot([], [], lw=2, label=title)
            self.setpoint_line, = self.axes.plot([], [], lw=2, color='r',
                                                 label=f'Target {title}')
            self.axes.legend()
            self.axes.grid(axis = 'y')

            self.x_data = []
            self.y_data = []
            self.setpoint_data = []

        def update_plot(self, x: float, y: float, setpoint: float) -> None:
            """Update the plot"""
            self.x_data.append(x)
            self.y_data.append(y)
            self.setpoint_data.append(setpoint)

            self.progress_line.set_data(self.x_data, self.y_data)
            self.setpoint_line.set_data(self.x_data, self.setpoint_data)

            self.axes.relim()
            self.axes.autoscale_view()
            self.draw()
