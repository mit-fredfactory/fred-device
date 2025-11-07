from PyQt5.QtWidgets import (
    QApplication, QWidget, QGridLayout, QLabel, QDoubleSpinBox, QPushButton,
    QMessageBox, QLineEdit, QCheckBox, QSlider
)
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from database import Database
from fiber_camera import FiberCamera

class UserInterface:
    def __init__(self) -> None:
        self.app = QApplication.instance() or QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()

        # Heater and motor control flags
        self.heater_started = False
        self.device_started = False

        # Add plots and controls
        self.diameter_plot = self.add_plots()
        self.target_diameter = self.add_diameter_controls()

        # Add target temperature spinbox for user input
        self.target_temperature_label = QLabel("Target Temperature (°C)")
        self.target_temperature_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        self.target_temperature = QDoubleSpinBox()
        self.target_temperature.setRange(20.0, 300.0)
        self.target_temperature.setValue(95.0)
        self.target_temperature.setSingleStep(0.1)
        self.target_temperature.setDecimals(1)
        self.layout.addWidget(self.target_temperature_label, 1, 7)
        self.layout.addWidget(self.target_temperature, 1, 8)

        # CSV filename input
        self.csv_filename = QLineEdit("Enter a file name")
        self.layout.addWidget(self.csv_filename, 22, 5, 1, 2)

        # Fiber camera setup
        self.fiber_camera = FiberCamera(self.target_diameter, self)
        if self.fiber_camera.diameter_coefficient == -1:
            self.show_message("Camera calibration data not found", "Please calibrate the camera.")
            self.fiber_camera.diameter_coefficient = 0.00782324

        # Raw and processed image labels and widgets
        raw_image_label = QLabel("Raw Image:")
        raw_image_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.layout.addWidget(raw_image_label, 10, 0, 1, 4)
        self.layout.addWidget(self.fiber_camera.raw_image, 11, 0, 6, 4)
        processed_image_label = QLabel("Processed Image:")
        processed_image_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.layout.addWidget(processed_image_label, 17, 0, 1, 4)
        self.layout.addWidget(self.fiber_camera.processed_image, 18, 0, 7, 4)

        # Add buttons including Start Heater and Start Motor buttons
        self.add_buttons()

        # Filter checkboxes
        self.erode_checkbox = QCheckBox("Enable Erode Filter")
        self.erode_checkbox.setChecked(True)
        self.erode_checkbox.stateChanged.connect(self.toggle_erode_filter)
        self.layout.addWidget(self.erode_checkbox, 12, 6, 1, 2)

        self.dilate_checkbox = QCheckBox("Enable Dilate Filter")
        self.dilate_checkbox.setChecked(True)
        self.dilate_checkbox.stateChanged.connect(self.toggle_dilate_filter)
        self.layout.addWidget(self.dilate_checkbox, 13, 6, 1, 2)

        self.gaussian_checkbox = QCheckBox("Enable Gaussian Blur")
        self.gaussian_checkbox.setChecked(True)
        self.gaussian_checkbox.stateChanged.connect(self.toggle_gaussian_filter)
        self.layout.addWidget(self.gaussian_checkbox, 14, 6, 1, 2)

        self.binary_checkbox = QCheckBox("Enable Binary Threshold")
        self.binary_checkbox.setChecked(True)
        self.binary_checkbox.stateChanged.connect(self.toggle_binary_filter)
        self.layout.addWidget(self.binary_checkbox, 15, 6, 1, 2)

        # Sliders for image processing parameters
        self.canny_lower_slider = QSlider(Qt.Horizontal)
        self.canny_lower_slider.setRange(0, 150)
        self.canny_lower_slider.setSingleStep(5)
        self.canny_lower_slider.setTickInterval(5)
        self.canny_lower_slider.setValue(100)
        self.canny_lower_slider.valueChanged.connect(self.update_canny_lower)
        self.canny_lower_value_label = QLabel(str(self.canny_lower_slider.value()))
        self.layout.addWidget(QLabel("Canny Lower"), 16, 5, 1, 1)
        self.layout.addWidget(self.canny_lower_slider, 16, 6, 1, 2)
        self.layout.addWidget(self.canny_lower_value_label, 16, 8)

        self.canny_upper_slider = QSlider(Qt.Horizontal)
        self.canny_upper_slider.setRange(150, 300)
        self.canny_upper_slider.setSingleStep(5)
        self.canny_upper_slider.setTickInterval(5)
        self.canny_upper_slider.setValue(250)
        self.canny_upper_slider.valueChanged.connect(self.update_canny_upper)
        self.canny_upper_value_label = QLabel(str(self.canny_upper_slider.value()))
        self.layout.addWidget(QLabel("Canny Upper"), 17, 5, 1, 1)
        self.layout.addWidget(self.canny_upper_slider, 17, 6, 1, 2)
        self.layout.addWidget(self.canny_upper_value_label, 17, 8)

        self.hough_threshold_slider = QSlider(Qt.Horizontal)
        self.hough_threshold_slider.setRange(10, 100)
        self.hough_threshold_slider.setSingleStep(5)
        self.hough_threshold_slider.setTickInterval(5)
        self.hough_threshold_slider.setValue(30)
        self.hough_threshold_slider.valueChanged.connect(self.update_hough_threshold)
        self.hough_threshold_value_label = QLabel(str(self.hough_threshold_slider.value()))
        self.layout.addWidget(QLabel("Hough Threshold"), 18, 5, 1, 1)
        self.layout.addWidget(self.hough_threshold_slider, 18, 6, 1, 2)
        self.layout.addWidget(self.hough_threshold_value_label, 18, 8)

        # Set layout stretch
        for col in range(10):
            self.layout.setColumnStretch(col, 1)
        for row in range(24):
            self.layout.setRowStretch(row, 1)

        self.window.setLayout(self.layout)
        self.window.setWindowTitle("MIT FrED")
        self.window.setGeometry(100, 100, 1600, 1000)
        self.window.setFixedSize(1600, 1000)
        self.window.setAutoFillBackground(True)

    def add_buttons(self):
        self.add_heater_button(0, 5)
        self.add_motor_button(1, 5)
        self.create_button("Calibrate camera", self.set_calibrate_camera, 0, 4)
        self.create_button("Start Ploting", self.set_camera_feedback, 0, 9)
        self.create_button("Download CSV File", self.set_download_csv, 22, 7)
        self.create_button("Exit", self.exit_program, 22, 9)

    def add_heater_button(self, row, col):
        self.heater_button = QPushButton("Start Heater")
        self.heater_button.setStyleSheet("background-color: green; font-size: 14px; font-weight: bold;")
        self.heater_button.setCheckable(True)
        self.heater_button.clicked.connect(self.toggle_heater)
        self.layout.addWidget(self.heater_button, row, col)

    def toggle_heater(self):
        self.heater_started = not self.heater_started
        if self.heater_started:
            self.heater_button.setText("Stop Heater")
            self.heater_button.setStyleSheet("background-color: red; font-size: 14px; font-weight: bold;")
            QMessageBox.information(self.window, "Heater Started", f"Heater started. Target: {self.target_temperature.value():.1f} °C")
        else:
            self.heater_button.setText("Start Heater")
            self.heater_button.setStyleSheet("background-color: green; font-size: 14px; font-weight: bold;")
            QMessageBox.information(self.window, "Heater Stopped", "Heater stopped.")

    def add_motor_button(self, row, col):
        self.motor_button = QPushButton("Start Motor")
        self.motor_button.setStyleSheet("background-color: green; font-size: 14px; font-weight: bold;")
        self.motor_button.setCheckable(True)
        self.motor_button.clicked.connect(self.toggle_motor)
        self.layout.addWidget(self.motor_button, row, col)

    def toggle_motor(self):
        self.device_started = not self.device_started
        if self.device_started:
            self.motor_button.setText("Stop Motor")
            self.motor_button.setStyleSheet("background-color: red; font-size: 14px; font-weight: bold;")
            QMessageBox.information(self.window, "Motor Started", "Motor started.")
        else:
            self.motor_button.setText("Start Motor")
            self.motor_button.setStyleSheet("background-color: green; font-size: 14px; font-weight: bold;")
            QMessageBox.information(self.window, "Motor Stopped", "Motor stopped.")

    def create_button(self, text, handler, row, col, obj_attr_name=None):
        btn = QPushButton(text)
        btn.setStyleSheet("background-color: green; font-size: 14px; font-weight: bold;")
        btn.clicked.connect(handler)
        self.layout.addWidget(btn, row, col)
        if obj_attr_name:
            setattr(self, obj_attr_name, btn)

    def update_canny_lower(self, value):
        self.fiber_camera.canny_lower = value
        self.canny_lower_value_label.setText(str(value))

    def update_canny_upper(self, value):
        self.fiber_camera.canny_upper = value
        self.canny_upper_value_label.setText(str(value))

    def update_hough_threshold(self, value):
        self.fiber_camera.hough_threshold = value
        self.hough_threshold_value_label.setText(str(value))

    def toggle_erode_filter(self, state):
        FiberCamera.use_erode = (state == 2)

    def toggle_dilate_filter(self, state):
        FiberCamera.use_dilate = (state == 2)

    def toggle_gaussian_filter(self, state):
        FiberCamera.use_gaussian = (state == 2)

    def toggle_binary_filter(self, state):
        FiberCamera.use_binary = (state == 2)

    def add_plots(self):
        diameter_plot = self.Plot("Diameter", "Diameter (mm)")
        self.layout.addWidget(diameter_plot, 0, 0, 10, 4)
        return diameter_plot

    def add_diameter_controls(self):
        label = QLabel("Target Diameter (mm)")
        label.setStyleSheet("font-size: 16px; font-weight: bold;")
        spin = QDoubleSpinBox()
        spin.setRange(0.3, 0.6)
        spin.setValue(0.35)
        spin.setSingleStep(0.01)
        spin.setDecimals(2)
        self.layout.addWidget(label, 0, 7)
        self.layout.addWidget(spin, 0, 8)
        return spin

    def start_gui(self) -> None:
        timer = QTimer()
        timer.timeout.connect(self.fiber_camera.camera_loop)
        timer.start(200)
        self.window.show()
        self.app.exec_()

    def set_camera_feedback(self) -> None:
        self.camera_feedback_enabled = not self.camera_feedback_enabled
        msg = "started" if self.camera_feedback_enabled else "stopped"
        QMessageBox.information(self.window, "Camera Feedback", f"Camera feedback {msg}.")

    def set_calibrate_camera(self) -> None:
        QMessageBox.information(self.window, "Camera Calibration", "Camera is calibrating.")
        self.fiber_camera.calibrate()
        QMessageBox.information(self.window, "Calibration", "Camera calibration completed.")

    def set_download_csv(self) -> None:
        QMessageBox.information(self.window, "Download CSV", "Downloading CSV file.")
        Database.generate_csv(self.csv_filename.text())

    def exit_program(self) -> None:
        self.window.close()
        self.app.quit()

    def show_message(self, title: str, message: str) -> None:
        QMessageBox.information(self.window, title, message)

    class Plot(FigureCanvas):
        def __init__(self, title: str, y_label: str) -> None:
            self.figure = Figure()
            self.axes = self.figure.add_subplot(111)
            super().__init__(self.figure)
            self.figure.subplots_adjust(top=0.92)
            self.axes.set_title(title)
            self.axes.set_xlabel("Time (s)")
            self.axes.set_ylabel(y_label)
            self.progress_line, = self.axes.plot([], [], lw=2, label=title)
            self.setpoint_line, = self.axes.plot([], [], lw=2, color='r', label=f'Target {title}')
            self.axes.legend()
            self.x_data = []
            self.y_data = []
            self.setpoint_data = []

        def update_plot(self, x: float, y: float, setpoint: float) -> None:
            max_points = 100
            self.x_data.append(x)
            self.y_data.append(y)
            self.setpoint_data.append(setpoint)
            self.x_data = self.x_data[-max_points:]
            self.y_data = self.y_data[-max_points:]
            self.setpoint_data = self.setpoint_data[-max_points:]
            self.progress_line.set_data(self.x_data, self.y_data)
            self.setpoint_line.set_data(self.x_data, self.setpoint_data)
            self.axes.relim()
            self.axes.autoscale_view()
            self.draw()
