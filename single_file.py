"""FrED Device Single File"""
import os
import sys
import cv2
from typing import Tuple
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Qt5Agg')  # Use the Qt5Agg backend for matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QGridLayout, QWidget, QDoubleSpinBox, QPushButton, QMessageBox, QLineEdit, QCheckBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap

class Extruder():
    def __init__(self):
        self.speed = 0.0
        self.duty_cycle = 0.0

class DC_Motor():
    def __init__(self):
        self.speed = 0.0
        self.set_speed = 0.0
        self.gain = 0.0
        self.oscillation_period = 0.0

class Steppper_Motor():
    def __init__(self):
        self.speed = 0.0
        self.set_speed = 0.0
        self.gain = 0.0
        self.oscillation_period = 0.0

class Fan():
    def __init__(self):
        self.duty_cycle = 0.0

class FiberCamera(QWidget):
    """Proceess video from camera to obtain the fiber diameter and display it"""
    diameter_coeff = 0.00782324
    use_binary_for_edges = True
    def __init__(self) -> None:
        super().__init__()
        self.raw_image = QLabel()
        self.processed_image = QLabel()
        self.capture = cv2.VideoCapture(0)
        self.line_value_updated = pyqtSignal(float)  # Create a new signal

    def camera_loop(self) -> None:
        """Loop to capture and process frames from the camera"""
        success, frame = self.capture.read()
        assert success, "Failed to capture frame"  # Check if frame is captured

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # To RGB for GUI
        edges, binary_frame = self.get_edges(frame)
        # Get diameter from the binary image
        # TODO: Tune and set to constants for fiber line detection
        detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 80,
                                         minLineLength=60, maxLineGap=10)
        fiber_diameter = self.get_fiber_diameter(detected_lines)
        # Plot lines on the frame
        frame = self.plot_lines(frame, detected_lines)
        # Emit the line_value_updated signal with the new line_value
        #self.line_value_updated.emit(line_value)
        # Update diameter plot
        #diameter_mm_list.append(round(float(line_value), 2))  # Stores diameter values
#             if line_value != 0:
#                 diameter_plot.update_plot(current_time, line_value)

        # Display the frame with lines
        image_for_gui = QImage(frame, frame.shape[1], frame.shape[0],
                                QImage.Format_RGB888)
        self.raw_image.setPixmap(QPixmap(image_for_gui))

        # Binary Image
        image_for_gui = QImage(binary_frame, binary_frame.shape[1],
                               binary_frame.shape[0], QImage.Format_Grayscale8)
        self.processed_image.setPixmap(QPixmap(image_for_gui))

    def get_edges(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Filter the frame to enhance the edges"""
        # Divide the image into 4 horiizontal sections, and keep the middle section
        height, _, _ = frame.shape
        frame = frame[height//4:3*height//4, :]  # Keep the middle section
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gaussian_blurred = cv2.GaussianBlur(gray_frame, (5, 5), 0) 
        threshold_value, binary_frame = cv2.threshold(
            gaussian_blurred, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #print(f'Threshold value: {threshold_value}')

        if FiberCamera.use_binary_for_edges is False:
            edges = cv2.Canny(gray_frame, 100, 250, apertureSize=3)
        else:
            edges = cv2.Canny(binary_frame, 100, 250, apertureSize=3)
        return edges, binary_frame

    def get_fiber_diameter(self, lines):
        """Get the fiber diameter from the edges detected in the image"""
        leftmost_min = sys.maxsize
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0
        if lines is None or len(lines) <= 1:
            return 0
        for line in lines:
            x0, _, x1, _ = line[0]
            # Find if local leftmost is less than the previous leftmost 
            leftmost_min = min(leftmost_min, x0, x1)
            leftmost_max = max(leftmost_max, min(x0, x1))
            rightmost_min = min(rightmost_min, max(x0, x1))
            rightmost_max = max(rightmost_max, x0, x1)

        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min))
                / 2 * FiberCamera.diameter_coeff )

    def plot_lines(self, frame, lines):
        """Plot the detected lines on the frame"""
        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
        return frame

    def closeEvent(self, event):
        self.cap.release()
        event.accept()

class UserInterface():
    """"Graphical User Interface Class"""
    def __init__(self) -> None:
        self.app = QApplication([])
        self.window = QWidget()

        self.title = QLabel("Diameter Close Loop Controls: ")
        self.title.setStyleSheet("font-size: 18px; font-weight: bold;")

        motor_label = QLabel("DC Spooling Motor")
        motor_label.setStyleSheet("font-size: 16px; font-weight: bold;")
    
        motor_plot = self.Plot("DC Motor Speed", "Speed (RPM)")

        temperature_label = QLabel("Temperature")
        temperature_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        temperature_plot = self.Plot("Temperature", "Temperature (C)")

        diameter_label = QLabel("Diameter")
        diameter_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        diameter_plot = self.Plot("Diameter", "Diameter (mm)")
        
        binary_checkbox = QCheckBox("Binary")
        binary_checkbox.setStyleSheet("font-size: 14px;")
        #binary_checkbox.stateChanged.connect(checkbox_state_changed)


        # # # # # # # Third Column # # # # # # 
        # Add the "Spooling Motor Set Speed (RPM)" label
        dc_set_label = QLabel("Spooling Motor Set Speed (RPM)")
        dc_set_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        # Add DC set speed slider
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(20)
        slider.setMaximum(60)
        slider.setValue(50)
    #     slider.valueChanged.connect(slider_value_changed)
        # Add a label to the slider
        slider_value_label = QLabel(str(slider.value()))

        # Add the "Controller Gain" label
        gain_label = QLabel("DC Motor Gain Ku")
        gain_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add Gain slider
        Gainslider = QDoubleSpinBox()
        Gainslider.setMinimum(0.0)
        Gainslider.setMaximum(2.0)
        Gainslider.setValue(0.4)
        Gainslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        Gainslider.setDecimals(1)  # Set the number of decimal places to show
        # Add a label to the slider
        Gainslider_value_label = QLabel(str(Gainslider.value()))

        # Add the "Oscillation Period" label
        Osc_label = QLabel("DC Motor Oscillation Period Tu")
        Osc_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add Oscillation slider
        Oscslider = QDoubleSpinBox()
        Oscslider.setMinimum(0.0)
        Oscslider.setMaximum(2.0)
        Oscslider.setValue(0.9)
        Oscslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        Oscslider.setDecimals(1)  # Set the number of decimal places to show
        # Add a label to the slider
        Oscslider_value_label = QLabel(format(Oscslider.value(), '.1f'))

        # Add the "Extrusion Motor Speed" label
        extr_label = QLabel("Extrusion Motor Speed (RPM)")
        extr_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        # Add Extrusion speed slider
        
        extrslider = QDoubleSpinBox()
        extrslider.setMinimum(0.0)
        extrslider.setMaximum(2.0)
        extrslider.setValue(1.2)
        extrslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        extrslider.setDecimals(1)  # Set the number of decimal places to show
        # Add a label to the slider
        extrslider_value_label = QLabel(format(extrslider.value(), '.1f'))

        # Add the "Temperature (C)" label
        temp_set_label = QLabel("Temperature (C)")
        temp_set_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        # Add Temperature slider
        tempslider = QSlider(Qt.Horizontal)
        tempslider.setMinimum(65)
        tempslider.setMaximum(105)
        tempslider.setValue(95)
        # Add a label to the slider
        tempslider_value_label = QLabel(str(tempslider.value()))

        # Add the "Kp" label
        kp_label = QLabel("Temperature Kp")
        kp_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add kp slider
        kpslider = QDoubleSpinBox()
        kpslider.setMinimum(0.0)
        kpslider.setMaximum(2.0)
        kpslider.setValue(1.4)
        kpslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        kpslider.setDecimals(1)  # Set the number of decimal places to show
    #         self.kpslider.valueChanged.connect(self.slider_value_changed)
        # Add a label to the slider
        kpslider_value_label = QLabel(format(kpslider.value(), '.1f'))

        # Add the "Ki" label
        ki_label = QLabel("Temperature Ki")
        ki_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add ki slider
        kislider = QDoubleSpinBox()
        kislider.setMinimum(0.0)
        kislider.setMaximum(2.0)
        kislider.setValue(0.2)
        kislider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        kislider.setDecimals(1)  # Set the number of decimal places to show
        kislider_value_label = QLabel(format(kislider.value(), '.1f'))

        # Add the "Kd" label
        kd_label = QLabel("Temperature Kd")
        kd_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add kd slider
        kdslider = QDoubleSpinBox()
        kdslider.setMinimum(0.0)
        kdslider.setMaximum(2.0)
        kdslider.setValue(0.8)
        kdslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        kdslider.setDecimals(1)  # Set the number of decimal places to show
        kdslider_value_label = QLabel(format(kdslider.value(), '.1f'))
        
        # Add the target diameter label
        diameter_label = QLabel("Target Diameter (mm)")
        diameter_label.setStyleSheet("font-size: 16px; font-weight: bold;")    
        # Add diameter slider
        diameterslider = QDoubleSpinBox()
        diameterslider.setMinimum(0.3)
        diameterslider.setMaximum(0.6)
        diameterslider.setValue(0.35)
        diameterslider.setSingleStep(0.01)  # Set the step size for decimal adjustment
        diameterslider.setDecimals(2)  # Set the number of decimal places to show
        # Add a label to the slider
        diameterslider_value_label = QLabel(format(diameterslider.value(), '.2f'))
        
        # Add the diameter ku label
        gain_d_label = QLabel("Diameter Gain ku")
        gain_d_label.setStyleSheet("font-size: 14px; font-weight: bold;")    
        # Add diameter slider
        gain_dslider = QDoubleSpinBox()
        gain_dslider.setMinimum(0.1)
        gain_dslider.setMaximum(2)
        gain_dslider.setValue(1.2)
        gain_dslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        gain_dslider.setDecimals(1)  # Set the number of decimal places to show
        # Add a label to the slider
        gain_dslider_value_label = QLabel(format(gain_dslider.value(), '.1f'))

        # Add the diameter ku label
        osc_d_label = QLabel("Diameter Oscillation Period Tu")
        osc_d_label.setStyleSheet("font-size: 14px; font-weight: bold;")    
        # Add diameter slider
        osc_dslider = QDoubleSpinBox()
        osc_dslider.setMinimum(0.1)
        osc_dslider.setMaximum(2)
        osc_dslider.setValue(0.8)
        osc_dslider.setSingleStep(0.1)  # Set the step size for decimal adjustment
        osc_dslider.setDecimals(1)  # Set the number of decimal places to show
        # Add a label to the slider
        osc_dslider_value_label = QLabel(format(osc_dslider.value(), '.1f'))

        # Add the "Fan Duty Cycle" label
        fan_set_label = QLabel("Fan Duty Cycle (%)")
        fan_set_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        # Add fan slider
        fanslider = QSlider(Qt.Horizontal)
        fanslider.setMinimum(0)
        fanslider.setMaximum(100)
        fanslider.setValue(30)
    #         fanslider.valueChanged.connect(self.slider_value_changed)
        # Add a label to the slider
        fanslider_value_label = QLabel(str(fanslider.value()))

        # Add an editable text box
        text_box = QLineEdit()
        text_box.setText("Enter a file name")

        # # # # # # # Fifth Column # # # # # # 
        video_widget = FiberCamera() # Add live video
        
        
        
        button = QPushButton("Download CSV File")
        #button.clicked.connect(print_button_clicked)
        button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
        
        start_diameter_button = QPushButton("Start/stop close loop control")
        #start_diameter_button.clicked.connect(start_diameter_button_clicked)
        start_diameter_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
        
        start_device_button = QPushButton("Start device")
        #start_device_button.clicked.connect(start_device_button_clicked)
        start_device_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
        
        calibrate_motor_button = QPushButton("Calibrate motor")
        #calibrate_motor_button.clicked.connect(calibrate_motor_button_clicked)
        calibrate_motor_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")
        
        calibrate_camera_button = QPushButton("Calibrate camera")
        #calibrate_camera_button.clicked.connect(calibrate_camera_button_clicked)
        calibrate_camera_button.setStyleSheet("background-color: green;font-size: 14px; font-weight: bold;")

        # ~~~~~~~~~~~ Set the Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        layout = QGridLayout()
        # # # # # # # First Column # # # # # # # 
        layout.addWidget(self.title,0,0,1,3) # Add title
    
        # Add the dc motor plot to the layout
        layout.addWidget(motor_plot, 11,0,8,6)
        
        layout.addWidget(start_diameter_button, 10, 0) # Add button
        
        layout.addWidget(start_device_button, 1, 0) # Add button
        
        layout.addWidget(calibrate_motor_button, 1, 1) # Add button
        
        layout.addWidget(calibrate_camera_button, 1, 2) # Add button
        
        layout.addWidget(binary_checkbox,10,1)
        
        # Add the temperature plot to the layout
        layout.addWidget(temperature_plot, 19, 0, 8, 6)   
        layout.addWidget(diameter_plot,2,0,8,6)  # Add the plot to the grid layout

        layout.addWidget(diameter_label,2,6) # Add dc set speed label
        layout.addWidget(diameterslider,3,6) # Add dc set speed slider
        layout.addWidget(diameterslider_value_label,3,7) # Add label to dc set speed slider
        diameterslider.valueChanged.connect(lambda value: diameterslider_value_label.setText(str(value))) # update dc set speed slider
        
        layout.addWidget(gain_d_label,4,6) # Add gain label
        layout.addWidget(gain_dslider,5,6) # Add gain slider
        layout.addWidget(gain_dslider_value_label,5,7) # Add label to gain slider
        gain_dslider.valueChanged.connect(lambda value: gain_dslider_value_label.setText(str(format(value,'.1f')))) # update gain slider
        
        layout.addWidget(osc_d_label,6,6) # Add gain label
        layout.addWidget(osc_dslider,7,6) # Add gain slider
        layout.addWidget(osc_dslider_value_label,7,7) # Add label to gain slider
        osc_dslider.valueChanged.connect(lambda value: osc_dslider_value_label.setText(str(format(value,'.1f'))))

        layout.addWidget(gain_label,8,6) # Add gain label
        layout.addWidget(Gainslider,9,6) # Add gain slider
        layout.addWidget(Gainslider_value_label,9,7) # Add label to gain slider
        Gainslider.valueChanged.connect(lambda value: Gainslider_value_label.setText(str(format(value,'.1f'))))
        
        layout.addWidget(Osc_label,10,6) # Add oscillation label
        layout.addWidget(Oscslider,11,6) # Add oscillation slider
        layout.addWidget(Oscslider_value_label,11,7) # Add label to oscillation slider
        Oscslider.valueChanged.connect(lambda value: Oscslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
        
        layout.addWidget(extr_label,12,6) # Add extrusino speed label
        layout.addWidget(extrslider,13,6) # Add extrusino speed slider
        layout.addWidget(extrslider_value_label,13,7) # Add label to extrusino slider
        extrslider.valueChanged.connect(lambda value: extrslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
        
        layout.addWidget(temp_set_label,14,6) # Add temp label
        layout.addWidget(tempslider,15,6) # Add temp slider
        layout.addWidget(tempslider_value_label,15,7) # Add label to temp slider
        tempslider.valueChanged.connect(lambda value: tempslider_value_label.setText(str(value))) # update dc set speed slider
        
        layout.addWidget(kp_label,16,6) # Add kp label
        layout.addWidget(kpslider,17,6) # Add kp slider
        layout.addWidget(kpslider_value_label,17,7) # Add label to kp slider
        kpslider.valueChanged.connect(lambda value: kpslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
        
        layout.addWidget(ki_label,18,6) # Add ki label
        layout.addWidget(kislider,19,6) # Add ki slider
        layout.addWidget(kislider_value_label,19,7) # Add label to ki slider
        kislider.valueChanged.connect(lambda value: kislider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider

        layout.addWidget(kd_label,20,6) # Add kd label
        layout.addWidget(kdslider,21,6) # Add kd slider
        layout.addWidget(kdslider_value_label,21,7) # Add label to kd slider
        kdslider.valueChanged.connect(lambda value: kdslider_value_label.setText(str(format(value,'.1f')))) # update oscillation slider
        
        layout.addWidget(fan_set_label,22,6) # Add temp label
        layout.addWidget(fanslider,23,6) # Add temp slider
        layout.addWidget(fanslider_value_label,23,7) # Add label to temp slider
        fanslider.valueChanged.connect(lambda value: fanslider_value_label.setText(str(value))) # update dc set speed slider

        layout.addWidget(video_widget.raw_image, 2, 8, 11, 1)  # Add the video_label to the layout

        layout.addWidget(video_widget.processed_image, 13, 8, 11,1)  # Add the video_label to the layout

        layout.addWidget(text_box,24,6) # Add editable text box

        layout.addWidget(button, 24, 8) # Add button
            
        # ~~~~~~ Show the Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self.window.setLayout(layout)
    

        self.window.setWindowTitle("FredBerry Pi - 07252024") # Name of Window
    #     window.resize(800, 1900)  # Set window size (width, height)
        window_width = 1400
        window_height = 800
        self.window.setGeometry(100,100,window_width,window_height)
        self.window.setFixedSize(window_width,window_height)
        self.window.setAutoFillBackground(True)
        
        # ~~~~~~~~~~~ Start threading the GUI and motor control ~~~~~~~~~~~    
        timer = QTimer()
        timer.timeout.connect(video_widget.camera_loop)
        timer.start(50)  # Update every 30 milliseconds
        
        #motor_thread = threading.Thread(target=motor_control_thread)
        #motor_thread.start()
        #threading.Lock()
        
        # Begin application
        self.window.show()
        self.app.exec_()
        
        #motor_thread.join()
    
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

if __name__ == "__main__":
    print("Starting FrED Device...")
    UserInterface()
    print("FrED Device Closed.")
