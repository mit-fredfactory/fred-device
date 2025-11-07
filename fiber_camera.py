import time  # Used for getting the current time, to track when frames are processed
import sys  # Used for system-specific parameters and functions, like getting the largest integer
import cv2  # OpenCV library for image and video processing
import numpy as np  # NumPy for numerical operations, such as creating arrays and kernels
from typing import Tuple  # Used for type hinting, to specify function return types
from PyQt5.QtWidgets import QWidget, QLabel, QDoubleSpinBox, QInputDialog, QMessageBox  # PyQt5 widgets for GUI
from PyQt5.QtGui import QImage, QPixmap  # PyQt5 classes for handling and displaying images

from database import Database  # Custom module to handle data storage and calibration
from typing import TYPE_CHECKING  # Used to avoid circular imports when type hinting
if TYPE_CHECKING:
    from user_interface import UserInterface  # Only imported for type hints, not at runtime

class FiberCamera(QWidget):
    # These are class-level flags to turn on/off certain image processing steps
    use_binary_for_edges = True  # If True, use the binary image for edge detection, else use grayscale
    use_erode = True  # If True, apply erosion to remove small noise
    use_dilate = True  # If True, apply dilation to restore eroded features
    use_gaussian = True  # If True, apply Gaussian blur to smooth the image
    use_binary = True  # If True, apply binary thresholding to the image

    def __init__(self, target_diameter: QDoubleSpinBox, gui: 'UserInterface') -> None:
        super().__init__()
        self.raw_image = QLabel()  # Label to show the original camera image in the GUI
        self.canny_image = QLabel()  # Label to show the edge-detected image in the GUI
        self.processed_image = QLabel()  # Label to show the processed (binary) image in the GUI
        self.target_diameter = target_diameter  # Spin box widget for user to set target diameter
        self.capture = cv2.VideoCapture(0)  # Open the default camera (0 means first webcam)
        self.gui = gui  # Reference to the main GUI for interaction
        self.diameter_coefficient = Database.get_calibration_data("diameter_coefficient")  # Calibration factor to convert pixels to mm
        self.previous_time = 0.0  # Store the time of the last frame for timing calculations
        self.canny_lower = 100  # Lower threshold for Canny edge detection, 100 is a common default
        self.canny_upper = 250  # Upper threshold for Canny edge detection, 250 is chosen to detect strong edges
        self.hough_threshold = 30  # Threshold for Hough line detection, 30 means a line must have at least 30 points

    def camera_loop(self) -> None:
        # Main loop to capture, process, and display camera images
        current_time = time.time()  # Get current time for data logging
        success, frame = self.capture.read()  # Capture a frame from the camera
        if not success:
            print("Failed to capture frame")
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR (OpenCV default) to RGB for display
        height, _, _ = frame.shape  # Get image height
        frame = frame[height//4:3*height//4, :]  # Crop to the central half vertically for better focus

        edges, binary_frame = self.get_edges(frame)  # Get edge and binary images for processing

        detected_lines = cv2.HoughLinesP(
            edges,  # Input edge image
            1,  # Distance resolution of the accumulator in pixels, 1 means every pixel is checked
            np.pi / 180,  # Angle resolution in radians, np.pi/180 is 1 degree
            self.hough_threshold,  # Minimum number of intersections to detect a line
            minLineLength=30,  # Minimum length of a line in pixels, 30 filters out short lines
            maxLineGap=100  # Maximum allowed gap between line segments to treat them as a single line
        )

        fiber_diameter = self.get_fiber_diameter(detected_lines)  # Calculate fiber diameter from detected lines

        frame = self.plot_lines(frame, detected_lines)  # Draw lines on the original frame for visualization

        # Store data for analysis and plotting
        Database.camera_timestamps.append(current_time)  # Save timestamp of this frame
        Database.diameter_readings.append(fiber_diameter)  # Save measured diameter
        if self.target_diameter is not None:
            Database.diameter_setpoint.append(self.target_diameter.value())  # Save target diameter set by user
        else:
            Database.diameter_setpoint.append(0)  # If not set, use 0
        Database.diameter_delta_time.append(current_time - self.previous_time)  # Time since last frame
        self.previous_time = current_time  # Update last frame time

        # Convert processed images to QImage and display in GUI labels
        image_for_gui = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        self.raw_image.setPixmap(QPixmap(image_for_gui))

        image_for_gui = QImage(edges, edges.shape[1], edges.shape[0], QImage.Format_Grayscale8)
        self.canny_image.setPixmap(QPixmap(image_for_gui))

        image_for_gui = QImage(binary_frame, binary_frame.shape[1], binary_frame.shape[0], QImage.Format_Grayscale8)
        self.processed_image.setPixmap(QPixmap(image_for_gui))

    def get_edges(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # Convert the input frame to grayscale for processing
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        kernel = np.ones((5,5), np.uint8)  # 5x5 square kernel for morphological operations, size chosen for moderate effect

        if FiberCamera.use_erode:
            frame = cv2.erode(frame, kernel, iterations=2)  # Erode twice to remove small white noise

        if FiberCamera.use_dilate:
            frame = cv2.dilate(frame, kernel, iterations=2)  # Dilate twice to restore objects eroded too much

        if FiberCamera.use_gaussian:
            frame = cv2.GaussianBlur(frame, (5, 5), 0)  # Apply Gaussian blur with 5x5 kernel, 0 means auto-calculate sigma

        if FiberCamera.use_binary:
            _, binary_frame = cv2.threshold(frame, 100, 255, cv2.THRESH_BINARY)  # Threshold at 100: below=0, above=255
        else:
            binary_frame = frame.copy()  # If not using binary, just copy the grayscale

        lower = getattr(self, "canny_lower", 100)  # Use object's canny_lower, default 100
        upper = getattr(self, "canny_upper", 250)  # Use object's canny_upper, default 250

        if FiberCamera.use_binary_for_edges is False:
            edges = cv2.Canny(frame, lower, upper, apertureSize=3)  # Use grayscale for edge detection
        else:
            edges = cv2.Canny(binary_frame, lower, upper, apertureSize=3)  # Use binary image for edge detection

        return edges, binary_frame  # Return both edge and binary images

    def get_fiber_diameter(self, lines):
        # This function estimates the fiber diameter using detected lines
        leftmost_min = sys.maxsize  # Initialize to largest possible integer
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0

        if lines is None or len(lines) <= 1:  # If no or only one line found, can't compute diameter
            return 0

        for line in lines:
            x0, _, x1, _ = line[0]  # Get x-coordinates of the line endpoints
            leftmost_min = min(leftmost_min, x0, x1)  # Smallest x (left edge)
            leftmost_max = max(leftmost_max, min(x0, x1))  # Largest of the smaller x's (right edge of leftmost lines)
            rightmost_min = min(rightmost_min, max(x0, x1))  # Smallest of the larger x's (left edge of rightmost lines)
            rightmost_max = max(rightmost_max, x0, x1)  # Largest x (right edge)

        # Average the width between leftmost and rightmost lines, then multiply by calibration coefficient
        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min)) / 2 * self.diameter_coefficient)

    def get_fiber_diameter_noC(self, lines):
        # Same as get_fiber_diameter, but without applying calibration coefficient (for calibration step)
        leftmost_min = sys.maxsize
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0
        if lines is None or len(lines) <= 1:
            return 0
        for line in lines:
            x0, _, x1, _ = line[0]
            leftmost_min = min(leftmost_min, x0, x1)
            leftmost_max = max(leftmost_max, min(x0, x1))
            rightmost_min = min(rightmost_min, max(x0, x1))
            rightmost_max = max(rightmost_max, x0, x1)
        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min)) / 2)

    def plot_lines(self, frame, lines):
        # Draw detected lines on the frame in blue for visualization
        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)  # Draw line in blue (BGR: 255,0,0), thickness=2
        return frame

    def calibrate(self):
        # Calibrate the system by comparing measured pixels to actual diameter
        num_samples = 50  # Number of frames to average for calibration, more samples = more stable result
        accumulated_diameter = 0  # Sum of measured diameters in pixels
        valid_samples = 0  # Number of valid samples taken

        diameter_mm, ok = QInputDialog.getDouble(
            self.gui.window,
            "Calibrate Camera",
            "Enter wire actual diameter (mm) for calibration",  # Prompt the user for real diameter
            decimals=4,  # Allow up to 4 decimal places for accuracy
            min=0.01,  # Minimum allowed value, prevents zero or negative input
            max=100.0  # Maximum allowed value, prevents unrealistic input
        )
        if not ok:  # If user cancels, exit calibration
            return

        for _ in range(num_samples):  # Take multiple samples for averaging
            success, frame = self.capture.read()
            if not success:
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            edges, _ = self.get_edges(frame)
            detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, self.hough_threshold, minLineLength=30, maxLineGap=100)
            fiber_diameter = self.get_fiber_diameter_noC(detected_lines)
            if fiber_diameter > 0:  # Only count valid measurements
                accumulated_diameter += fiber_diameter
                valid_samples += 1

        if valid_samples > 0:
            average_diameter_px = accumulated_diameter / valid_samples  # Average diameter in pixels
        else:
            average_diameter_px = 1  # Prevent division by zero

        self.diameter_coefficient = diameter_mm / average_diameter_px  # Calculate calibration factor (mm per pixel)
        Database.update_calibration_data("diameter_coefficient", str(self.diameter_coefficient))  # Save to database
        QMessageBox.information(self.gui.window, "Calibration", "Diameter calibration done.")  # Notify user

    def camera_feedback(self, current_time: float) -> None:
        # Capture a frame, process it, and update the GUI and database with the result
        try:
            success, frame = self.capture.read()
            if not success:
                return
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, _, _ = frame.shape
            frame = frame[height//4:3*height//4, :]
            edges, binary_frame = self.get_edges(frame)
            detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, self.hough_threshold, minLineLength=30, maxLineGap=100)
            current_diameter = self.get_fiber_diameter(detected_lines)
            if self.gui.diameter_plot:
                self.gui.diameter_plot.update_plot(current_time, current_diameter, self.target_diameter.value())  # Update plot
            Database.camera_timestamps.append(current_time)
            Database.diameter_readings.append(current_diameter)
            Database.diameter_setpoint.append(self.target_diameter.value())
            Database.diameter_delta_time.append(current_time - self.previous_time)
            self.previous_time = current_time
        except Exception as e:
            print(f"Error in camera feedback: {e}")

    def closeEvent(self, event):
        # Release the camera resource when the widget is closed to free hardware
        self.capture.release()
        event.accept()
