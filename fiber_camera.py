"""Module to process video from camera to obtain the fiber diameter and display it"""
import time
import sys
import cv2
from PyQt5.QtWidgets import QWidget, QLabel, QImage, QPixmap, QDoubleSpinBox, QCheckBox, QTimer, QApplication

from database import Database

class FiberCamera(QWidget):
    """Proceess video from camera to obtain the fiber diameter and display it"""
    use_binary_for_edges = True
    def __init__(self, target_diameter: QDoubleSpinBox) -> None:
        super().__init__()
        self.raw_image = QLabel()
        self.processed_image = QLabel()
        self.target_diameter = target_diameter
        self.capture = cv2.VideoCapture(0)
        self.line_value_updated = pyqtSignal(float)  # Create a new signal
        self.diameter_coefficient = Database.get_calibration_data(
            "diameter_coefficient")
        self.previous_time = 0.0

    def camera_loop(self) -> None:
        """Loop to capture and process frames from the camera"""
        success, frame = self.capture.read()
        assert success, "Failed to capture frame"  # Check if frame is captured

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # To RGB for GUI
        height, _, _ = frame.shape
        frame = frame[height//4:3*height//4, :]  # Keep the middle section
        edges, binary_frame = self.get_edges(frame)
        # Get diameter from the binary image
        # TODO: Tune and set to constants for fiber line detection
        detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50,
                                         minLineLength=80, maxLineGap=20)
        fiber_diameter = self.get_fiber_diameter(detected_lines)
        # Plot lines on the frame
        frame = self.plot_lines(frame, detected_lines)
        # Emit the line_value_updated signal with the new line_value
        #self.line_value_updated.emit(line_value)
        Database.diameter_delta_time.append(time.time() - self.previous_time)
        self.previous_time = time.time()
        Database.diameter_readings.append(fiber_diameter)
        Database.diameter_setpoint.append(self.target_diameter.value())

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
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Gray
        kernel = np.ones((5,5), np.uint8)
        frame = cv2.erode(frame, kernel, iterations=2)
        frame = cv2.dilate(frame, kernel, iterations=1)
        gaussian_blurred = cv2.GaussianBlur(frame, (5, 5), 0) 
        threshold_value, binary_frame = cv2.threshold(
            gaussian_blurred, 127, 255, cv2.THRESH_BINARY)
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
                / 2 * self.diameter_coefficient )

    def plot_lines(self, frame, lines):
        """Plot the detected lines on the frame"""
        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
        return frame

    def calibrate(self):
        """Calibrate the camera"""
        num_samples = 20
        accumulated_diameter = 0
        average_diameter = 0
        valid_samples = 0

        for _ in range(num_samples):
            success, frame = self.capture.read()
            assert success, "Failed to capture frame"  # Check if frame is captured
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            edges, _ = self.get_edges(frame)
            detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 80,
                                         minLineLength=60, maxLineGap=10)
            fiber_diameter = self.get_fiber_diameter(detected_lines)
            if fiber_diameter is not None:
                accumulated_diameter += fiber_diameter
                valid_samples += 1

        if valid_samples > 0:
            average_diameter = accumulated_diameter / valid_samples
        
        print(f"Average width of wire: {average_diameter} mm")

        self.diameter_coefficient = 0.45/average_diameter
        print(f"Diameter_coeff: {self.diameter_coefficient} mm")

        Database.update_calibration_data("diameter_coefficient", 
                                         str(self.diameter_coefficient))

    def closeEvent(self, event):
        """Close the camera when the window is closed"""
        self.cap.release()
        event.accept()
