import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

from ml_robotics_project.UI.ICameraView import ICameraView


class CameraView(QWidget, ICameraView):
    """A simple camera view widget that displays a camera feed."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Feed")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for camera feed...")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

    def update_frame(self, rgb_image: np.ndarray):
        # Convert image to Qt format
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(
            rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
        )
        image_pixmap = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)

        # Display image
        self.label.setPixmap(QPixmap.fromImage(image_pixmap))


def main(args=None):
    app = QApplication(sys.argv)
    window = CameraView()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraView()
    window.show()
    sys.exit(app.exec_())
