"""
UI components and widgets for KaudaV2 Control
"""
from PyQt5.QtWidgets import QCheckBox, QSlider, QLabel
from PyQt5.QtCore import Qt


class ToggleSwitch(QCheckBox):
    """Custom toggle switch widget"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setChecked(False)
        self.setStyleSheet("""
            QCheckBox::indicator {
                width: 40px;
                height: 20px;
                border-radius: 10px;
                background-color: #888;
            }
            QCheckBox::indicator:checked {
                background-color: #4CAF50;
            }
            QCheckBox::indicator:unchecked {
                background-color: #888;
            }
        """)


def create_slider(label_text, minv, maxv, default):
    """
    Create a labeled slider
    Args:
        label_text: Label text
        minv: Minimum value
        maxv: Maximum value
        default: Default value
    Returns:
        tuple: (slider, label)
    """
    label = QLabel(label_text)
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(int(minv))
    slider.setMaximum(int(maxv))
    slider.setValue(int(default))
    return slider, label


def create_joint_slider(joint_name, joint_num, minv, maxv, default=0):
    """
    Create a joint slider with label
    Args:
        joint_name: Name of the joint (e.g., "Base", "Shoulder", "Elbow")
        joint_num: Joint number (1-5)
        minv: Minimum angle value (degrees)
        maxv: Maximum angle value (degrees)
        default: Default angle value (degrees)
    Returns:
        tuple: (slider, label)
    """
    label = QLabel(f"J{joint_num} {joint_name}")
    slider = QSlider(Qt.Horizontal)
    slider.setRange(int(minv), int(maxv))
    slider.setValue(int(default))
    return slider, label
