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
    Create a labeled slider with dynamic value display
    Args:
        label_text: Label text
        minv: Minimum value
        maxv: Maximum value
        default: Default value
    Returns:
        tuple: (slider, label, value_label)
    """
    label = QLabel(label_text)
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(int(minv))
    slider.setMaximum(int(maxv))
    slider.setValue(int(default))
    slider.setMaximumWidth(150)  # Set minimum width for better UX
    
    # Create value label with modern styling
    value_label = QLabel(str(int(default)))
    value_label.setMinimumWidth(50)
    value_label.setAlignment(Qt.AlignCenter)
    value_label.setStyleSheet("""
        QLabel {
            background-color: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 4px 8px;
            font-weight: bold;
            color: #4a9eff;
        }
    """)
    
    # Connect slider to update value label
    slider.valueChanged.connect(lambda v: value_label.setText(str(v)))
    
    return slider, label, value_label


def create_joint_slider(joint_name, joint_num, minv, maxv, default=0):
    """
    Create a joint slider with label and dynamic value display
    Args:
        joint_name: Name of the joint (e.g., "Base", "Shoulder", "Elbow")
        joint_num: Joint number (1-5)
        minv: Minimum angle value (degrees)
        maxv: Maximum angle value (degrees)
        default: Default angle value (degrees)
    Returns:
        tuple: (slider, label, value_label)
    """
    label = QLabel(f"J{joint_num} {joint_name}")
    slider = QSlider(Qt.Horizontal)
    slider.setRange(int(minv), int(maxv))
    slider.setValue(int(default))
    slider.setMaximumWidth(150)  # Set minimum width for better UX
    
    # Create value label with modern styling and degree symbol
    value_label = QLabel(f"{int(default)}°")
    value_label.setMinimumWidth(60)
    value_label.setAlignment(Qt.AlignCenter)
    value_label.setStyleSheet("""
        QLabel {
            background-color: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 4px 8px;
            font-weight: bold;
            color: #4a9eff;
        }
    """)
    
    # Connect slider to update value label with degree symbol
    slider.valueChanged.connect(lambda v: value_label.setText(f"{v}°"))
    
    return slider, label, value_label
