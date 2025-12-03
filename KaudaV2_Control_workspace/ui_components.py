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
    label = QLabel(f"{label_text} ({minv} → {maxv})")
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(int(minv))
    slider.setMaximum(int(maxv))
    slider.setValue(int(default))
    return slider, label
