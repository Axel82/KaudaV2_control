"""
UI components and widgets for KaudaV2 Control
"""
from PyQt5.QtWidgets import QCheckBox, QSlider, QLabel, QSpinBox
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
    Create a labeled slider with dynamic value input
    Args:
        label_text: Label text
        minv: Minimum value
        maxv: Maximum value
        default: Default value
    Returns:
        tuple: (slider, label, spinbox)
    """
    label = QLabel(label_text)
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(int(minv))
    slider.setMaximum(int(maxv))
    slider.setValue(int(default))
    slider.setMaximumWidth(150)
    
    # Create spinbox for value input
    spinbox = QSpinBox()
    spinbox.setMinimum(int(minv))
    spinbox.setMaximum(int(maxv))
    spinbox.setValue(int(default))
    spinbox.setMinimumWidth(60)
    spinbox.setAlignment(Qt.AlignCenter)
    spinbox.setStyleSheet("""
        QSpinBox {
            background-color: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 4px;
            font-weight: bold;
            color: #4a9eff;
        }
        QSpinBox::up-button, QSpinBox::down-button {
            width: 0px; 
            height: 0px;
        }
    """)
    
    # Bidirectional connection
    # Block signals to prevent infinite loops during updates
    def update_spinbox(val):
        spinbox.blockSignals(True)
        spinbox.setValue(val)
        spinbox.blockSignals(False)
        
    def update_slider(val):
        slider.blockSignals(True)
        slider.setValue(val)
        slider.blockSignals(False)
        # Emit slider valueChanged signal manually if needed for parent listeners, 
        # but usually setting value triggers it effectively unless blocked. 
        # Here we blocked it, so we need to be careful if outside code relies on 'valueChanged' from slider 
        # when spinbox changes.
        # Actually, standard behavior: 
        # Slider changes -> update Spinbox (visual only, logic usually listens to slider)
        # Spinbox changes -> update Slider -> Slider emits valueChanged -> logic listens to slider
        
        # So we SHOULD NOT block slider signals if we want the main app to react!
        # But if we don't block, we get a loop: Spinbox -> Slider -> Spinbox...
        # QSpinBox/QSlider check if value changed before emitting, breaking the loop naturally?
        # Let's test standard loop breaking:
        # If val == current, no signal.
        
    # Simplified connections relying on value-change check preventing loops
    slider.valueChanged.connect(spinbox.setValue)
    spinbox.valueChanged.connect(slider.setValue)
    
    return slider, label, spinbox


def create_joint_slider(joint_name, joint_num, minv, maxv, default=0):
    """
    Create a joint slider with label and dynamic value input
    Args:
        joint_name: Name of the joint (e.g., "Base", "Shoulder", "Elbow")
        joint_num: Joint number (1-5)
        minv: Minimum angle value (degrees)
        maxv: Maximum angle value (degrees)
        default: Default angle value (degrees)
    Returns:
        tuple: (slider, label, spinbox)
    """
    label = QLabel(f"J{joint_num} {joint_name}")
    slider = QSlider(Qt.Horizontal)
    slider.setRange(int(minv), int(maxv))
    slider.setValue(int(default))
    slider.setMaximumWidth(150)
    
    # Create spinbox
    spinbox = QSpinBox()
    spinbox.setRange(int(minv), int(maxv))
    spinbox.setValue(int(default))
    spinbox.setSuffix("°")
    spinbox.setMinimumWidth(70)
    spinbox.setAlignment(Qt.AlignCenter)
    spinbox.setStyleSheet("""
        QSpinBox {
            background-color: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 4px;
            font-weight: bold;
            color: #4a9eff;
        }
        QSpinBox::up-button, QSpinBox::down-button {
            width: 0px; 
            height: 0px;
        }
    """)
    
    # Bidirectional connection
    slider.valueChanged.connect(spinbox.setValue)
    spinbox.valueChanged.connect(slider.setValue)
    
    return slider, label, spinbox
