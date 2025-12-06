"""
Dark theme and stylesheet for KaudaV2 Control
"""
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt


def get_dark_palette():
    """
    Create and return a dark color palette
    Returns:
        QPalette: Dark theme palette
    """
    dark_palette = QPalette()
    
    # Colors
    color_bg = QColor(45, 45, 45)
    color_fg = QColor(220, 220, 220)
    color_base = QColor(30, 30, 30)
    color_alt_base = QColor(45, 45, 45)
    color_btn = QColor(60, 60, 60)
    color_highlight = QColor(42, 130, 218)  # Blue highlight
    color_highlight_text = QColor(255, 255, 255)

    dark_palette.setColor(QPalette.Window, color_bg)
    dark_palette.setColor(QPalette.WindowText, color_fg)
    dark_palette.setColor(QPalette.Base, color_base)
    dark_palette.setColor(QPalette.AlternateBase, color_alt_base)
    dark_palette.setColor(QPalette.ToolTipBase, color_highlight)
    dark_palette.setColor(QPalette.ToolTipText, color_highlight_text)
    dark_palette.setColor(QPalette.Text, color_fg)
    dark_palette.setColor(QPalette.Button, color_btn)
    dark_palette.setColor(QPalette.ButtonText, color_fg)
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, color_highlight)
    dark_palette.setColor(QPalette.Highlight, color_highlight)
    dark_palette.setColor(QPalette.HighlightedText, color_highlight_text)

    return dark_palette


def get_stylesheet():
    """
    Get the application stylesheet
    Returns:
        str: CSS stylesheet string
    """
    return """
        QWidget {
            color: #ffffff;
        }
        QToolTip { 
            color: #ffffff; 
            background-color: #2a82da; 
            border: 1px solid white; 
        }
        QGroupBox {
            border: 1px solid #555;
            border-radius: 5px;
            margin-top: 10px;
            font-weight: bold;
            color: #ffffff;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 3px;
            left: 10px;
        }
        QTabWidget::pane {
            border: 1px solid #555;
            background: #2d2d2d;
        }
        QTabBar::tab {
            background: #3c3f41;
            color: #ffffff;
            padding: 6px 8px;
            border: 1px solid #555;
            border-bottom-color: #555;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            min-width: 60px;
        }
        QTabBar::tab:selected, QTabBar::tab:hover {
            background: #505355;
            color: #ffffff;
        }
        QTabBar::tab:selected {
            border-color: #555;
            border-bottom-color: #2d2d2d;
        }
        QPushButton {
            background-color: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 5px;
            color: #ffffff;
        }
        QPushButton:hover {
            background-color: #505355;
        }
        QPushButton:pressed {
            background-color: #2a82da;
            color: #ffffff;
        }
        QSlider::groove:horizontal {
            border: none;
            height: 6px;
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #3a3a3a, stop:1 #2d2d2d);
            margin: 2px 0;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #4a9eff, stop:1 #2a82da);
            border: 2px solid #1a5fa0;
            width: 20px;
            height: 20px;
            margin: -8px 0;
            border-radius: 10px;
        }
        QSlider::handle:horizontal:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #5aafff, stop:1 #3a92ea);
            border: 2px solid #2a6fb0;
        }
        QSlider::handle:horizontal:pressed {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #3a8eef, stop:1 #1a72ca);
        }
        QSlider::sub-page:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #4a9eff, stop:1 #2a82da);
            border-radius: 3px;
        }
        QComboBox {
            background: #3c3f41;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 4px;
            color: #ffffff;
        }
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 20px;
            border-left-width: 1px;
            border-left-color: #555;
            border-left-style: solid;
        }
        QTextEdit {
            background-color: #1e1e1e;
            color: #ffffff;
            border: 1px solid #555;
        }
        QLabel {
            color: #ffffff;
        }
    """
