"""
Serial communication utilities for KaudaV2 Control
"""
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal


class SerialReader(QThread):
    """Thread for reading serial data from Arduino"""
    new_line = pyqtSignal(str)
    
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip('\r\n')
                    if line:
                        self.new_line.emit(line)
                else:
                    self.msleep(20)
            except Exception as e:
                self.new_line.emit(f"[SerialReadError] {e}")
                self.running = False

    def stop(self):
        self.running = False
        self.wait(200)


def scan_ports():
    """Scan and return list of available COM ports"""
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


def format_angles_command(angles):
    """
    Format angles for Arduino command
    Args:
        angles: list of 5 angles in degrees
    Returns:
        str: formatted command string
    """
    return "A," + ",".join([f"{a:.2f}" for a in angles]) + "\n"


def format_goto_command(angles):
    """
    Format GOTO command for Arduino
    Args:
        angles: list of 5 angles in degrees
    Returns:
        str: formatted GOTO command string
    """
    return f"GOTO;A1={angles[0]};A2={angles[1]};A3={angles[2]};A4={angles[3]};A5={angles[4]}\n"
