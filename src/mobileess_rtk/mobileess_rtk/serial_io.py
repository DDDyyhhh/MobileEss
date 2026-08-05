"""Thin pyserial adapter used by the ROS node."""
import serial


class SerialPort:
    def __init__(self, device: str, baud: int, timeout: float = 0.2):
        self._serial = serial.Serial(device, baudrate=baud, timeout=timeout)

    def readline(self) -> bytes:
        return self._serial.readline()

    def write(self, data: bytes) -> int:
        return self._serial.write(data)

    def close(self) -> None:
        self._serial.close()
