"""CH340 Modbus RTU reader; hardware is optional on Windows."""
import os
import queue
import re
import threading
import time

_WINDOWS = os.name == "nt"
if not _WINDOWS:
    import select
    import termios
    import tty
else:
    try:
        import serial as _serial
        from serial.tools import list_ports as _list_ports
        if not callable(getattr(_serial, "Serial", None)):
            _serial = _list_ports = None
    except (ImportError, OSError):
        _serial = _list_ports = None


def _windows_port(value):
    """Accept only an explicitly selected Windows COM port."""
    value = value.strip().upper()
    if value.startswith("\\\\.\\"):
        value = value[4:]
    return value if re.fullmatch(r"COM[1-9][0-9]*", value) else ""


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xa001 if value & 1 else value >> 1
    return value.to_bytes(2, "little")


class RiseWindow:
    """Compare against the first sample of each fixed observation window."""
    def __init__(self, seconds=20.0, rise=20.0, max_gap=3.0):
        self.max_gap = max_gap
        self.seconds = seconds
        self.rise = rise
        self.clear()

    def clear(self):
        self.baseline = None
        self.started_at = None
        self.last_at = None

    def add(self, timestamp, value):
        if self.last_at is not None and (timestamp < self.last_at or (self.max_gap is not None and timestamp - self.last_at > self.max_gap)):
            self.clear()
        if self.started_at is None or timestamp - self.started_at > self.seconds:
            self.baseline = value
            self.started_at = timestamp
        self.last_at = timestamp
        return value - self.baseline >= self.rise - 1e-9


def decode_reading(data):
    if len(data) != 9 or data[:3] != bytes([1, 3, 4]) or crc16(data[:-2]) != data[-2:]:
        raise OSError("Modbus timeout or invalid response/CRC")
    temperature = int.from_bytes(data[3:5], "big", signed=True) / 10.0
    humidity = int.from_bytes(data[5:7], "big") / 10.0
    if not -50 <= temperature <= 250:
        raise OSError("temperature out of range")
    if not 0 <= humidity <= 100:
        raise OSError("humidity out of range")
    return temperature, humidity


class ModbusTemperatureReader:
    def __init__(self):
        default = "" if _WINDOWS else "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        self.port = os.environ.get("ISAAC_TEMP_PORT", default).strip()
        self.auto_port = "ISAAC_TEMP_PORT" not in os.environ
        self.status = "idle"
        self.events = queue.Queue(maxsize=32)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="lab-modbus-temperature", daemon=True)

    def start(self):
        if self.thread.ident is not None:
            return
        if _WINDOWS:
            if self.port and not _windows_port(self.port):
                self.status = "ISAAC_TEMP_PORT must be a Windows COM port, for example COM3"
                self._emit(None, self.status)
                return
            if _serial is None:
                self.status = "Windows USB sensor disabled: optional pyserial is unavailable"
                if self.port:
                    self._emit(None, self.status)
                return
            if not self.auto_port and not self.port:
                return
            self.port = _windows_port(self.port)
            self.thread.start()
            return
        if self.port:
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2.0)

    def _emit(self, value, error=None):
        event = (time.monotonic(), value, error)
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.events.get_nowait()
            self.events.put_nowait(event)

    def _run(self):
        if _WINDOWS:
            self._run_windows()
        else:
            self._run_linux()

    def _find_windows_port(self):
        # Enumerate USB metadata; never open/probe unrelated serial devices.
        ports = sorted({
            _windows_port(item.device) for item in _list_ports.comports()
            if getattr(item, "vid", None) == 0x1A86 and _windows_port(item.device)
        })
        if len(ports) > 1:
            raise OSError("Multiple CH340 USB ports found; set ISAAC_TEMP_PORT=COMx")
        return ports[0] if ports else ""

    def _run_windows(self):
        body = bytes([1, 3, 0, 0, 0, 2])
        request = body + crc16(body)
        while not self.stop_event.is_set():
            connection = None
            try:
                port = self._find_windows_port() if self.auto_port else self.port
                if not port:
                    self.status = "No CH340 temperature sensor detected"
                    self.stop_event.wait(2.0)
                    continue
                self.port = port
                connection = _serial.Serial(
                    port=port, baudrate=9600, bytesize=_serial.EIGHTBITS,
                    parity=_serial.PARITY_NONE, stopbits=_serial.STOPBITS_ONE,
                    timeout=0.1, write_timeout=0.8,
                )
                self.status = "Connected to " + port
                while not self.stop_event.is_set():
                    started = time.monotonic()
                    connection.reset_input_buffer()
                    if connection.write(request) != len(request):
                        raise OSError("Incomplete Modbus request")
                    data = bytearray()
                    deadline = started + 0.8
                    while len(data) < 9 and time.monotonic() < deadline and not self.stop_event.is_set():
                        data.extend(connection.read(9 - len(data)))
                    if self.stop_event.is_set():
                        break
                    self._emit(decode_reading(data))
                    self.stop_event.wait(max(0, 1.0 - (time.monotonic() - started)))
            except (OSError, _serial.SerialException) as error:
                self.status = str(error)
                self._emit(None, self.status)
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except OSError:
                        pass
            self.stop_event.wait(2.0)

    def _run_linux(self):
        while not self.stop_event.is_set():
            # Wait quietly for the known sensor; do not probe unrelated ports.
            if self.auto_port and not os.path.exists(self.port):
                self.stop_event.wait(2.0)
                continue
            fd = None
            old = None
            try:
                fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                old = termios.tcgetattr(fd)
                tty.setraw(fd)
                attrs = termios.tcgetattr(fd)
                attrs[4] = attrs[5] = termios.B9600
                attrs[2] |= termios.CLOCAL | termios.CREAD
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                body = bytes([1, 3, 0, 0, 0, 2])
                while not self.stop_event.is_set():
                    started = time.monotonic()
                    termios.tcflush(fd, termios.TCIFLUSH)
                    os.write(fd, body + crc16(body))
                    data = bytearray()
                    deadline = started + 0.8
                    while len(data) < 9 and time.monotonic() < deadline and not self.stop_event.is_set():
                        if select.select([fd], [], [], min(0.1, max(0, deadline - time.monotonic())))[0]:
                            chunk = os.read(fd, 256)
                            if not chunk:
                                raise OSError("sensor disconnected")
                            data.extend(chunk)
                    self._emit(decode_reading(data))
                    self.stop_event.wait(max(0, 1.0 - (time.monotonic() - started)))
            except (OSError, termios.error) as error:
                self._emit(None, str(error))
            finally:
                if fd is not None:
                    try:
                        if old is not None:
                            termios.tcsetattr(fd, termios.TCSANOW, old)
                    except (OSError, termios.error):
                        pass
                    os.close(fd)
            self.stop_event.wait(2.0)
