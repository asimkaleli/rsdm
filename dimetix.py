# dimetix.py

import time
import serial
from serial.tools import list_ports

DIMETIX_HINTS = ("Dimetix", "D-Series USB Serial Port")

def iter_candidate_ports():
    ports = list(list_ports.comports())
    strong = [p for p in ports if (p.manufacturer and any(h in p.manufacturer for h in DIMETIX_HINTS))
                                or (p.product and any(h in p.product for h in DIMETIX_HINTS))]
    acm = [p for p in ports if p.device and p.device.startswith("/dev/ttyACM")]
    usb = [p for p in ports if p.device and p.device.startswith("/dev/ttyUSB")]
    seen = set()
    for group in (strong, acm, usb):
        for p in group:
            dev = p.device
            if dev and dev not in seen:
                seen.add(dev)
                yield dev

def _clean_resp(resp: str, cmd: str) -> str:
    s = (resp or "").replace("\n", "").replace("\r", "").strip()
    if s.startswith(cmd):
        s = s[len(cmd):].strip()
    return s

def open_port(port: str, timeout: float = 1.5) -> serial.Serial:
    ser = serial.Serial()
    ser.port      = port
    ser.baudrate  = 19200
    ser.bytesize  = serial.SEVENBITS
    ser.parity    = serial.PARITY_EVEN
    ser.stopbits  = serial.STOPBITS_ONE
    ser.timeout   = timeout
    ser.write_timeout = timeout
    ser.open()
    try:
        ser.setDTR(True)
        ser.setRTS(False)
    except Exception:
        pass
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def close_port(ser: serial.Serial):
    try:
        ser.close()
    except Exception:
        pass

def send_cmd_open(ser, cmd: str = "s0g", timeout: float = 1.5):
    """
    AÇIK seri porta komutu 'cmd + \\r\\n' olarak gönderir.
    Yanıtı tek sefer okumaya çalışır, portu kapatmaz.

    DÖNÜŞ: (ok: bool, resp: str)
    """
    old_to, old_wto = getattr(ser, "timeout", None), getattr(ser, "write_timeout", None)

    try:
        # Geçici timeout (kopuk portta burada bile hata olabilir)
        try:
            ser.timeout = timeout
            ser.write_timeout = timeout
        except Exception:
            pass  # EIO anında burası bile patlayabilir

        # HER ZAMAN '\r\n' gönder
        ser.write((cmd + "\r\n").encode("ascii"))
        ser.flush()

        # '\r' gelene kadar oku (tek deneme)
        raw = ser.read_until(b"\r")
        resp = raw.decode(errors="replace").replace("\n", "").replace("\r", "").strip()
        return True, resp

    except Exception as e:
        return False, f"ERR: {e}"

    finally:
        # Port bozuksa reconfigure patlamasın diye güvenli geri yükleme
        try:
            if old_to is not None:
                ser.timeout = old_to
            if old_wto is not None:
                ser.write_timeout = old_wto
        except Exception:
            # Burada EIO vs. olursa yutuyoruz; üst katmanda reconnect denenir.
            pass


def find_and_open_port(cmd: str = "s0g", timeout: float = 1.5):
    """
    Uygun portu bulur ve AÇAR.
    DÖNÜŞ: (ser: serial.Serial, port_path: str) veya (None, None)
    """
    for port in iter_candidate_ports():
        try:
            ser = open_port(port, timeout=timeout)
        except Exception:
            continue
        ok, resp = send_cmd_open(ser, cmd=cmd, timeout=timeout)
        if ok and resp:
            return ser, port
        close_port(ser)
    return None, None
