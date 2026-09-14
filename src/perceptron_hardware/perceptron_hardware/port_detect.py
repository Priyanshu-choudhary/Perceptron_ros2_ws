"""Work out which serial port is the lidar and which is the STM32 ECU.

Both devices ship on a Silicon Labs CP2102, both enumerate as 10c4:ea60, and
neither adapter was programmed with a unique iSerialNumber -- they BOTH report
"0001". So udev cannot tell them apart by attribute, and /dev/ttyUSB numbering
follows enumeration order, which changes with plug order and, under WSL, with
the order the devices were attached with usbipd.

Pointing the STM32 bridge at the lidar is a silent failure: the port opens,
bytes stream in, and no frame ever validates. So instead of trusting the
device name, ask each port what it is by listening to it.

The two protocols are unmistakable once you check the integrity field rather
than just looking for header bytes -- random noise at the wrong baud rate hits
any given byte value constantly, so header-only matching gives false positives:

    lidar   230400 baud, 0x54 0x2C frames, 47 bytes, CRC8 poly 0x4D
    STM32   115200 baud, 0x55 (39 B) / 0x56 (31 B) frames, XOR sum, 0x0A end

The permanent fix is to give one adapter a distinct serial with cp210x-cfg and
go back to a udev rule; until then this costs about a second at startup.
"""

import glob
import time

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

LIDAR_BAUD = 230400
STM32_BAUD = 115200


def _ld19_table():
    t = bytearray(256)
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t[i] = c
    return bytes(t)


_TABLE = _ld19_table()


def _crc8(data):
    crc = 0
    for b in bytearray(data):
        crc = _TABLE[(crc ^ b) & 0xFF]
    return crc


def _xorsum(data):
    c = 0
    for b in bytearray(data):
        c ^= b
    return c


def _sniff(port, baud, seconds=1.0, want=1024):
    """Grab up to `want` bytes from `port` at `baud`, or b'' on any failure."""
    try:
        s = serial.Serial(port, baud, timeout=0.3)
    except Exception:
        return b''
    try:
        s.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds and len(buf) < want:
            buf.extend(s.read(256))
        return bytes(buf)
    except Exception:
        return b''
    finally:
        try:
            s.close()
        except Exception:
            pass


def _count_lidar_frames(buf):
    n = 0
    i = 0
    while i + 47 <= len(buf):
        if buf[i] == 0x54 and buf[i + 1] == 0x2C and _crc8(buf[i:i + 46]) == buf[i + 46]:
            n += 1
            i += 47
        else:
            i += 1
    return n


def _count_stm32_frames(buf):
    n = 0
    i = 0
    while i < len(buf):
        h = buf[i]
        size = 39 if h == 0x55 else (31 if h == 0x56 else 0)
        if size and i + size <= len(buf):
            f = buf[i:i + size]
            if f[-1] == 0x0A and f[-2] == _xorsum(f[:-2]):
                n += 1
                i += size
                continue
        i += 1
    return n


def identify(port, seconds=1.0):
    """Return 'lidar', 'stm32' or None for one port."""
    if serial is None:
        return None

    buf = _sniff(port, LIDAR_BAUD, seconds)
    if _count_lidar_frames(buf) >= 3:
        return 'lidar'

    buf = _sniff(port, STM32_BAUD, seconds)
    if _count_stm32_frames(buf) >= 3:
        return 'stm32'

    return None


def detect_ports(candidates=None, seconds=1.0):
    """Map roles to device paths: {'lidar': '/dev/ttyUSB0', 'stm32': ...}.

    Roles that could not be identified are simply absent, so callers can fall
    back to a configured default and say something useful about what is
    missing rather than failing on a KeyError.
    """
    if candidates is None:
        candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))

    found = {}
    for dev in candidates:
        role = identify(dev, seconds)
        if role and role not in found:
            found[role] = dev
    return found


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Identify robot serial ports.')
    ap.add_argument('--seconds', type=float, default=1.0)
    ap.add_argument('--quiet', action='store_true', help='print only role=path')
    args = ap.parse_args()

    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if not args.quiet:
        print('candidates: %s' % (', '.join(ports) if ports else 'none'))

    found = detect_ports(ports, args.seconds)
    for role in ('lidar', 'stm32'):
        if role in found:
            print('%s=%s' % (role, found[role]))
        elif not args.quiet:
            print('%s=NOT FOUND' % role)
    return 0 if len(found) else 1


if __name__ == '__main__':
    raise SystemExit(main())
