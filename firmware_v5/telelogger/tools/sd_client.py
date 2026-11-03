#!/usr/bin/env python3
"""Read the Freematics ONE+ SD card over USB serial (telelogger SD console).

usage: [FREEMATICS_PORT=/dev/cu.usbserial-XXXX] sd_client.py list [DIR]            (default /DATA; e.g. /LOGS)
       sd_client.py get PATH [PATH ...] --out DIR
       sd_client.py get-all DIR --out OUTDIR

Talks to the running telelogger without resetting it.
"""
import os, sys, termios, time, select

PORT = os.environ.get("FREEMATICS_PORT", "/dev/cu.usbserial-01E748DD")


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def read_until(fd, buf, pred, timeout):
    """read into buf until pred(buf) is true; timeout is idle time."""
    end = time.time() + timeout
    while not pred(buf):
        if time.time() > end:
            raise TimeoutError(f"timed out; last bytes {bytes(buf[-200:])!r}")
        r, _, _ = select.select([fd], [], [], 0.5)
        if r:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                continue
            buf += chunk
            end = time.time() + timeout
    return buf


def send(fd, cmd):
    # drop pending log output, then send the command
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, (cmd + "\n").encode())


def check_err(buf, start_marker):
    i = buf.find(b"\nERR ") if b"ERR " in buf else -1
    if i >= 0 and start_marker not in buf[:i + 1]:
        line = buf[i + 1:].split(b"\n", 1)[0].decode(errors="replace")
        raise RuntimeError(line)


def sd_list(fd, d):
    send(fd, f"SD LIST {d}")
    buf = bytearray(b"\n")
    def done(b):
        check_err(b, b"BEGIN LIST")
        return b"END LIST\n" in b
    buf = read_until(fd, buf, done, 10)
    body = buf.split(b"BEGIN LIST\n", 1)[1].split(b"END LIST\n", 1)[0]
    return [ln.split(" ", 2) for ln in body.decode(errors="replace").splitlines() if ln]


def sd_get(fd, path):
    send(fd, f"SD GET {path}")
    buf = bytearray(b"\n")
    marker = f"BEGIN FILE {path} ".encode()
    def have_header(b):
        check_err(b, marker)
        i = b.find(marker)
        return i >= 0 and b.find(b"\n", i) >= 0
    buf = read_until(fd, buf, have_header, 10)
    i = buf.find(marker)
    nl = buf.find(b"\n", i)
    size = int(buf[i + len(marker):nl])
    start = nl + 1
    buf = read_until(fd, buf, lambda b: len(b) >= start + size + len(b"\nEND FILE\n"), 15)
    data = bytes(buf[start:start + size])
    if buf[start + size:start + size + 10] != b"\nEND FILE\n":
        raise RuntimeError(f"{path}: transfer corrupted")
    return data


def main():
    fd = open_port()
    if sys.argv[1] == "list":
        for name, size, ts in sd_list(fd, sys.argv[2] if len(sys.argv) > 2 else "/DATA"):
            print(f"{name:>12} {int(size):>8}  {ts}")
        return
    outdir = sys.argv[sys.argv.index("--out") + 1]
    os.makedirs(outdir, exist_ok=True)
    if sys.argv[1] == "get-all":
        d = sys.argv[2].rstrip("/")
        paths = [f"{d}/{name}" for name, _, _ in sd_list(fd, d)]
    else:
        paths = sys.argv[2:sys.argv.index("--out")]
    for p in paths:
        data = sd_get(fd, p)
        with open(os.path.join(outdir, os.path.basename(p)), "wb") as f:
            f.write(data)
        print(f"{p} -> {len(data)} bytes")


main()
