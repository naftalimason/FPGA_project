#!/usr/bin/env python3
"""
FPGA board bring-up test for the uartBurst_ahbSingle design (Nexys A7-100T).

Test sequence (as specified):
  1. Register checker write/read validation (img_rgf, uart_rgf, fifo_rgf).
  2. Read the UART error R0C register (clear-on-read) + verify it cleared.
  3. Load one 256x256 PNG, split it into red_hex.mem / green_hex.mem /
     blue_hex.mem, then perform the image burst write.
  4. Poll img_rgf.IMG_STATUS.ready_to_read; when valid, image burst read back
     and compare against the generated .mem contents.
  5. Read the UART error R0C register again.

Link parameters (from UART_package.sv / chiptop.sv):
  8,000,000 baud, 8 data bits, ODD parity (PARITY_ODD = 1), 1 stop bit,
  hardware RTS/CTS flow control (UART_RTS / UART_CTS pads, active low =
  standard RS-232 semantics, handled by the driver via rtscts=True).

Message formats (message_format_spec.txt; bytes listed in wire order,
'{' first):
  REG WRITE  (16B, PC->FPGA): { W A2 A1 A0 , V 0 DH1 DH0 , V 0 DL1 DL0 }
  REG READ   ( 6B, PC->FPGA): { R A2 A1 A0 }
  REG REPLY  (16B, FPGA->PC): { R A2 A1 A0 , V 0 DH1 DH0 , V 0 DL1 DL0 }
  IMG WR HDR (16B, PC->FPGA): { I 0 0 0 , H H2 H1 H0 , W W2 W1 W0 }
  PAYLOAD    (16B, both dir): { R0 G0 B0 R1 , G1 B1 R2 G2 , B2 R3 G3 B3 }
  IMG RD REQ (16B, PC->FPGA): { R 0 0 0 , H H2 H1 H0 , W W2 W1 W0 }
  RESEND {E} ( 6B, FPGA->PC): { E 0 0 0 }   -> PC must resend its last message

Address map (address_map_v1 + RTL localparams - RTL is authoritative):
  A2=0x04 img_rgf : 0x0000 IMG_STATUS  (ready_to_read=[18], height=[17:9],
                                        width=[8:0])
                    0x0004 IMG_RX_MON  (complete=[16], row=[15:8], col=[7:0])
                    0x0008 IMG_TX_MON
                    0x000C IMG_CTRL    (RO mirror)
                    0x0010 IMG_CHECKER (RW, 24-bit)
  A2=0x08 uart_rgf: 0x0000 UART_REG    (R0C: clsf[23:16], mac[15:8], phy[7:0];
                                        cleared by a completed read)
                    0x0004 UART_CONFIG (WO, parity_odd=[0])
                    0x0008 UART_CHECKER(RW, 24-bit)
                    0x000C CLK_STATUS  (RO, clk_sel=[0])
  A2=0x0C fifo_rgf: 0x0000..0x0014 FIFO status regs
                    0x0018 FIFO_CHECKER(RW, 24-bit)

Channel word packing (classifier.sv / composer.sv): each .mem line is one
32-bit word {P0,P1,P2,P3} with the QUAD'S FIRST pixel in the MSB (byte 3).

Place one 256x256 PNG in the same directory as this script. The script
finds it automatically and writes red_hex.mem, green_hex.mem, blue_hex.mem,
readback_raw.bin, and readback_image.png into that same directory.

Usage:
  python fpga_uart_test.py                 # auto-detect COM port
  python fpga_uart_test.py --port COM11    # explicit port
  python fpga_uart_test.py --skip-image    # registers only
  python fpga_uart_test.py --dry-run       # PNG conversion + self-check only

Every complete PC->FPGA message is printed in full hexadecimal wire order.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

# ============================================================
# Link configuration (must match UART_package.sv)
# ============================================================
BAUD_RATE = 8_000_000
LINE_BITS_PER_BYTE = 11          # 1 start + 8 data + 1 parity + 1 stop

# ASCII literals
LB, RB, COMMA = 0x7B, 0x7D, 0x2C            # '{'  '}'  ','
OP_W, OP_V, OP_R, OP_I, OP_H, OP_E = 0x57, 0x56, 0x52, 0x49, 0x48, 0x45

# BAR addresses (A2)
BAR_IMG, BAR_UART, BAR_FIFO = 0x04, 0x08, 0x0C

# Register offsets ({A1,A0}, byte offsets, stride 4 - from RTL localparams)
IMG_STATUS, IMG_RX_MON, IMG_TX_MON, IMG_CTRL, IMG_CHECKER = (
    0x0000, 0x0004, 0x0008, 0x000C, 0x0010)
UART_REG, UART_CONFIG, UART_CHECKER, CLK_STATUS = 0x0000, 0x0004, 0x0008, 0x000C
FIFO_CHECKER = 0x0018
FIFO_RX_R, FIFO_TX_R = 0x0000, 0x000C

IMG_H = IMG_W = 256
QUADS = IMG_H * IMG_W // 4                  # 16,384 payload messages
PAYLOAD_BYTES = QUADS * 16                  # 262,144 wire bytes per direction

MEM_DIR = Path(__file__).resolve().parent
MEM_FILES = {"red": "red_hex.mem", "green": "green_hex.mem", "blue": "blue_hex.mem"}

MAX_RESEND_RETRIES = 8


# ============================================================
# Message builders / parsers (wire order: '{' first)
# ============================================================
def msg_reg_write(a2: int, offset: int, value: int) -> bytes:
    a1, a0 = (offset >> 8) & 0xFF, offset & 0xFF
    dh1, dh0 = (value >> 24) & 0xFF, (value >> 16) & 0xFF
    dl1, dl0 = (value >> 8) & 0xFF, value & 0xFF
    return bytes([LB, OP_W, a2, a1, a0, COMMA, OP_V, 0x00, dh1, dh0,
                  COMMA, OP_V, 0x00, dl1, dl0, RB])


def msg_reg_read(a2: int, offset: int) -> bytes:
    return bytes([LB, OP_R, a2, (offset >> 8) & 0xFF, offset & 0xFF, RB])


def msg_img_write_header(h: int = IMG_H, w: int = IMG_W) -> bytes:
    return bytes([LB, OP_I, 0, 0, 0,
                  COMMA, OP_H, (h >> 16) & 0xFF, (h >> 8) & 0xFF, h & 0xFF,
                  COMMA, OP_W, (w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF, RB])


def msg_img_read_request(h: int = IMG_H, w: int = IMG_W) -> bytes:
    return bytes([LB, OP_R, 0, 0, 0,
                  COMMA, OP_H, (h >> 16) & 0xFF, (h >> 8) & 0xFF, h & 0xFF,
                  COMMA, OP_W, (w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF, RB])


def msg_payload(rw: int, gw: int, bw: int) -> bytes:
    """One pixel-payload message from the three 32-bit channel words
    ({P0,P1,P2,P3}, first pixel of the quad in the MSB)."""
    r = rw.to_bytes(4, "big")
    g = gw.to_bytes(4, "big")
    b = bw.to_bytes(4, "big")
    return bytes([LB, r[0], g[0], b[0], r[1],
                  COMMA, g[1], b[1], r[2], g[2],
                  COMMA, b[2], r[3], g[3], b[3], RB])


def parse_reg_reply(m: bytes):
    """Return (a2, offset, value) or None if the frame doesn't match."""
    if len(m) != 16 or m[0] != LB or m[1] != OP_R or m[5] != COMMA \
            or m[6] != OP_V or m[10] != COMMA or m[11] != OP_V or m[15] != RB:
        return None
    a2, offset = m[2], (m[3] << 8) | m[4]
    value = (m[8] << 24) | (m[9] << 16) | (m[13] << 8) | m[14]
    return a2, offset, value


def parse_payload(m: bytes):
    """Return (r_word, g_word, b_word) or None."""
    if len(m) != 16 or m[0] != LB or m[5] != COMMA or m[10] != COMMA or m[15] != RB:
        return None
    rw = (m[1] << 24) | (m[4] << 16) | (m[8] << 8) | m[12]
    gw = (m[2] << 24) | (m[6] << 16) | (m[9] << 8) | m[13]
    bw = (m[3] << 24) | (m[7] << 16) | (m[11] << 8) | m[14]
    return rw, gw, bw


def is_resend_request(m: bytes) -> bool:
    return len(m) == 6 and m[0] == LB and m[1] == OP_E and m[5] == RB


def decode_r0c(value: int) -> str:
    return (f"phy={value & 0xFF} mac={(value >> 8) & 0xFF} "
            f"clsf={(value >> 16) & 0xFF} (raw=0x{value:08X})")


# ============================================================
# PNG -> channel .mem conversion
# ============================================================
def write_mem_words(path: Path, words: list[int]) -> None:
    """Write one uppercase eight-hex-digit word per line."""
    path.write_text("".join(f"{word:08X}\n" for word in words))


def png_to_mem_words(image_path: Path, out_dir: Path) -> dict[str, list[int]]:
    """Convert one 256x256 PNG into the three channel word arrays and files.

    Pixels are read left-to-right, top-to-bottom. Every four consecutive pixels
    become one 32-bit channel word {P0,P1,P2,P3}, with P0 in bits [31:24].
    """
    if image_path.suffix.lower() != ".png":
        raise ValueError(f"input image must be a .png file: {image_path}")
    if not image_path.is_file():
        raise FileNotFoundError(f"PNG file not found: {image_path}")

    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required for PNG input. Install it with: pip install pillow"
        ) from exc

    with Image.open(image_path) as source:
        image = source.convert("RGB")
        if image.size != (IMG_W, IMG_H):
            raise ValueError(
                f"{image_path.name}: expected {IMG_W}x{IMG_H} pixels, "
                f"got {image.width}x{image.height}"
            )
        pixels = list(image.getdata())

    channels = {
        "red": [pixel[0] for pixel in pixels],
        "green": [pixel[1] for pixel in pixels],
        "blue": [pixel[2] for pixel in pixels],
    }

    mem: dict[str, list[int]] = {}
    for name, values in channels.items():
        words = [
            (values[i] << 24) | (values[i + 1] << 16) |
            (values[i + 2] << 8) | values[i + 3]
            for i in range(0, len(values), 4)
        ]
        if len(words) != QUADS:
            raise RuntimeError(
                f"internal conversion error for {name}: "
                f"expected {QUADS} words, got {len(words)}"
            )
        mem[name] = words

    out_dir.mkdir(parents=True, exist_ok=True)
    for name, filename in MEM_FILES.items():
        path = out_dir / filename
        write_mem_words(path, mem[name])
        print(f"  generated {path} ({len(mem[name]):,} words)")

    return mem


def format_wire_message(msg: bytes) -> str:
    """Return every byte in exact UART wire order, without truncation."""
    return " ".join(f"{byte:02X}" for byte in msg)


# ============================================================
# Serial link with {E}-resend handling
# ============================================================
class FpgaLink:
    def __init__(self, port: str):
        self.ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_ODD,       # PARITY_ODD = 1 in UART_package.sv
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            write_timeout=5.0,
            rtscts=True,                    # UART_RTS / UART_CTS at the pads
        )
        self.last_sent: bytes | None = None
        self.resend_count = 0
        self.tx_message_count = 0

    def close(self):
        self.ser.close()

    # ---- TX with resend support -------------------------------------
    def _print_tx(self, msg: bytes, note: str = "") -> None:
        """Print one complete logical PC->FPGA message."""
        self.tx_message_count += 1
        suffix = f" [{note}]" if note else ""
        print(f"TX #{self.tx_message_count:06d} ({len(msg)} bytes){suffix}: "
              f"{format_wire_message(msg)}")

    def send(self, msg: bytes):
        """Send one message; on an {E} NAK in the input stream, resend."""
        self.last_sent = msg
        self._print_tx(msg)
        self.ser.write(msg)
        self._service_nak()

    def send_batch(self, messages: list[bytes]) -> None:
        """Print every logical message, then write the batch as one chunk."""
        for msg in messages:
            self._print_tx(msg)
        self.ser.write(b"".join(messages))

    def _service_nak(self):
        """If the FPGA pushed an {E} while we were sending, resend last msg."""
        for _ in range(MAX_RESEND_RETRIES):
            if self.ser.in_waiting < 6:
                return
            head = self.ser.read(1)
            if head != bytes([LB]):
                continue                     # resync: drop stray byte
            rest = self.ser.read(5)
            frame = head + rest
            if is_resend_request(frame):
                self.resend_count += 1
                print(f"  [E] resend request from FPGA -> resending last message "
                      f"(total resends: {self.resend_count})")
                if self.last_sent is None:
                    raise RuntimeError("FPGA requested resend before any message was sent")
                self._print_tx(self.last_sent, note="RESEND")
                self.ser.write(self.last_sent)
            else:
                # Not an {E}: push back is impossible - stash for read_message
                self._pushback = frame
                return
        raise RuntimeError("FPGA keeps NAKing the same message "
                           f"({MAX_RESEND_RETRIES} resends) - aborting")

    # ---- RX -----------------------------------------------------------
    _pushback: bytes = b""

    def _read_exact(self, n: int, timeout_s: float) -> bytes:
        buf = bytearray()
        if self._pushback:
            buf.extend(self._pushback)
            self._pushback = b""
        t0 = time.perf_counter()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            elif time.perf_counter() - t0 > timeout_s:
                break
        return bytes(buf)

    def read_message(self, timeout_s: float = 2.0) -> bytes:
        """Read one framed FPGA->PC message (6-byte {E} or 16-byte reply)."""
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            b0 = self._read_exact(1, timeout_s)
            if not b0:
                continue
            if b0[0] != LB:
                continue                     # resync on '{'
            body = self._read_exact(5, timeout_s)
            frame = b0 + body
            if len(frame) < 6:
                continue
            if is_resend_request(frame):
                return frame
            rest = self._read_exact(10, timeout_s)
            return frame + rest
        return b""

    # ---- Register access (with E-retry) -------------------------------
    def write_reg(self, a2: int, offset: int, value: int):
        self.send(msg_reg_write(a2, offset, value))
        time.sleep(0.002)                    # let the APB write land

    def read_reg(self, a2: int, offset: int, tries: int = 3) -> int:
        for attempt in range(tries):
            self.send(msg_reg_read(a2, offset))
            reply = self.read_message()
            if is_resend_request(reply):
                self.resend_count += 1
                print("  [E] NAK on read request -> retrying")
                continue
            parsed = parse_reg_reply(reply)
            if parsed is None:
                print(f"  ! bad reply frame (attempt {attempt + 1}): "
                      f"{reply.hex(' ') if reply else '<timeout>'}")
                continue
            ra2, roff, value = parsed
            if (ra2, roff) != (a2, offset):
                print(f"  ! reply address echo mismatch: got A2=0x{ra2:02X} "
                      f"off=0x{roff:04X}, expected A2=0x{a2:02X} off=0x{offset:04X}")
            return value
        raise RuntimeError(f"register read A2=0x{a2:02X} off=0x{offset:04X} failed")


# ============================================================
# Test phases
# ============================================================
def test_checkers(link: FpgaLink) -> bool:
    """Phase 1: write/read the three 24-bit checker scratch registers."""
    print("\n=== Phase 1: register checker write/read validation ===")
    checkers = [("img_rgf ", BAR_IMG, IMG_CHECKER),
                ("uart_rgf", BAR_UART, UART_CHECKER),
                ("fifo_rgf", BAR_FIFO, FIFO_CHECKER)]
    patterns = [0xA5C3F0, 0x5A3C0F, 0x000000, 0xFFFFFF]
    ok = True
    for name, a2, off in checkers:
        initial = link.read_reg(a2, off)
        print(f"  {name} checker initial value: 0x{initial:08X}")
        for pat in patterns:
            link.write_reg(a2, off, pat)
            got = link.read_reg(a2, off)
            expect = pat & 0xFFFFFF          # 24-bit register
            status = "OK" if got == expect else "FAIL"
            if got != expect:
                ok = False
            print(f"  {name} wrote 0x{pat:06X} read 0x{got:08X}  [{status}]")
    print(f"Phase 1 result: {'PASS' if ok else 'FAIL'}")
    return ok


def read_r0c(link: FpgaLink, label: str, verify_clear: bool = True) -> bool:
    """Read UART_REG (R0C error counters); a completed read clears it."""
    print(f"\n=== R0C error register read ({label}) ===")
    val = link.read_reg(BAR_UART, UART_REG)
    print(f"  UART_REG: {decode_r0c(val)}")
    ok = True
    if verify_clear:
        val2 = link.read_reg(BAR_UART, UART_REG)
        cleared = (val2 == 0)
        # Note: an error between the two reads would legitimately re-count.
        print(f"  after clear-on-read: {decode_r0c(val2)}  "
              f"[{'OK - cleared' if cleared else 'note: nonzero'}]")
        ok = cleared
    return ok


def send_image(link: FpgaLink, mem: dict[str, list[int]]) -> float:
    """Phase 2: image burst write (header + 16,384 payloads)."""
    print("\n=== Phase 2: image burst write (256x256 generated from PNG) ===")
    print("  sending header {I,H,W} ...")
    link.send(msg_img_write_header())
    time.sleep(0.005)                        # header classify + grant window

    print(f"  streaming {QUADS:,} payload messages ({PAYLOAD_BYTES:,} bytes, "
          f"RTS/CTS paced) ...")
    t0 = time.perf_counter()
    MESSAGES_PER_CHUNK = 4096 // 16
    for first_q in range(0, QUADS, MESSAGES_PER_CHUNK):
        last_q = min(first_q + MESSAGES_PER_CHUNK, QUADS)
        messages = [
            msg_payload(mem["red"][q], mem["green"][q], mem["blue"][q])
            for q in range(first_q, last_q)
        ]
        link.send_batch(messages)
        # Peek for an async {E} NAK. Mid-burst recovery is intentionally
        # unchanged: a NAK means the image transfer must restart.
        if link.ser.in_waiting >= 6:
            frame = link._read_exact(6, 0.2)
            if is_resend_request(frame):
                raise RuntimeError(
                    "FPGA sent {E} mid-burst - payload stream is corrupted; "
                    "restart the image write")
    link.ser.flush()
    elapsed = time.perf_counter() - t0
    rate = PAYLOAD_BYTES / elapsed if elapsed > 0 else 0
    print(f"  done in {elapsed:.3f} s ({rate / 1e6:.3f} MB/s payload, "
          f"~{rate * LINE_BITS_PER_BYTE / 8 / 1e6:.3f} Mbaud effective)")
    return elapsed


def wait_image_ready(link: FpgaLink, timeout_s: float = 10.0) -> bool:
    """Phase 3: poll img_rgf.IMG_STATUS.ready_to_read (bit 18)."""
    print("\n=== Phase 3: waiting for IMG_STATUS.ready_to_read ===")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        status = link.read_reg(BAR_IMG, IMG_STATUS)
        ready = (status >> 18) & 1
        height = (status >> 9) & 0x1FF
        width = status & 0x1FF
        if ready:
            print(f"  IMG_STATUS=0x{status:08X}: ready=1, "
                  f"height={height}, width={width}  [OK]")
            dims_ok = (height == IMG_H and width == IMG_W)
            if not dims_ok:
                print(f"  ! dimension mismatch (expected {IMG_H}x{IMG_W})")
            return dims_ok
        time.sleep(0.05)
    rx_mon = link.read_reg(BAR_IMG, IMG_RX_MON)
    print(f"  TIMEOUT: ready_to_read still 0 after {timeout_s} s "
          f"(IMG_STATUS=0x{status:08X}, IMG_RX_MON=0x{rx_mon:08X}: "
          f"complete={(rx_mon >> 16) & 1} row={(rx_mon >> 8) & 0xFF} "
          f"col={rx_mon & 0xFF})")
    return False


def read_image(link: FpgaLink, mem: dict[str, list[int]], out_dir: Path) -> bool:
    """Phase 4: image burst read + comparison against the .mem contents."""
    print("\n=== Phase 4: image burst read-back ===")
    link.ser.reset_input_buffer()
    link.send(msg_img_read_request())

    print(f"  capturing {PAYLOAD_BYTES:,} bytes ...")
    t0 = time.perf_counter()
    raw = link._read_exact(PAYLOAD_BYTES, timeout_s=30.0)
    elapsed = time.perf_counter() - t0
    print(f"  captured {len(raw):,}/{PAYLOAD_BYTES:,} bytes in {elapsed:.3f} s")
    (out_dir / "readback_raw.bin").write_bytes(raw)

    if len(raw) < PAYLOAD_BYTES:
        print("  FAIL: incomplete capture (raw dump saved to readback_raw.bin)")
        return False

    bad_frames = word_errors = 0
    r_words, g_words, b_words = [], [], []
    for q in range(QUADS):
        parsed = parse_payload(raw[q * 16:(q + 1) * 16])
        if parsed is None:
            bad_frames += 1
            r_words.append(0); g_words.append(0); b_words.append(0)
            continue
        rw, gw, bw = parsed
        r_words.append(rw); g_words.append(gw); b_words.append(bw)
        if rw != mem["red"][q] or gw != mem["green"][q] or bw != mem["blue"][q]:
            if word_errors < 5:
                print(f"  mismatch at quad {q}: "
                      f"R 0x{rw:08X}/0x{mem['red'][q]:08X} "
                      f"G 0x{gw:08X}/0x{mem['green'][q]:08X} "
                      f"B 0x{bw:08X}/0x{mem['blue'][q]:08X} (got/expected)")
            word_errors += 1

    print(f"  framing errors : {bad_frames}/{QUADS}")
    print(f"  word mismatches: {word_errors}/{QUADS}")

    try:
        from PIL import Image
        img = Image.new("RGB", (IMG_W, IMG_H))
        px = img.load()
        for q in range(QUADS):
            r = r_words[q].to_bytes(4, "big")
            g = g_words[q].to_bytes(4, "big")
            b = b_words[q].to_bytes(4, "big")
            for k in range(4):
                idx = q * 4 + k
                px[idx % IMG_W, idx // IMG_W] = (r[k], g[k], b[k])
        png = out_dir / "readback_image.png"
        img.save(png)
        print(f"  read-back image saved to {png}")
    except Exception as exc:                 # PNG is a nicety, not a gate
        print(f"  (PNG save skipped: {exc})")

    ok = bad_frames == 0 and word_errors == 0
    print(f"Phase 4 result: {'PASS - image read back matches the .mem files'
                             if ok else 'FAIL'}")
    return ok


# ============================================================
# Support
# ============================================================
def autodetect_port() -> str | None:
    """Prefer the FTDI/Digilent COM port of the Nexys A7."""
    candidates = []
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}"
        if any(k in text for k in ("FTDI", "Digilent", "USB Serial")):
            candidates.append(p.device)
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        print(f"Multiple candidate ports {candidates}; using {candidates[-1]} "
              f"(override with --port)")
        return candidates[-1]
    return None


def find_input_png() -> Path:
    """Find the single source PNG stored beside this script.

    The generated readback_image.png is ignored so it cannot be selected as the
    next input image. Exactly one other PNG must be present.
    """
    candidates = sorted(
        path for path in MEM_DIR.iterdir()
        if path.is_file()
        and path.suffix.lower() == ".png"
        and path.name.lower() != "readback_image.png"
    )
    if not candidates:
        raise FileNotFoundError(
            f"No input PNG found in the script directory: {MEM_DIR}"
        )
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise RuntimeError(
            "More than one input PNG was found beside the script. "
            f"Keep only the intended source image there: {names}"
        )
    return candidates[0]


def dry_run(mem: dict[str, list[int]]) -> bool:
    """Self-check without hardware: builders and parsers must round-trip."""
    print("\n=== DRY RUN: message round-trip self-check ===")
    ok = True
    m = msg_reg_write(BAR_UART, UART_CHECKER, 0x00A5C3F0)
    ok &= len(m) == 16 and m[0] == LB and m[15] == RB
    reply = bytes([LB, OP_R, BAR_UART, 0x00, 0x08, COMMA, OP_V, 0, 0x00, 0xA5,
                   COMMA, OP_V, 0, 0xC3, 0xF0, RB])
    ok &= parse_reg_reply(reply) == (BAR_UART, UART_CHECKER, 0x00A5C3F0)
    for q in (0, 1, QUADS - 1):
        p = msg_payload(mem["red"][q], mem["green"][q], mem["blue"][q])
        ok &= parse_payload(p) == (mem["red"][q], mem["green"][q], mem["blue"][q])
    ok &= is_resend_request(bytes([LB, OP_E, 0, 0, 0, RB]))
    hdr = msg_img_write_header()
    ok &= hdr[7:10] == b"\x00\x01\x00" and hdr[12:15] == b"\x00\x01\x00"
    print(f"  builders/parsers round-trip: {'PASS' if ok else 'FAIL'}")
    print(f"  mem files: {QUADS} words per channel loaded, "
          f"first R/G/B words: 0x{mem['red'][0]:08X} 0x{mem['green'][0]:08X} "
          f"0x{mem['blue'][0]:08X}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="uartBurst_ahbSingle FPGA test")
    ap.add_argument("--port", help="COM port (default: auto-detect FTDI)")
    ap.add_argument("--skip-image", action="store_true",
                    help="run only the register tests")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware: convert the local PNG and self-check messages")
    args = ap.parse_args()

    mem: dict[str, list[int]] | None = None
    if not args.skip_image:
        try:
            image_path = find_input_png()
            print(f"Converting local PNG to channel .mem files: {image_path.name}")
            mem = png_to_mem_words(image_path, MEM_DIR)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            print(f"Image preparation failed: {exc}")
            return 2

    if args.dry_run:
        if mem is None:
            print("Nothing to check: --dry-run cannot be combined with --skip-image")
            return 2
        return 0 if dry_run(mem) else 1

    port = args.port or autodetect_port()
    if not port:
        print("No COM port found - connect the board or pass --port COMxx")
        return 2
    print(f"Opening {port} @ {BAUD_RATE:,} baud, 8-O-1, RTS/CTS on")
    link = FpgaLink(port)

    results: dict[str, bool] = {}
    try:
        time.sleep(0.3)
        link.ser.reset_input_buffer()
        link.ser.reset_output_buffer()

        results["checkers"] = test_checkers(link)
        results["r0c_first"] = read_r0c(link, "after checker cycle")

        if not args.skip_image:
            assert mem is not None
            send_image(link, mem)
            results["img_ready"] = wait_image_ready(link)
            if results["img_ready"]:
                results["img_readback"] = read_image(link, mem, MEM_DIR)
            else:
                results["img_readback"] = False
            results["r0c_second"] = read_r0c(link, "after image write/read")
    finally:
        link.close()

    print("\n=== SUMMARY ===")
    for name, ok in results.items():
        print(f"  {name:14s}: {'PASS' if ok else 'FAIL'}")
    if link.resend_count:
        print(f"  {{E}} resend events serviced: {link.resend_count}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
