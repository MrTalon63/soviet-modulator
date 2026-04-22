#!/usr/bin/env python3
import argparse
import os
import struct
import sys
import time

import serial


FRAME_SIZE_DEFAULT = 256
PAYLOAD_SIZE_DEFAULT = 252


def read_ack(ser: serial.Serial, timeout: float = 2.0) -> bytes:
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b in (b"\n", b"\r"):
            continue
        # Ignore text-mode chatter and only react to binary control bytes.
        if b not in (b"K", b"E", b"D", b"B"):
            continue
        return b
    raise TimeoutError("Timed out waiting for chunk ACK")


def wait_for_byte(ser: serial.Serial, expected: bytes, timeout: float = 2.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b in (b"\n", b"\r"):
            continue
        if b not in (b"K", b"E", b"D", b"B"):
            continue
        if b == expected:
            return
        if b == b"E":
            raise RuntimeError("Device reported upload error (E)")
        # Ignore other control bytes seen out-of-order and keep waiting.
    raise TimeoutError(f"Timed out waiting for {expected!r}")


def read_text_line(ser: serial.Serial, timeout: float = 1.0) -> str:
    deadline = time.time() + timeout
    line = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b in (b"\r", b"\n"):
            if line:
                return line.decode(errors="replace")
            continue
        line.extend(b)
    return ""


def set_rate(ser: serial.Serial, rate: int) -> None:
    ser.write(b"r")
    ser.flush()
    # Device enters input mode for rate value.
    time.sleep(0.02)
    ser.write(f"{rate}\n".encode())
    ser.flush()


def set_convolution(ser: serial.Serial, enabled: bool) -> None:
	ser.write(b"c")
	ser.flush()

	state = ""
	deadline = time.time() + 1.0
	while time.time() < deadline:
		line = read_text_line(ser, timeout=0.2)
		if not line:
			continue
		if "CCSDS convolutional coding:" in line:
			state = line
			break

	if enabled and "OFF" in state:
		ser.write(b"c")
		ser.flush()
	elif not enabled and "ON" in state:
		ser.write(b"c")
		ser.flush()


def ensure_randomizer_on(ser: serial.Serial) -> None:
    ser.write(b"n")
    ser.flush()

    state = ""
    deadline = time.time() + 1.0
    while time.time() < deadline:
        line = read_text_line(ser, timeout=0.2)
        if not line:
            continue
        if "CCSDS randomizer:" in line:
            state = line
            break

    if "OFF" in state:
        ser.write(b"n")
        ser.flush()


def send_file(
    port: str,
    baud: int,
    path: str,
    chunk_size: int,
    max_bytes: int,
    prepacked_asm: bool,
    stop_before_upload: bool,
    start_after_upload: bool,
) -> None:
    max_chunk = FRAME_SIZE_DEFAULT if prepacked_asm else PAYLOAD_SIZE_DEFAULT
    if chunk_size <= 0 or chunk_size > max_chunk:
        raise ValueError(f"chunk-size must be between 1 and {max_chunk}")

    file_size = os.path.getsize(path)
    target_size = min(file_size, max_bytes) if max_bytes > 0 else file_size

    with open(path, "rb") as f, serial.Serial(port, baudrate=baud, timeout=2, write_timeout=2) as ser:
        # Let CDC settle and clear stale console text.
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if stop_before_upload:
            ser.write(b"p")
            ser.flush()
            time.sleep(0.05)
            ser.reset_input_buffer()

        # Configure TX link for binary transfer profile.
        #set_rate(ser, 32000)
        #set_convolution(ser, True)
        ensure_randomizer_on(ser)
        time.sleep(1)
        ser.reset_input_buffer()

        # Enter binary upload mode.
        ser.write(b"u")
        ser.flush()
        wait_for_byte(ser, b"B")

        # Mode byte: 0 = prepend ASM in firmware, 1 = input already includes ASM.
        ser.write(b"\x01" if prepacked_asm else b"\x00")
        ser.flush()

        total = 0
        chunks = 0
        expected_chunks = (target_size + chunk_size - 1) // chunk_size if target_size > 0 else 0
        print(f"sending {target_size} bytes in up to {expected_chunks} chunks (chunk_size={chunk_size})")
        last_print_time = 0
        while total < target_size:
            remaining = target_size - total
            block = f.read(min(chunk_size, remaining))
            if not block:
                break

            pkt = struct.pack("<H", len(block)) + block
            ser.write(pkt)
            ser.flush()

            ack = read_ack(ser)
            if ack != b"K":
                print() # Newline before error
                raise RuntimeError(f"Missing ACK after chunk {chunks}, got: {ack!r}")

            total += len(block)
            chunks += 1
            
            now = time.time()
            if now - last_print_time > 0.1 or total == target_size:
                progress = (total / target_size) * 100 if target_size > 0 else 100
                bar_len = 40
                filled_len = int(bar_len * total // target_size) if target_size > 0 else bar_len
                bar = '=' * filled_len + '-' * (bar_len - filled_len)
                print(f"\r[{bar}] {progress:.1f}% | {chunks}/{expected_chunks} chunks | {total}/{target_size} bytes", end="", flush=True)
                last_print_time = now

        print() # Newline after finishing transfer

        # End transfer.
        ser.write(b"\x00\x00")
        ser.flush()

        # Polling FIFO status during drain phase
        ser.timeout = 0.1
        drain_complete = False
        last_q_time = 0
        line_buf = bytearray()
        status = {"queue": "?/?", "enq": "?", "deq": "?"}
        
        while not drain_complete:
            now = time.time()
            if now - last_q_time > 0.2:
                ser.write(b"q")
                ser.flush()
                last_q_time = now
            
            b = ser.read(1)
            if not b:
                continue
                
            if b == b"D":
                drain_complete = True
                break
                
            if b in (b"\r", b"\n"):
                if line_buf:
                    line = line_buf.decode(errors="replace")
                    line_buf.clear()
                    if "queue depth:" in line:
                        status["queue"] = line.split("queue depth:")[1].strip()
                    elif "enqueued total:" in line:
                        status["enq"] = line.split("enqueued total:")[1].strip()
                    elif "dequeued total:" in line:
                        status["deq"] = line.split("dequeued total:")[1].strip()
                        print(f"\r[Draining] FIFO Queue: {status['queue']} | Enqueued: {status['enq']} | Dequeued: {status['deq']}       ", end="", flush=True)
            else:
                line_buf.extend(b)

        ser.timeout = 2.0

        # One final poll to ensure accurate ending status
        ser.write(b"q")
        ser.flush()
        deadline = time.time() + 1.0
        while time.time() < deadline:
            line = read_text_line(ser, timeout=0.2)
            if not line:
                continue
            if "queue depth:" in line:
                status["queue"] = line.split("queue depth:")[1].strip()
            elif "enqueued total:" in line:
                status["enq"] = line.split("enqueued total:")[1].strip()
            elif "dequeued total:" in line:
                status["deq"] = line.split("dequeued total:")[1].strip()
            elif "tx drain complete:" in line:
                break
                
        print(f"\r[Draining] FIFO Queue: {status['queue']} | Enqueued: {status['enq']} | Dequeued: {status['deq']} (Drain Complete)       ")

        if start_after_upload:
            time.sleep(0.02)
            ser.write(b"s")
            ser.flush()

        if total != target_size:
            raise RuntimeError(f"size mismatch: sent {total} bytes but target is {target_size} bytes")

        print(f"done: {chunks} chunks, {total}/{target_size} bytes")


def main() -> int:
    parser = argparse.ArgumentParser(description="Push binary file to Pico modulator in framed binary chunks")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM8")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--file", required=True, help="Path to .bin file")
    parser.add_argument("--chunk-size", type=int, default=0, help="Chunk size (auto: 252 without ASM, 256 with prepacked ASM)")
    parser.add_argument("--first-frames", type=int, default=0, help="Send only first N frames/chunks from the file")
    parser.add_argument("--prepacked-asm", action="store_true", help="Input file already includes ASM at the start of each frame")
    parser.add_argument("--stop-before-upload", action="store_true", help="Send 'p' before binary upload")
    parser.add_argument("--start-after-upload", action="store_true", help="Send 's' after binary upload")
    args = parser.parse_args()

    chunk_size = args.chunk_size or (FRAME_SIZE_DEFAULT if args.prepacked_asm else PAYLOAD_SIZE_DEFAULT)
    frame_payload = FRAME_SIZE_DEFAULT if args.prepacked_asm else PAYLOAD_SIZE_DEFAULT
    max_bytes = args.first_frames * frame_payload if args.first_frames > 0 else 0

    try:
        send_file(
            args.port,
            args.baud,
            args.file,
            chunk_size,
            max_bytes,
            args.prepacked_asm,
            args.stop_before_upload,
            args.start_after_upload,
        )
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
