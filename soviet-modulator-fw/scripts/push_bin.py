import argparse
import serial
import time
import sys
import os


def get_status(ser):
    # Ask the modulator for its current configuration
    ser.write(b"q")

    # Read until we see the final line to guarantee we captured the whole block
    start_time = time.time()
    response = b""
    while time.time() - start_time < 2.0:
        response += ser.read_all()
        if b"upload active:" in response:
            break
        time.sleep(0.05)

    status = {}
    lines = response.decode("utf-8", errors="ignore").split("\n")
    for line in lines:
        line = line.strip()
        if ":" in line:
            key, val = line.split(":", 1)
            status[key.strip()] = val.strip()
    return status


def toggle_if_needed(ser, status, key, desired_state, toggle_cmd):
    current = status.get(key)
    if current is not None:
        is_on = current.startswith("ON")
        if (desired_state and not is_on) or (not desired_state and is_on):
            print(f"Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            ser.write(toggle_cmd.encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()  # clear buffer


def print_progress(bytes_sent, total_size, start_time):
    elapsed = time.time() - start_time
    speed_kbps = ((bytes_sent * 8) / elapsed) / 1000 if elapsed > 0 else 0

    if total_size:
        percent = (bytes_sent / total_size) * 100
        bar_len = 30
        filled = int(bar_len * bytes_sent / total_size)
        bar = "=" * filled + "-" * (bar_len - filled)
        sys.stdout.write(
            f"\r[{bar}] {percent:.1f}% | {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps    "
        )
    else:
        sys.stdout.write(
            f"\rStreaming... | {bytes_sent/1024:.1f} KB sent | {speed_kbps:.1f} kbps    "
        )
    sys.stdout.flush()


def generate_space_packet(apid, seq, payload):
    header = bytearray(6)
    header[0] = (apid >> 8) & 0x07
    header[1] = apid & 0xFF
    header[2] = 0xC0 | ((seq >> 8) & 0x3F)
    header[3] = seq & 0xFF
    pdl = len(payload) - 1
    header[4] = (pdl >> 8) & 0xFF
    header[5] = pdl & 0xFF
    return header + payload


def main():
    parser = argparse.ArgumentParser(
        description="Upload binary frames to BPSK Modulator"
    )
    parser.add_argument("port", help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument(
        "--baud",
        type=int,
        default=12000000,
        help="Baud rate (default 12000000 to uncap OS driver)",
    )
    parser.add_argument("--rate", type=int, help="Set symbol rate (Hz)")
    parser.add_argument(
        "--crate",
        type=int,
        choices=[0, 1, 2, 3, 4],
        help="Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)",
    )
    parser.add_argument(
        "--rs", type=int, choices=[0, 1], help="0=Disable, 1=Enable Reed-Solomon"
    )
    parser.add_argument(
        "--conv", type=int, choices=[0, 1], help="0=Disable, 1=Enable Convolutional"
    )
    parser.add_argument(
        "--rand", type=int, choices=[0, 1], help="0=Disable, 1=Enable Randomizer"
    )
    parser.add_argument(
        "--dual", type=int, choices=[0, 1], help="0=Disable, 1=Enable Dual Basis"
    )
    parser.add_argument(
        "--fecf", type=int, choices=[0, 1], help="0=Disable, 1=Enable FECF"
    )
    parser.add_argument(
        "--apid",
        type=int,
        default=1,
        help="Application Process ID (APID) for Space Packets (default: 1)",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=1000,
        help="Number of payload bytes per Space Packet (default: 1000)",
    )
    parser.add_argument(
        "file", help="Binary file to upload (use '-' for standard input)"
    )

    args = parser.parse_args()

    if args.file != "-" and not os.path.exists(args.file):
        print(f"Error: File {args.file} not found.")
        return

    print(f"Connecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=10, write_timeout=300.0)

    # Apply configuration parameters if passed in
    status = get_status(ser)
    retry_count = 0
    while not status and retry_count < 10:
        time.sleep(0.5)
        status = get_status(ser)
        retry_count += 1

    if not status:
        print("Error: Could not read status from MCU. Is it stuck in a boot loop?")
        sys.exit(1)

    if any(
        a is not None
        for a in [
            args.rate,
            args.crate,
            args.rs,
            args.conv,
            args.rand,
            args.dual,
            args.fecf,
        ]
    ):
        print("\nApplying desired FEC settings...")

        if args.rate is not None:
            print(f"Setting symbol rate to {args.rate} Hz...")
            ser.write(f"r{args.rate}\n".encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()

        if args.crate is not None:
            print(
                f"Setting convolutional rate to {['1/2','2/3','3/4','5/6','7/8'][args.crate]}..."
            )
            ser.write(f"k{args.crate}\n".encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()

        if args.rs is not None:
            toggle_if_needed(ser, status, "RS (255,223)", args.rs == 1, "y")
        if args.conv is not None:
            toggle_if_needed(ser, status, "Convolutional", args.conv == 1, "c")
        if args.rand is not None:
            toggle_if_needed(ser, status, "Randomizer", args.rand == 1, "n")
        if args.dual is not None:
            toggle_if_needed(ser, status, "Dual Basis", args.dual == 1, "d")
        if args.fecf is not None:
            toggle_if_needed(ser, status, "FECF (CRC-16)", args.fecf == 1, "e")

        # Re-fetch status after toggles are applied to get the new Expected Payload
        status = get_status(ser)

    print("\n--- Final Modulator Status ---")
    for k in [
        "RS (255,223)",
        "FECF (CRC-16)",
        "Convolutional",
        "Randomizer",
        "Dual Basis",
        "Expected Payload",
    ]:
        print(f"  {k}: {status.get(k, 'Unknown')}")
    print("------------------------------\n")

    # Ensure modulation is started
    print("Starting modulation...")
    ser.write(b"s")
    time.sleep(0.1)
    ser.read_all()

    # Enter binary upload mode
    print("Entering binary upload mode and resetting MCU buffers...")
    ser.write(b"u")

    # Wait securely for the 'B' ready signal
    ready = False
    start_wait = time.time()
    response = b""
    while time.time() - start_wait < 2.0:
        response += ser.read_all()
        if b"B" in response:
            ready = True
            break
        time.sleep(0.05)

    if not ready:
        print(
            "Warning: Did not receive 'B' ready signal. Modulator might not be ready."
        )

    print("Waiting 2.0s for SDR to achieve PLL lock on the idle carrier...")
    time.sleep(2.0)

    print(f"Starting upload from {'stdin' if args.file == '-' else args.file}...")

    # Read and upload file in chunks
    f = sys.stdin.buffer if args.file == "-" else open(args.file, "rb")
    total_size = os.path.getsize(args.file) if args.file != "-" else None
    bytes_sent = 0
    start_time = time.time()
    last_ui_update = 0

    seq = 0
    sp_payload_size = args.size  # Keep Space Packets comfortably inside FIFO capacity

    try:
        while True:
            chunk = f.read(sp_payload_size)
            if not chunk:
                break

            packet = generate_space_packet(args.apid, seq, chunk)
            seq = (seq + 1) & 0x3FFF

            # Write directly; TinyUSB CDC backpressure will safely block PySerial if the 128KB MCU FIFO fills!
            # We chunk this to 64 bytes to perfectly bypass Windows usbser.sys buffer overflow bugs
            written = 0
            while written < len(packet):
                try:
                    chunk_size = min(64, len(packet) - written)
                    res = ser.write(packet[written : written + chunk_size])
                    if res:
                        written += res
                except serial.SerialTimeoutException:
                    print(
                        "\n\nError: USB Write Timeout. The MCU stopped receiving data."
                    )
                    sys.exit(1)

            bytes_sent += len(chunk)

            now = time.time()
            if now - last_ui_update > 0.2:  # Update console at 5Hz
                print_progress(bytes_sent, total_size, start_time)
                last_ui_update = now

        print_progress(bytes_sent, total_size, start_time)

        print(
            "\n\nUpload pushed to hardware FIFO successfully!\nThe modulator will continuously transmit the buffered packets."
        )

        print("Waiting for MCU to empty USB buffer and exit binary mode...", end="")
        sys.stdout.flush()
        while True:
            line = ser.readline()
            if b"Binary mode timeout" in line:
                print()
                break
            if not line:
                sys.stdout.write(".")
                sys.stdout.flush()

        print("Monitoring FIFO drain progress...")
        while True:
            status = get_status(ser)
            if "FIFO depth" not in status:
                # MCU hasn't responded to 'q' yet, wait and try again
                time.sleep(1.0)
                continue

            fifo_str = status.get("FIFO depth", "0")
            frames_str = status.get("TX Frames pending", "0")
            chunks_str = status.get("Active User Chunks", "0")
            try:
                depth = int(fifo_str.split("/")[0])
                frames = int(frames_str)
                chunks = int(chunks_str)
                if depth == 0 and frames == 0 and chunks == 0:
                    time.sleep(
                        1.0
                    )  # Give it 1 extra second to ensure the final PIO bits physically left the antenna
                    print("\nQueues drained completely! Transmission finished.")
                    break
                sys.stdout.write(
                    f"\rDraining: {depth} bytes | {frames} frames | {chunks} chunks left...    "
                )
                sys.stdout.flush()
            except ValueError:
                pass
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\nOperation interrupted by user. Closing...")
    finally:
        if args.file != "-" and "f" in locals():
            f.close()
        ser.close()

    total_time = time.time() - start_time
    avg_kbps = ((bytes_sent * 8) / total_time) / 1000 if total_time > 0 else 0
    print(f"Total transmission time: {total_time:.2f} seconds")
    print(f"Average data rate: {avg_kbps:.2f} kbps")


if __name__ == "__main__":
    main()
