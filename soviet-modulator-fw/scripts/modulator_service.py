import argparse
import serial
import time
import sys
import struct
import threading
import asyncio
import queue
from collections import defaultdict

# Max sizes prevent RAM exhaustion and natively exert TCP backpressure when full
HIGH_PRIORITY_QUEUE = queue.Queue(maxsize=5000)
NORMAL_PRIORITY_QUEUE = queue.Queue(maxsize=5000)

seq_counters = defaultdict(int)


def get_status(ser):
    # Ask the modulator for its current configuration
    ser.write(b"q")
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
            print(f"[Serial] Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            ser.write(toggle_cmd.encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()  # clear buffer


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


def serial_worker(args):
    """Maintains the USB CDC connection and streams from queues to the Modulator."""
    port = args.port
    baud = args.baud
    ser_instance = [None]

    def reader_daemon():
        """Continuously reads telemetry from the MCU and prints it alongside local queue status."""
        while True:
            ser = ser_instance[0]
            if ser and ser.is_open:
                try:
                    line = ser.readline()
                    if line:
                        text = line.decode("utf-8", errors="ignore").strip()
                        if text.startswith("[MCU]"):
                            sys.stdout.write(
                                f"\r{text} | High Q: {HIGH_PRIORITY_QUEUE.qsize()} | Normal Q: {NORMAL_PRIORITY_QUEUE.qsize()}    "
                            )
                            sys.stdout.flush()
                        else:
                            sys.stdout.write(f"\n{text}\n")
                            sys.stdout.flush()
                except serial.SerialException:
                    pass
            else:
                time.sleep(0.1)

    threading.Thread(target=reader_daemon, daemon=True).start()

    while True:
        try:
            print(f"[Serial] Connecting to {port} at {baud} baud...")
            ser = serial.Serial(port, baud, timeout=10, write_timeout=30.0)

            # Apply configuration parameters
            status = get_status(ser)
            retry_count = 0
            while not status and retry_count < 10:
                time.sleep(0.5)
                status = get_status(ser)
                retry_count += 1

            if not status:
                raise serial.SerialException(
                    "MCU not responding to 'q' status command."
                )

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
                print("[Serial] Applying desired FEC settings...")

                if args.rate is not None:
                    print(f"[Serial] Setting symbol rate to {args.rate} Hz...")
                    ser.write(f"r{args.rate}\n".encode("utf-8"))
                    time.sleep(0.1)
                    ser.read_all()

                if args.crate is not None:
                    print(
                        f"[Serial] Setting convolutional rate to {['1/2','2/3','3/4','5/6','7/8'][args.crate]}..."
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

                # Re-fetch status after toggles are applied
                status = get_status(ser)

            print("\n[Serial] --- Final Modulator Status ---")
            for k in [
                "RS (255,223)",
                "FECF (CRC-16)",
                "Convolutional",
                "Randomizer",
                "Dual Basis",
                "Expected Payload",
            ]:
                print(f"[Serial]   {k}: {status.get(k, 'Unknown')}")
            print("[Serial] ------------------------------\n")

            # Start modulation and enter binary mode
            ser.write(b"s")
            time.sleep(0.5)
            ser.read_all()
            ser.write(b"u")

            # Wait securely for the 'B' ready signal
            ready = False
            start_wait = time.time()
            response = b""
            while time.time() - start_wait < 3.0:
                response += ser.read_all()
                if b"B" in response:
                    ready = True
                    break
                time.sleep(0.05)

            if not ready:
                print(
                    "[Serial] Warning: Did not receive 'B' ready signal. Proceeding anyway."
                )
            else:
                print("[Serial] Modulator locked into Binary Mode. Stream ready!")

            # Expose the PySerial instance to the reader daemon now that initialization is complete
            ser_instance[0] = ser
            filler_seq = 0

            while True:
                priority_tag = "HIGH"
                try:
                    # Always check high priority first (non-blocking)
                    apid, payload = HIGH_PRIORITY_QUEUE.get_nowait()
                except queue.Empty:
                    priority_tag = "NORMAL"
                    try:
                        # Block for 2 seconds waiting for normal priority
                        apid, payload = NORMAL_PRIORITY_QUEUE.get(timeout=2.0)
                    except queue.Empty:
                        # The queues are empty. Push a small APID 2047 filler packet to the MCU
                        # to prevent the MCU's 5-second Binary Mode Watchdog from timing out!
                        packet = generate_space_packet(2047, filler_seq, b"\x00" * 16)
                        filler_seq = (filler_seq + 1) & 0x3FFF

                        written = 0
                        while written < len(packet):
                            chunk_size = min(64, len(packet) - written)
                            res = ser.write(packet[written : written + chunk_size])
                            if res:
                                written += res
                        continue

                # We have a valid user packet
                seq = seq_counters[apid]
                seq_counters[apid] = (seq + 1) & 0x3FFF

                packet = generate_space_packet(apid, seq, payload)

                # Write in 64-byte chunks to perfectly bypass Windows usbser.sys buffer overflow bugs
                written = 0
                while written < len(packet):
                    chunk_size = min(64, len(packet) - written)
                    res = ser.write(packet[written : written + chunk_size])
                    if res:
                        written += res

                if priority_tag == "HIGH":
                    HIGH_PRIORITY_QUEUE.task_done()
                else:
                    NORMAL_PRIORITY_QUEUE.task_done()

        except serial.SerialException as e:
            ser_instance[0] = None  # Safely disconnect the reader daemon
            print(f"\n[Serial] Connection lost: {e}")
            print("[Serial] Attempting to reconnect in 3 seconds...")
            time.sleep(3.0)
        except Exception as e:
            ser_instance[0] = None
            print(f"\n[Serial] Unexpected error: {e}")
            time.sleep(3.0)


async def client_handler(reader, writer):
    """Parses incoming TCP packets and routes them to the appropriate priority queue."""
    addr = writer.get_extra_info("peername")
    print(f"[TCP] Client connected from {addr}")
    try:
        while True:
            # Protocol Header: Priority(1 byte), APID(2 bytes), Length(4 bytes) = 7 bytes total
            try:
                header = await reader.readexactly(7)
            except asyncio.IncompleteReadError:
                break  # Clean client disconnect (EOF)

            priority, apid, length = struct.unpack("!B H I", header)

            try:
                payload = await reader.readexactly(length)
            except asyncio.IncompleteReadError:
                break  # Clean client disconnect (EOF)

            # Put into queue. Running in a thread pool yields the event loop for other
            # clients, while still blocking this specific stream to exert TCP backpressure!
            if priority == 1:
                await asyncio.to_thread(HIGH_PRIORITY_QUEUE.put, (apid, payload))
            else:
                await asyncio.to_thread(NORMAL_PRIORITY_QUEUE.put, (apid, payload))

    except Exception as e:
        print(f"[TCP] Client {addr} error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"[TCP] Client disconnected: {addr}")


async def start_tcp_server(host, port):
    server = await asyncio.start_server(client_handler, host, port)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    print(f"[TCP] Service listening for incoming streams on {addrs}")

    async with server:
        await server.serve_forever()


def main():
    parser = argparse.ArgumentParser(
        description="Continuous BPSK USB/TCP Modulator Service"
    )
    parser.add_argument("port", help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument(
        "--baud", type=int, default=12000000, help="Baud rate (default 12000000)"
    )
    parser.add_argument(
        "--bind", default="0.0.0.0", help="TCP bind address (default 0.0.0.0)"
    )
    parser.add_argument(
        "--tcpport", type=int, default=8000, help="TCP listen port (default 8000)"
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

    args = parser.parse_args()

    # Start the serial worker thread
    serial_thread = threading.Thread(target=serial_worker, args=(args,), daemon=True)
    serial_thread.start()

    # Run TCP server on the main thread
    try:
        asyncio.run(start_tcp_server(args.bind, args.tcpport))
    except KeyboardInterrupt:
        print("\nShutting down Modulator Service...")
        sys.exit(0)


if __name__ == "__main__":
    main()
