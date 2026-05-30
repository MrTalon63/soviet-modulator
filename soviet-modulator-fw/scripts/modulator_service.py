import argparse
import serial
import time
import sys
import os
import struct
import threading
import asyncio
import queue
from collections import defaultdict

# Max sizes prevent RAM exhaustion and natively exert TCP backpressure when full
# These defaults are dynamically overridden in main() based on the --qsize argument
HIGH_PRIORITY_QUEUE = queue.Queue()
NORMAL_PRIORITY_QUEUE = queue.Queue()
mcu_fifo_level = 0
seq_counters = defaultdict(int)
TARGET_FIFO_LEVEL = 24000


def get_status(ser):
    # Flush input buffer to remove old telemetry
    ser.read_all()
    # Ask the modulator for its current configuration
    ser.write(b"q")
    start_time = time.time()
    response = b""
    while time.time() - start_time < 2.0:
        response += ser.read_all()
        if b"Core 1 Headroom:" in response:
            break
        time.sleep(0.05)
    status = {}
    lines = response.decode("utf-8", errors="ignore").split("\n")
    for line in lines:
        line = line.strip()
        if ":" in line and not line.startswith("[MCU]"):
            key, val = line.split(":", 1)
            status[key.strip()] = val.strip()
    return status, response


def toggle_if_needed(ser, status, key, desired_state, toggle_cmd):
    current = status.get(key)
    if current is not None:
        is_on = current.startswith("ON")
        if (desired_state and not is_on) or (not desired_state and is_on):
            print(f"[Serial] Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            ser.write(toggle_cmd.encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()  # clear buffer
            return True
    return False


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
    global mcu_fifo_level, TARGET_FIFO_LEVEL
    port = args.port
    baud = args.baud
    ser_instance = [None]

    def reader_daemon():
        """Continuously reads telemetry from the MCU and prints it alongside local queue status."""
        global mcu_fifo_level
        while True:
            ser = ser_instance[0]
            if ser and ser.is_open:
                try:
                    line = ser.readline()
                    if line:
                        text = line.decode("utf-8", errors="ignore").strip()
                        if text.startswith("[MCU]"):
                            if "FIFO:" in text:
                                try:
                                    mcu_fifo_level = int(
                                        text.split("FIFO: ")[1].split("/")[0]
                                    )
                                except ValueError:
                                    pass
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
            ser = serial.Serial(port, baud, timeout=1.0, write_timeout=30.0)

            # Check if MCU is stuck in binary mode before restarting
            time.sleep(0.5)
            response = ser.read_all()
            if b"[MCU]" in response:
                print(
                    "[Serial] MCU is currently in Binary Mode. Waiting 15 seconds for watchdog to timeout..."
                )
                time.sleep(15.5)
                ser.read_all()

            print("[Serial] Restarting MCU...")
            ser.write(b"m")
            time.sleep(1.5)
            ser.close()
            time.sleep(0.5)

            print(f"[Serial] Reconnecting after MCU restart...")
            ser = serial.Serial(port, baud, timeout=10, write_timeout=30.0)

            # Apply configuration parameters
            status, response = get_status(ser)
            retry_count = 0
            while not status and retry_count < 10:
                time.sleep(0.5)
                status, response = get_status(ser)
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
                    args.inter,
                    args.rs,
                    args.ldpc,
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

                if args.inter is not None:
                    print(f"[Serial] Setting RS interleave to {args.inter}...")
                    ser.write(f"l{args.inter}\n".encode("utf-8"))
                    time.sleep(0.1)
                    ser.read_all()

                if args.rs is not None:
                    if toggle_if_needed(ser, status, "RS (255,223)", args.rs == 1, "y"):
                        status, _ = get_status(ser)
                if args.ldpc is not None:
                    if toggle_if_needed(
                        ser, status, "LDPC (8160,7136)", args.ldpc == 1, "L"
                    ):
                        status, _ = get_status(ser)
                if args.conv is not None:
                    if toggle_if_needed(
                        ser, status, "Convolutional", args.conv == 1, "c"
                    ):
                        status, _ = get_status(ser)
                if args.rand is not None:
                    if toggle_if_needed(ser, status, "Randomizer", args.rand == 1, "n"):
                        status, _ = get_status(ser)
                if args.dual is not None:
                    if toggle_if_needed(ser, status, "Dual Basis", args.dual == 1, "d"):
                        status, _ = get_status(ser)
                if args.fecf is not None:
                    if toggle_if_needed(
                        ser, status, "FECF (CRC-16)", args.fecf == 1, "e"
                    ):
                        status, _ = get_status(ser)

                # Re-fetch status after toggles are applied
                status, _ = get_status(ser)

            print("\n[Serial] --- Final Modulator Status ---")
            for k in [
                "Symbol Rate",
                "Frame Size",
                "RS Interleave",
                "RS (255,223)",
                "LDPC (8160,7136)",
                "FECF (CRC-16)",
                "Convolutional",
                "Randomizer",
                "Dual Basis",
                "Expected Payload",
            ]:
                print(f"[Serial]   {k}: {status.get(k, 'Unknown')}")

            # --- Calculate Usable Bandwidth ---
            try:
                sym_rate_str = status.get("Symbol Rate", f"{args.rate} Hz")
                symbol_rate = int(sym_rate_str.split()[0])

                frame_size_str = status.get("Frame Size", "1024 bytes")
                frame_size = int(frame_size_str.split()[0])

                conv_status = status.get("Convolutional", "OFF")
                symbols_per_frame = frame_size * 8
                if "ON" in conv_status:
                    if "1/2" in conv_status:
                        symbols_per_frame = frame_size * 16
                    elif "2/3" in conv_status:
                        symbols_per_frame = int(frame_size * 8 * 3 / 2)
                    elif "3/4" in conv_status:
                        symbols_per_frame = int(frame_size * 8 * 4 / 3)
                    elif "5/6" in conv_status:
                        symbols_per_frame = int(frame_size * 8 * 6 / 5)
                    elif "7/8" in conv_status:
                        symbols_per_frame = int(frame_size * 8 * 8 / 7)

                frames_per_sec = symbol_rate / symbols_per_frame

                expected_payload_str = status.get("Expected Payload", "882 bytes")
                payload_bytes = int(expected_payload_str.split()[0])

                usable_bytes_per_sec = frames_per_sec * payload_bytes
                usable_kbps = (usable_bytes_per_sec * 8) / 1000

                TARGET_FIFO_LEVEL = int(
                    max(8192, min(104000, usable_bytes_per_sec * 0.4))
                )
                print(
                    f"[Serial]   Usable Bandwidth: {usable_kbps:.2f} kbps ({usable_bytes_per_sec/1024:.2f} KB/s)"
                )
            except Exception:
                pass
            print("[Serial] ------------------------------\n")

            # Start modulation and enter binary mode
            # Note: Initial OID idle frames will transmit until user data arrives
            ser.write(b"s")
            time.sleep(1.0)  # Allow service queues to populate (increased from 0.5s)
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
            keep_alive_seq = 0
            last_keep_alive = time.time()

            while True:
                priority_tag = "HIGH"
                apid = None
                payload = None

                try:
                    # Always check high priority first (non-blocking)
                    apid, payload = HIGH_PRIORITY_QUEUE.get_nowait()
                except queue.Empty:
                    priority_tag = "NORMAL"
                    try:
                        # Wait up to 2 seconds for a normal priority packet, then send keep-alive if empty
                        apid, payload = NORMAL_PRIORITY_QUEUE.get(timeout=2.0)
                    except queue.Empty:
                        # No user packets. Send a minimal keep-alive packet to prevent MCU watchdog timeout.
                        # The MCU still generates idle VCDUs internally.
                        packet = generate_space_packet(
                            2047,
                            keep_alive_seq,
                            b"\xff\xff\xff\xff\xff\xff\xff\xff",
                        )
                        keep_alive_seq = (keep_alive_seq + 1) & 0x3FFF

                        ser.write(packet)
                        mcu_fifo_level += len(packet)
                        continue

                # We have a valid user packet - send it with flow control
                seq = seq_counters[apid]
                seq_counters[apid] = (seq + 1) & 0x3FFF

                packet = generate_space_packet(apid, seq, payload)

                # Dynamic Flow Control: ~400ms buffer based on stream speed to perfectly absorb
                # 100ms telemetry latency without underflow, while keeping responsiveness high
                while mcu_fifo_level > TARGET_FIFO_LEVEL:
                    time.sleep(0.01)

                ser.write(packet)
                mcu_fifo_level += len(
                    packet
                )  # Predict level to prevent instantly overfilling

                if priority_tag == "HIGH":
                    HIGH_PRIORITY_QUEUE.task_done()
                else:
                    NORMAL_PRIORITY_QUEUE.task_done()

        except ImportError as e:
            print(f"\n[Fatal Error] {e}")
            os._exit(1)  # Immediately kill the entire multi-threaded daemon
        except serial.SerialException as e:
            ser_instance[0] = None  # Safely disconnect the reader daemon
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
            print(f"\n[Serial] Connection lost: {e}")
            print("[Serial] Attempting to reconnect in 3 seconds...")
            time.sleep(3.0)
        except Exception as e:
            ser_instance[0] = None
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
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

    parser.add_argument(
        "--rate", type=int, default=1000000, help="Set symbol rate (Hz)"
    )
    parser.add_argument(
        "--crate",
        type=int,
        default=4,
        choices=[0, 1, 2, 3, 4],
        help="Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)",
    )
    parser.add_argument(
        "--inter",
        type=int,
        choices=[1, 2, 4, 5, 8],
        help="Set RS interleave depth",
    )
    parser.add_argument(
        "--rs",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Reed-Solomon",
    )
    parser.add_argument(
        "--ldpc",
        type=int,
        default=0,
        choices=[0, 1],
        help="0=Disable, 1=Enable LDPC",
    )
    parser.add_argument(
        "--conv",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Convolutional",
    )
    parser.add_argument(
        "--rand",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Randomizer",
    )
    parser.add_argument(
        "--dual", type=int, choices=[0, 1], help="0=Disable, 1=Enable Dual Basis"
    )
    parser.add_argument(
        "--fecf", type=int, default=1, choices=[0, 1], help="0=Disable, 1=Enable FECF"
    )
    parser.add_argument(
        "--qsize",
        type=int,
        default=50,
        help="Max items per priority queue (default 50, ~50KB at 1KB/item)",
    )

    args = parser.parse_args()

    global HIGH_PRIORITY_QUEUE, NORMAL_PRIORITY_QUEUE
    HIGH_PRIORITY_QUEUE = queue.Queue(maxsize=args.qsize)
    NORMAL_PRIORITY_QUEUE = queue.Queue(maxsize=args.qsize)

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
