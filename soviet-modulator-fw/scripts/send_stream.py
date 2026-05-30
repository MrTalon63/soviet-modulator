import argparse
import socket
import sys
import struct
import os
import time


def main():
    parser = argparse.ArgumentParser(
        description="Client to push files/streams to the Modulator Service"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Service host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Service port (default: 8000)"
    )
    parser.add_argument(
        "--apid", type=int, default=1, help="APID for these packets (default: 1)"
    )
    parser.add_argument(
        "--high-priority", action="store_true", help="Route to High Priority queue"
    )
    parser.add_argument(
        "--size",
        type=int,
        default=1000,
        help="Payload chunk size per packet (default: 1000)",
    )
    parser.add_argument("file", help="File to send (use '-' for live stdin streaming)")
    args = parser.parse_args()

    priority = 1 if args.high_priority else 0

    print(f"Connecting to Modulator Service at {args.host}:{args.port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
    except Exception as e:
        print(f"Failed to connect to service: {e}")
        sys.exit(1)

    f = sys.stdin.buffer if args.file == "-" else open(args.file, "rb")
    total_size = os.path.getsize(args.file) if args.file != "-" else None

    print(
        f"Streaming data with APID {args.apid} | Priority: {'HIGH' if priority else 'NORMAL'}..."
    )
    bytes_sent = 0
    start_time = time.time()
    last_update = start_time - 0.5
    last_bytes = 0

    try:
        while True:
            chunk = f.read(args.size)
            if not chunk:
                break

            # Protocol: [Priority: 1B] [APID: 2B] [Length: 4B] [Payload...]
            header = struct.pack("!B H I", priority, args.apid, len(chunk))
            sock.sendall(header + chunk)
            bytes_sent += len(chunk)

            now = time.time()
            if now - last_update >= 0.5:
                speed_kbps = (
                    ((bytes_sent - last_bytes) * 8) / (now - last_update)
                ) / 1000
                last_bytes = bytes_sent
                last_update = now
                if total_size:
                    percent = (bytes_sent / total_size) * 100
                    sys.stdout.write(
                        f"\r[{percent:.1f}%] Sent {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                    )
                else:
                    sys.stdout.write(
                        f"\rStreaming... | Sent {bytes_sent/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                    )
                sys.stdout.flush()
    except Exception as e:
        print(f"\nError sending data: {e}")
    finally:
        if args.file != "-":
            f.close()
        sock.close()
        print("\nTransmission complete. Socket closed.")


if __name__ == "__main__":
    main()
