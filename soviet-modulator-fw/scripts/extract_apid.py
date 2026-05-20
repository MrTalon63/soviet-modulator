import argparse
import os
import sys
import socket
import time


class InputSource:
    def __init__(self, source):
        self.is_udp = source.startswith("udp://")
        if self.is_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            host, port = source[6:].split(":")
            self.sock.bind((host, int(port)))
            self.f = None
        else:
            self.f = open(source, "rb")

    def read_chunk(self):
        if self.is_udp:
            return self.sock.recv(65536)
        else:
            return self.f.read(65536)


class OutputTarget:
    def __init__(self, dest):
        self.is_udp = dest.startswith("udp://")
        if self.is_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            host, port = dest[6:].split(":")
            self.dest = (host, int(port))
            self.f = None
        else:
            self.f = open(dest, "wb")

    def write(self, data):
        if self.is_udp:
            idx = 0
            while idx < len(data):
                self.sock.sendto(data[idx : idx + 65000], self.dest)
                idx += 65000
        else:
            self.f.write(data)

    def close(self):
        if self.f:
            self.f.close()


def process_buffer(buffer, target_apid, out_dest, stats):
    """
    Parses complete Space Packets from the byte buffer.
    Returns the remaining incomplete bytes.
    """
    while len(buffer) >= 6:
        # Parse Space Packet Header (6 bytes)
        apid = ((buffer[0] & 0x07) << 8) | buffer[1]
        seq = ((buffer[2] & 0x3F) << 8) | buffer[3]
        pdl = (buffer[4] << 8) | buffer[5]
        pkt_len = pdl + 7

        # If we don't have the full packet yet, break and wait for more data
        if len(buffer) < pkt_len:
            break

        packet = buffer[:pkt_len]
        buffer = buffer[pkt_len:]

        if apid == target_apid:
            # Strip the 6-byte SP header and write the raw payload data
            payload = packet[6:]

            # Check sequence gap to prevent pixel shifts during RF dropouts
            if stats["last_seq"] is not None:
                if seq == stats["last_seq"]:
                    # Duplicated packet detected! Ignore it to prevent pixel shift!
                    stats["duplicated_packets"] += 1
                    continue
                expected_seq = (stats["last_seq"] + 1) & 0x3FFF
                if seq != expected_seq:
                    missing = (seq - expected_seq) % 0x4000
                    # Sanity limit: only pad if gap is reasonable (prevents infinite files on stream restart)
                    if missing < 500:
                        stats["padded_packets"] += missing
                        out_dest.write(b"\x00" * (missing * len(payload)))
            else:
                # First packet received! If it's not seq 0, pad the beginning of the file!
                if seq > 0 and seq < 500:
                    stats["padded_packets"] += seq
                    out_dest.write(b"\x00" * (seq * len(payload)))

            stats["last_seq"] = seq
            out_dest.write(payload)
            stats["extracted_bytes"] += len(payload)
            stats["extracted_packets"] += 1
        elif apid == 2047:
            stats["filler_packets"] += 1
        else:
            stats["other_packets"] += 1

    return buffer


def main():
    parser = argparse.ArgumentParser(
        description="Extract APID payload data from SatDump AOS frames."
    )
    parser.add_argument(
        "input_source",
        help="Input binary file from SatDump, or udp://IP:PORT to listen for live streams",
    )
    parser.add_argument(
        "output_dest",
        help="Output binary file, or udp://IP:PORT to stream extracted payloads",
    )
    parser.add_argument(
        "--apid",
        type=int,
        default=1,
        help="Application Process ID to extract (default: 1)",
    )
    parser.add_argument(
        "--vcid", type=int, default=0, help="Virtual Channel ID to filter (default: 0)"
    )
    parser.add_argument(
        "--format",
        choices=["cadu", "vcdu"],
        default="cadu",
        help="Input format: 'cadu' (with ASM) or 'vcdu' (stripped)",
    )
    parser.add_argument(
        "--inter",
        type=int,
        default=4,
        choices=[1, 2, 4, 5, 8],
        help="Reed-Solomon interleave depth (default: 4)",
    )
    parser.add_argument(
        "--no-rs",
        action="store_true",
        help="Indicate Reed-Solomon was disabled (VCDU size uses full payload space)",
    )
    parser.add_argument(
        "--fecf",
        action="store_true",
        help="Indicate Frame Error Control Field (CRC-16) is present at the end of frames",
    )

    args = parser.parse_args()

    if not args.input_source.startswith("udp://") and not os.path.exists(
        args.input_source
    ):
        print(f"Error: Input file '{args.input_source}' not found.")
        sys.exit(1)

    vcdu_size = (255 * args.inter) if args.no_rs else (223 * args.inter)

    stats = {
        "extracted_packets": 0,
        "extracted_bytes": 0,
        "filler_packets": 0,
        "other_packets": 0,
        "dropped_gaps": 0,
        "padded_packets": 0,
        "fragmented_drops": 0,
        "duplicated_frames": 0,
        "duplicated_packets": 0,
        "last_seq": None,
    }

    in_src = InputSource(args.input_source)
    out_dest = OutputTarget(args.output_dest)

    print(f"Listening/Reading from: {args.input_source}")
    print(f"Writing/Streaming to:   {args.output_dest}")
    print("Parsing frames...")

    last_fc = None
    sync_state = False
    sp_buffer = bytearray()
    frame_buffer = bytearray()
    asm = b"\x1a\xcf\xfc\x1d"
    search_idx = 0
    last_print = time.time()

    try:
        while True:
            chunk = in_src.read_chunk()
            if not chunk:
                break  # EOF for files

            frame_buffer.extend(chunk)

            while True:
                if args.format == "cadu":
                    idx = frame_buffer.find(asm, search_idx)
                    if idx == -1:
                        if len(frame_buffer) > 3:
                            frame_buffer = frame_buffer[-3:]
                        search_idx = 0
                        break
                    if len(frame_buffer) >= idx + 4 + vcdu_size:
                        vcdu = frame_buffer[idx + 4 : idx + 4 + vcdu_size]
                        search_idx = idx + 4
                    else:
                        frame_buffer = frame_buffer[idx:]
                        search_idx = 0
                        break
                else:
                    if len(frame_buffer) >= search_idx + vcdu_size:
                        vcdu = frame_buffer[search_idx : search_idx + vcdu_size]
                        search_idx += vcdu_size
                    else:
                        frame_buffer = frame_buffer[search_idx:]
                        search_idx = 0
                        break

                # Parse VCDU Header
                parsed_vcid = vcdu[1] & 0x3F
                if parsed_vcid != args.vcid:
                    continue

                frame_count = (vcdu[2] << 16) | (vcdu[3] << 8) | vcdu[4]

                # Check continuity
                if last_fc is not None:
                    if frame_count == last_fc:
                        # SDR output the same frame twice due to a PLL slip. Ignore it entirely!
                        stats["duplicated_frames"] += 1
                        continue
                    if frame_count != (last_fc + 1) & 0xFFFFFF:
                        # Frame drop detected! Discard corrupted partial packet in buffer
                        stats["dropped_gaps"] += 1
                        sync_state = False
                        if len(sp_buffer) > 0:
                            stats["fragmented_drops"] += 1
                        sp_buffer.clear()

                last_fc = frame_count

                # Parse MPDU Header
                fhp = (vcdu[6] << 8) | vcdu[7]  # Full 16-bit FHP for AOS

                # Packet Zone bounds
                pz_start = 8
                pz_end = vcdu_size - (2 if args.fecf else 0)
                pz_data = vcdu[pz_start:pz_end]

                if fhp == 0xFFFE:
                    # Frame contains only Idle Data
                    continue

                if fhp == 0xFFFF:
                    # No packet starts in this frame. It's a pure continuation of the previous packet.
                    if sync_state:
                        sp_buffer.extend(pz_data)
                        sp_buffer = process_buffer(
                            sp_buffer, args.apid, out_dest, stats
                        )
                    continue

                if fhp < len(pz_data):
                    # A new packet starts exactly at index 'fhp' within the packet zone!
                    if sync_state:
                        # Finish assembling the tail of the previous packet
                        sp_buffer.extend(pz_data[:fhp])
                        sp_buffer = process_buffer(
                            sp_buffer, args.apid, out_dest, stats
                        )
                        # If there's leftover garbage here, it implies a malformed packet length. Flush it.
                        if len(sp_buffer) > 0:
                            stats["fragmented_drops"] += 1
                            sp_buffer.clear()
                    else:
                        # We were out of sync. Discard any stray garbage bytes before locking onto the new header!
                        sp_buffer.clear()

                    # We are perfectly locked now!
                    sync_state = True

                    # Start assembling the new packet(s)
                    sp_buffer.extend(pz_data[fhp:])
                    sp_buffer = process_buffer(sp_buffer, args.apid, out_dest, stats)
                else:
                    # FHP is >= payload size. This means the FHP header was corrupted by RF noise!
                    stats["fragmented_drops"] += 1
                    sync_state = False
                    sp_buffer.clear()

            now = time.time()
            if now - last_print > 1.0:
                sys.stdout.write(
                    f"\r[Live] Extracted: {stats['extracted_packets']} pkts | Payload: {stats['extracted_bytes'] / 1024:.2f} KB | Drops: {stats['dropped_gaps']}    "
                )
                sys.stdout.flush()
                last_print = now

    except KeyboardInterrupt:
        print("\nStreaming interrupted by user.")
    finally:
        out_dest.close()

    print("\n\nExtraction Complete!")
    print(f"Target APID:       {args.apid}")
    print(f"Target VCID:       {args.vcid}")
    print("---------------------------------")
    print(f"Extracted Packets: {stats['extracted_packets']}")
    print(f"Extracted Payload: {stats['extracted_bytes'] / 1024:.2f} KB")
    print(f"Filler Packets:    {stats['filler_packets']} (Discarded)")
    print(
        f"Padded Packets:    {stats['padded_packets']} (Zero-filled to preserve image alignment)"
    )
    print(f"Duplicated Frames: {stats['duplicated_frames']} (Ignored)")
    print(f"Duplicated Pkts:   {stats['duplicated_packets']} (Ignored)")
    print(f"Other APIDs:       {stats['other_packets']} (Ignored)")
    print(f"Drop Gaps Handled: {stats['dropped_gaps']}")
    print(f"Fragmented Drops:  {stats['fragmented_drops']} (Destroyed by gap)")

    if stats["extracted_bytes"] > 0:
        if args.output_dest.startswith("udp://"):
            print(f"\nStreamed successfully to: {args.output_dest}")
        else:
            print(f"\nSaved successfully to: {args.output_dest}")
    else:
        print("\nWarning: No matching APID payload data was found!")


if __name__ == "__main__":
    main()
