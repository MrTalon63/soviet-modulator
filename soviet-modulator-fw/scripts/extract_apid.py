import argparse
import os
import sys


def process_buffer(buffer, target_apid, out_file, stats):
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
                        out_file.write(b"\x00" * (missing * len(payload)))
            else:
                # First packet received! If it's not seq 0, pad the beginning of the file!
                if seq > 0 and seq < 500:
                    stats["padded_packets"] += seq
                    out_file.write(b"\x00" * (seq * len(payload)))

            stats["last_seq"] = seq
            out_file.write(payload)
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
    parser.add_argument("input_file", help="Input binary file from SatDump")
    parser.add_argument(
        "output_file", help="Output binary file for extracted payload data"
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
        help="Input format: 'cadu' (1024 bytes with ASM) or 'vcdu' (stripped, default 892 bytes)",
    )
    parser.add_argument(
        "--no-rs",
        action="store_true",
        help="Indicate Reed-Solomon was disabled (VCDU size 1020 instead of 892)",
    )
    parser.add_argument(
        "--fecf",
        action="store_true",
        help="Indicate Frame Error Control Field (CRC-16) is present at the end of frames",
    )

    args = parser.parse_args()

    if not os.path.exists(args.input_file):
        print(f"Error: Input file '{args.input_file}' not found.")
        sys.exit(1)

    vcdu_size = 1020 if args.no_rs else 892

    # Determine frame boundaries based on format
    with open(args.input_file, "rb") as f:
        data = f.read()

    print(f"Loaded {len(data)} bytes. Parsing frames...")

    vcdus = []
    if args.format == "cadu":
        asm = b"\x1a\xcf\xfc\x1d"
        idx = 0
        while True:
            idx = data.find(asm, idx)
            if idx == -1:
                break
            if idx + 4 + vcdu_size <= len(data):
                vcdus.append(data[idx + 4 : idx + 4 + vcdu_size])
                idx += 4  # Advance just past the ASM so we can seamlessly resynchronize if the SDR drops a byte!
            else:
                break
    else:
        # Raw VCDUs (SatDump .frames output)
        for i in range(0, len(data), vcdu_size):
            chunk = data[i : i + vcdu_size]
            if len(chunk) == vcdu_size:
                vcdus.append(chunk)

    print(f"Found {len(vcdus)} valid Transfer Frames.")

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

    last_fc = None
    sync_state = False
    buffer = bytearray()

    with open(args.output_file, "wb") as out_f:
        for vcdu in vcdus:
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
                    if len(buffer) > 0:
                        stats["fragmented_drops"] += 1
                    buffer.clear()

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
                    buffer.extend(pz_data)
                    buffer = process_buffer(buffer, args.apid, out_f, stats)
                continue

            if fhp < len(pz_data):
                # A new packet starts exactly at index 'fhp' within the packet zone!
                if sync_state:
                    # Finish assembling the tail of the previous packet
                    buffer.extend(pz_data[:fhp])
                    buffer = process_buffer(buffer, args.apid, out_f, stats)
                    # If there's leftover garbage here, it implies a malformed packet length. Flush it.
                    if len(buffer) > 0:
                        stats["fragmented_drops"] += 1
                        buffer.clear()
                else:
                    # We were out of sync. Discard any stray garbage bytes before locking onto the new header!
                    buffer.clear()

                # We are perfectly locked now!
                sync_state = True

                # Start assembling the new packet(s)
                buffer.extend(pz_data[fhp:])
                buffer = process_buffer(buffer, args.apid, out_f, stats)
            else:
                # FHP is >= payload size. This means the FHP header was corrupted by RF noise!
                stats["fragmented_drops"] += 1
                sync_state = False
                buffer.clear()

    print("\nExtraction Complete!")
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
        print(f"\nSaved successfully to: {args.output_file}")
    else:
        print("\nWarning: No matching APID payload data was found!")


if __name__ == "__main__":
    main()
