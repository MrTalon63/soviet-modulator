import argparse
import serial
import time
import sys
import os

def get_status(ser):
    # Ask the modulator for its current configuration
    ser.write(b'q')
    
    # Read until we see the final line to guarantee we captured the whole block
    start_time = time.time()
    response = b""
    while time.time() - start_time < 2.0:
        response += ser.read_all()
        if b"tx drain complete:" in response:
            break
        time.sleep(0.05)
        
    status = {}
    lines = response.decode('utf-8', errors='ignore').split('\n')
    for line in lines:
        line = line.strip()
        if ":" in line:
            key, val = line.split(":", 1)
            status[key.strip()] = val.strip()
    return status

def toggle_if_needed(ser, status, key, desired_state, toggle_cmd):
    current = status.get(key)
    if current is not None:
        is_on = current.startswith('ON')
        if (desired_state and not is_on) or (not desired_state and is_on):
            print(f"Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            ser.write(toggle_cmd.encode('utf-8'))
            time.sleep(0.1)
            ser.read_all() # clear buffer

def print_progress(bytes_sent, total_size, start_time):
    elapsed = time.time() - start_time
    speed_kbps = ((bytes_sent * 8) / elapsed) / 1000 if elapsed > 0 else 0
    
    if total_size:
        percent = (bytes_sent / total_size) * 100
        bar_len = 30
        filled = int(bar_len * bytes_sent / total_size)
        bar = '=' * filled + '-' * (bar_len - filled)
        sys.stdout.write(f"\r[{bar}] {percent:.1f}% | {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps    ")
    else:
        sys.stdout.write(f"\rStreaming... | {bytes_sent/1024:.1f} KB sent | {speed_kbps:.1f} kbps    ")
    sys.stdout.flush()

def print_drain_progress(current, total):
    percent = ((total - current) / total) * 100 if total > 0 else 100.0
    bar_len = 30
    filled = int(bar_len * (total - current) / total) if total > 0 else bar_len
    bar = '=' * filled + '-' * (bar_len - filled)
    sys.stdout.write(f"\rDraining TX FIFO: [{bar}] {percent:.1f}% | {current}/{total} frames remaining    ")
    sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description="Upload binary frames to BPSK Modulator")
    parser.add_argument("port", help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=12000000, help="Baud rate (default 12000000 to uncap OS driver)")
    parser.add_argument("--rate", type=int, help="Set symbol rate (Hz)")
    parser.add_argument("--crate", type=int, choices=[0,1,2,3,4], help="Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)")
    parser.add_argument("-I", "--interleave", type=int, choices=[1,2,4,8,16], help="Set RS interleaving depth")
    parser.add_argument("--rs", type=int, choices=[0,1], help="0=Disable, 1=Enable Reed-Solomon")
    parser.add_argument("--conv", type=int, choices=[0,1], help="0=Disable, 1=Enable Convolutional")
    parser.add_argument("--rand", type=int, choices=[0,1], help="0=Disable, 1=Enable Randomizer")
    parser.add_argument("--dual", type=int, choices=[0,1], help="0=Disable, 1=Enable Dual Basis")
    parser.add_argument("--prepacked", action='store_true', help="Indicate frames already contain 4-byte ASM")
    parser.add_argument("file", help="Binary file to upload (use '-' for standard input)")
    
    args = parser.parse_args()

    if args.file != "-" and not os.path.exists(args.file):
        print(f"Error: File {args.file} not found.")
        return

    print(f"Connecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1, write_timeout=2.0)
    try:
        ser.set_buffer_size(rx_size=262144, tx_size=262144) # Uncap Windows usbser.sys internal buffers
    except Exception:
        pass # Ignore on non-Windows systems
    time.sleep(0.5)
    ser.read_all() # Flush startup text

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

    if any(a is not None for a in [args.rate, args.crate, args.interleave, args.rs, args.conv, args.rand, args.dual]):
        print("\nApplying desired FEC settings...")
        
        if args.rate is not None:
            print(f"Setting symbol rate to {args.rate} Hz...")
            ser.write(f"r{args.rate}\n".encode('utf-8'))
            time.sleep(0.1)
            ser.read_all()
            
        if args.crate is not None:
            print(f"Setting convolutional rate to {['1/2','2/3','3/4','5/6','7/8'][args.crate]}...")
            ser.write(f"k{args.crate}\n".encode('utf-8'))
            time.sleep(0.1)
            ser.read_all()
            
        if args.interleave is not None:
            print(f"Setting RS interleaving depth to {args.interleave}...")
            ser.write(f"l{args.interleave}\n".encode('utf-8'))
            time.sleep(0.1)
            ser.read_all()
            
        if args.rs is not None: toggle_if_needed(ser, status, "RS (255,223)", args.rs == 1, 'y')
        if args.conv is not None: toggle_if_needed(ser, status, "Convolutional", args.conv == 1, 'c')
        if args.rand is not None: toggle_if_needed(ser, status, "Randomizer", args.rand == 1, 'n')
        if args.dual is not None: toggle_if_needed(ser, status, "Dual Basis", args.dual == 1, 'd')

        # Re-fetch status after toggles are applied to get the new Expected Payload
        status = get_status(ser)

    print("\n--- Final Modulator Status ---")
    for k in ["RS (255,223)", "Convolutional", "Randomizer", "Dual Basis", "Expected Payload"]:
        print(f"  {k}: {status.get(k, 'Unknown')}")
    print("------------------------------\n")

    # Determine chunk size from final status
    chunk_size = 1020
    if "Expected Payload" in status:
        val = status["Expected Payload"]
        import re
        m = re.search(r'(\d+)\s+bytes\s+\((\d+)', val)
        if m:
            chunk_size = int(m.group(2)) if args.prepacked else int(m.group(1))
    else:
        # Fallback if expected payload isn't parsed
        chunk_size = 892 if status.get("RS (255,223)") == "ON" else (1024 if args.prepacked else 1020)

    # Ensure modulation is started
    print("Starting modulation...")
    ser.write(b's')
    time.sleep(0.1)
    ser.read_all()

    # Enter binary upload mode
    ser.write(b'u')
    
    # Wait securely for the 'B' ready signal
    ready = False
    start_wait = time.time()
    response = b""
    while time.time() - start_wait < 2.0:
        response += ser.read_all()
        if b'B' in response:
            ready = True
            break
        time.sleep(0.05)
        
    if not ready:
        print("Warning: Did not receive 'B' ready signal. Modulator might not be ready.")
    
    # Send mode byte (0 for append ASM, 1 for prepacked)
    mode_byte = b'\x01' if args.prepacked else b'\x00'
    ser.write(mode_byte)
    
    print(f"Starting upload from {'stdin' if args.file == '-' else args.file} (Chunk size: {chunk_size} bytes)")
    
    # Read and upload file in chunks
    f = sys.stdin.buffer if args.file == "-" else open(args.file, "rb")
    total_size = os.path.getsize(args.file) if args.file != "-" else None
    bytes_sent = 0
    start_time = time.time()
    last_ui_update = 0
    
    window_size = 200 # Push the sliding window right to the edge of the MCU's 255-chunk queue limit
    chunk_ready_to_read = True
    
    file_buffer = bytearray()
    eof_reached = False

    in_flight = 0
    in_flight_lock = threading.Lock()
    upload_error = False
    upload_running = True

    def ack_reader():
        nonlocal in_flight, upload_error
        ser.timeout = 0.1 # Short timeout so thread can exit quickly on completion
        while upload_running or in_flight > 0:
            try:
                acks = ser.read(max(1, ser.in_waiting))
                if acks:
                    with in_flight_lock:
                        in_flight -= acks.count(b'K')
                    if b'E' in acks:
                        upload_error = True
                        break
            except Exception:
                upload_error = True
                break

    ack_thread = threading.Thread(target=ack_reader)
    ack_thread.daemon = True
    ack_thread.start()

    try:
        while chunk_ready_to_read:
            if upload_error:
                print("\n\nError: MCU rejected the chunk size or connection dropped. Disconnecting.")
                sys.exit(1)

            with in_flight_lock:
                if in_flight >= window_size:
                    time.sleep(0.001)
                    continue

            # 2. Batch read/serialize to blast to the OS in one massive USB Bulk Transfer
            payload_batch = bytearray()
            chunks_added = 0
            
            while chunk_ready_to_read and len(payload_batch) < 65536:
                with in_flight_lock:
                    if in_flight + chunks_added >= window_size:
                        break

                # Slurp 1MB at a time to completely eliminate OS disk I/O / Pipe overhead
                if len(file_buffer) < chunk_size and not eof_reached:
                    new_data = f.read(1048576) 
                    if not new_data:
                        eof_reached = True
                    else:
                        file_buffer.extend(new_data)
                        
                if len(file_buffer) == 0:
                    chunk_ready_to_read = False
                    break
                    
                take_len = min(chunk_size, len(file_buffer))
                chunk = file_buffer[:take_len]
                del file_buffer[:take_len] # Instant memmove in C-backend
                
                payload_batch.extend(len(chunk).to_bytes(2, byteorder='little'))
                payload_batch.extend(chunk)
                chunks_added += 1
                bytes_sent += len(chunk)
                
            if chunks_added > 0:
                try:
                    ser.write(payload_batch)
                except serial.SerialTimeoutException:
                    print("\n\nError: USB Write Timeout. The MCU stopped reading data.")
                    sys.exit(1)
                
                with in_flight_lock:
                    in_flight += chunks_added
                
                now = time.time()
                if now - last_ui_update > 0.2: # Update console at 5Hz to prevent terminal lag
                    print_progress(bytes_sent, total_size, start_time)
                    last_ui_update = now
                
        # Wait for the remaining ACKs to drain before proceeding
        while True:
            with in_flight_lock:
                if in_flight <= 0:
                    break
            if upload_error:
                print("\n\nError: MCU rejected the chunk size or connection dropped. Disconnecting.")
                sys.exit(1)
            time.sleep(0.01)

    finally:
        upload_running = False
        ack_thread.join(timeout=2.0)
        ser.timeout = 1
        if args.file != "-":
            f.close()

    print_progress(bytes_sent, total_size, start_time)
    print("\n") # Add newline so the 100% progress bar isn't overwritten!
    # End of stream packet (length = 0)
    ser.write(b'\x00\x00')
    print("\n\nUpload buffered successfully! Waiting for radio transmission to drain...")
    
    max_queue = None
    while True:
        ser.write(b'q')
        time.sleep(0.2)
        text = ser.read_all().decode('utf-8', errors='ignore')
        
        if 'tx drain complete: YES' in text or text.strip() == 'D' or text.endswith('\nD') or text.endswith('\rD') or text.startswith('DModulator'):
            if max_queue is not None:
                print_drain_progress(0, max_queue)
            print("\n\nDrain complete. Transmission finished!")
            
            total_time = time.time() - start_time
            avg_kbps = ((bytes_sent * 8) / total_time) / 1000 if total_time > 0 else 0
            print(f"Total transmission time: {total_time:.2f} seconds")
            print(f"Average data rate: {avg_kbps:.2f} kbps")
            break
            
        for line in text.split('\n'):
            if "queue depth:" in line:
                try:
                    parts = line.split("queue depth:")[1].strip().split('/')
                    current_q = int(parts[0])
                    if max_queue is None or current_q > max_queue:
                        max_queue = current_q
                    print_drain_progress(current_q, max_queue)
                except Exception:
                    pass

    ser.close()

if __name__ == "__main__":
    main()
