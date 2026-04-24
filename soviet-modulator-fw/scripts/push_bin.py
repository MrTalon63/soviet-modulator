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
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default 921600)")
    parser.add_argument("--rate", type=int, help="Set symbol rate (Hz)")
    parser.add_argument("--crate", type=int, choices=[0,1,2,3,4], help="Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)")
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

    if any(a is not None for a in [args.rate, args.crate, args.rs, args.conv, args.rand, args.dual]):
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
        if "892" in val: chunk_size = 892
        elif "1020" in val:
            chunk_size = 1024 if args.prepacked else 1020
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
    
    window_size = 16 # Conservative sliding window for rock-solid stability
    in_flight = 0
    chunk_ready_to_read = True

    try:
        while chunk_ready_to_read or in_flight > 0:
            # 1. Drain pending ACKs non-blockingly
            if ser.in_waiting > 0:
                err_check = ser.read(ser.in_waiting)
                in_flight -= err_check.count(b'K')
                if b'E' in err_check:
                    print("\n\nError: MCU rejected the chunk size. Disconnecting.")
                    sys.exit(1)

            # 2. Write one chunk at a time and pull ACKs instantly to prevent circular deadlocks!
            while in_flight < window_size and chunk_ready_to_read:
                chunk = f.read(chunk_size)
                if not chunk:
                    chunk_ready_to_read = False
                    break
                
                payload = len(chunk).to_bytes(2, byteorder='little') + chunk
                try:
                    ser.write(payload)
                except serial.SerialTimeoutException:
                    print("\n\nError: USB Write Timeout. The MCU stopped reading data.")
                    sys.exit(1)
                in_flight += 1
                bytes_sent += len(chunk)
                
                # Instantly clear MCU TX buffer
                if ser.in_waiting > 0:
                    err_check = ser.read(ser.in_waiting)
                    in_flight -= err_check.count(b'K')
                    if b'E' in err_check:
                        print("\n\nError: MCU rejected the chunk size. Disconnecting.")
                        sys.exit(1)
                        
                now = time.time()
                if now - last_ui_update > 0.2: # Update console at 5Hz to prevent terminal lag
                    print_progress(bytes_sent, total_size, start_time)
                    last_ui_update = now
                
            # 3. If window is full, OR if we are at EOF and waiting for final ACKs, block for an ACK
            if in_flight >= window_size or (not chunk_ready_to_read and in_flight > 0):
                acks = ser.read(max(1, ser.in_waiting))
                if not acks and ser.in_waiting == 0:
                    print("\n\nError: USB Read Timeout. The MCU stopped acknowledging data.")
                    sys.exit(1)
                in_flight -= acks.count(b'K')
                if b'E' in acks:
                    print("\n\nError: MCU rejected the chunk size. Disconnecting.")
                    sys.exit(1)
    finally:
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
