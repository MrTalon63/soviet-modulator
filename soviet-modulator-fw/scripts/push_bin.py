import argparse
import serial
import time
import sys
import os
import ctypes


def get_status(ser):
    # Flush input buffer to remove old telemetry
    ser.read_all()
    # Ask the modulator for its current configuration
    ser.write(b"q")

    # Read until we see the final line to guarantee we captured the whole block
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
            print(f"Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            ser.write(toggle_cmd.encode("utf-8"))
            time.sleep(0.1)
            ser.read_all()  # clear buffer


def print_progress(bytes_sent, total_size, start_time, mcu_temp=None):
    elapsed = time.time() - start_time
    speed_kbps = ((bytes_sent * 8) / elapsed) / 1000 if elapsed > 0 else 0

    temp_str = f" | Temp: {mcu_temp}C" if mcu_temp is not None else ""

    if total_size:
        percent = (bytes_sent / total_size) * 100
        bar_len = 30
        filled = int(bar_len * bytes_sent / total_size)
        bar = "=" * filled + "-" * (bar_len - filled)
        sys.stdout.write(
            f"\r[{bar}] {percent:.1f}% | {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps{temp_str}    "
        )
    else:
        sys.stdout.write(
            f"\rStreaming... | {bytes_sent/1024:.1f} KB sent | {speed_kbps:.1f} kbps{temp_str}    "
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
        "--spi",
        action="store_true",
        help="Use CH347 SPI for high-speed payload transfer instead of Serial",
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
    status, response = get_status(ser)
    retry_count = 0
    while not status and retry_count < 10:
        if b"[MCU]" in response:
            print(
                "\nMCU is stuck in Binary Mode. Waiting 16 seconds for watchdog to timeout..."
            )
            time.sleep(16.0)
        else:
            time.sleep(0.5)
        status, response = get_status(ser)
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
        status, _ = get_status(ser)

    print("\n--- Final Modulator Status ---")
    for k in [
        "Symbol Rate",
        "RS (255,223)",
        "FECF (CRC-16)",
        "Convolutional",
        "Randomizer",
        "Dual Basis",
        "Expected Payload",
    ]:
        print(f"  {k}: {status.get(k, 'Unknown')}")

    # --- Calculate Usable Bandwidth ---
    try:
        sym_rate_str = status.get("Symbol Rate", "1000000 Hz")
        symbol_rate = int(sym_rate_str.split()[0])

        conv_status = status.get("Convolutional", "OFF")
        symbols_per_frame = 8192
        if "ON" in conv_status:
            if "1/2" in conv_status:
                symbols_per_frame = 16384
            elif "2/3" in conv_status:
                symbols_per_frame = 12288
            elif "3/4" in conv_status:
                symbols_per_frame = 10922
            elif "5/6" in conv_status:
                symbols_per_frame = 9830
            elif "7/8" in conv_status:
                symbols_per_frame = 9362

        frames_per_sec = symbol_rate / symbols_per_frame

        expected_payload_str = status.get("Expected Payload", "882 bytes")
        payload_bytes = int(expected_payload_str.split()[0])

        usable_bytes_per_sec = frames_per_sec * payload_bytes
        usable_kbps = (usable_bytes_per_sec * 8) / 1000

        print(
            f"  Usable Bandwidth: {usable_kbps:.2f} kbps ({usable_bytes_per_sec/1024:.2f} KB/s)"
        )
    except Exception:
        pass
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

    ch347 = None
    if args.spi:
        print("Connecting to CH347 via High-Speed USB and setting up SPI Master...")

        ch347 = None
        try:
            dll = None
            base_dir = os.path.dirname(os.path.abspath(__file__))
            cwd_dir = os.getcwd()
            search_paths = [
                "CH347DLLA64.DLL",
                "CH347DLL.DLL",
                os.path.join(base_dir, "CH347DLLA64.DLL"),
                os.path.join(base_dir, "CH347DLL.DLL"),
                os.path.join(cwd_dir, "CH347DLLA64.DLL"),
                os.path.join(cwd_dir, "CH347DLL.DLL"),
            ]
            import importlib.util
            import glob

            for mod_name in ["ch347", "ch347api", "CH347", "CH347API"]:
                try:
                    spec = importlib.util.find_spec(mod_name)
                    if spec and spec.submodule_search_locations:
                        for loc in spec.submodule_search_locations:
                            search_paths.extend(glob.glob(os.path.join(loc, "*.dll")))
                except Exception:
                    pass
            for path in search_paths:
                try:
                    dll = ctypes.windll.LoadLibrary(path)
                    if not hasattr(dll, "CH347OpenDevice"):
                        dll = ctypes.cdll.LoadLibrary(path)
                    if (
                        dll
                        and hasattr(dll, "CH347OpenDevice")
                        and hasattr(dll, "CH347SPI_Init")
                    ):
                        print(f"[SPI] Loaded native DLL from: {path}")
                        break
                    dll = None
                except Exception:
                    continue

            if dll:

                class SPI_CONFIG(ctypes.Structure):
                    _pack_ = 1
                    _fields_ = [
                        ("iMode", ctypes.c_byte),
                        ("iClock", ctypes.c_byte),
                        ("iByteOrder", ctypes.c_byte),
                        ("iSpiWriteReadInterval", ctypes.c_ushort),
                        ("iSpiOutDefaultData", ctypes.c_byte),
                        ("iChipSelect", ctypes.c_ulong),
                        ("CS1Polarity", ctypes.c_byte),
                        ("CS2Polarity", ctypes.c_byte),
                        ("iIsAutoDeativeCS", ctypes.c_ushort),
                        ("iActiveDelay", ctypes.c_ushort),
                        ("iDelayDeactive", ctypes.c_ulong),
                    ]

                open_dev = getattr(dll, "CH347OpenDevice", None)
                init_spi = getattr(dll, "CH347SPI_Init", None)
                write_spi = getattr(dll, "CH347SPI_Write", None)
                close_dev = getattr(dll, "CH347CloseDevice", None)

                if open_dev and init_spi and write_spi:
                    open_dev.argtypes = [ctypes.c_ulong]
                    open_dev.restype = ctypes.c_void_p
                    init_spi.argtypes = [ctypes.c_ulong, ctypes.POINTER(SPI_CONFIG)]
                    init_spi.restype = ctypes.c_bool
                    write_spi.argtypes = [
                        ctypes.c_ulong,
                        ctypes.c_ulong,
                        ctypes.c_ulong,
                        ctypes.c_ulong,
                        ctypes.c_void_p,
                    ]
                    write_spi.restype = ctypes.c_bool
                    if close_dev:
                        close_dev.argtypes = [ctypes.c_ulong]
                        close_dev.restype = ctypes.c_bool

                    h = None
                    dev_idx = 0
                    for i in range(8):
                        temp_h = open_dev(i)
                        if temp_h and temp_h not in [
                            -1,
                            0,
                            0xFFFFFFFF,
                            0xFFFFFFFFFFFFFFFF,
                        ]:
                            h = temp_h
                            dev_idx = i
                            break

                    if not h:
                        print(
                            "[SPI] Native Hook OpenDevice returned invalid handle. (Device occupied, or using WinUSB driver!)"
                        )

                    cfg = SPI_CONFIG()
                    cfg.iMode = 0
                    cfg.iClock = 1
                    cfg.iByteOrder = 1
                    cfg.iSpiWriteReadInterval = 0
                    cfg.iSpiOutDefaultData = 0xFF
                    cfg.iChipSelect = 0x80
                    cfg.CS1Polarity = 0
                    cfg.CS2Polarity = 0
                    cfg.iIsAutoDeativeCS = 1
                    cfg.iActiveDelay = 0
                    cfg.iDelayDeactive = 0

                    success_cs = None
                    for cs in [0x80, 0x00, 128]:
                        cfg.iChipSelect = cs
                        if init_spi(dev_idx, ctypes.byref(cfg)):
                            success_cs = cs
                            break

                    if success_cs is not None:

                        class CH347Native:
                            def spi_tx(self, data):
                                b_data = bytes(data)
                                if not write_spi(
                                    dev_idx, success_cs, len(b_data), 4096, b_data
                                ):
                                    time.sleep(0.002)
                                    write_spi(
                                        dev_idx, success_cs, len(b_data), 4096, b_data
                                    )

                        ch347 = CH347Native()
                        print("[SPI] Native CH347 DLL Hook Initialized Successfully.")
                    else:
                        print("[SPI] Native Hook SPI_Init returned False.")
                        if close_dev:
                            close_dev(dev_idx)
        except Exception as e:
            print(f"[SPI] Native Hook Warning: {e}")

        init_errors = []
        if ch347 is None:
            try:
                import ch347api

                if hasattr(ch347api, "Ch347Api"):
                    ch347 = ch347api.Ch347Api()
                elif hasattr(ch347api, "CH347API"):
                    ch347 = ch347api.CH347API()
                elif hasattr(ch347api, "CH347"):
                    ch347 = ch347api.CH347()
                elif hasattr(ch347api, "CH347SPI_Init"):
                    ch347 = ch347api
                elif hasattr(ch347api, "SPIDevice"):
                    try:
                        ch347 = ch347api.SPIDevice()
                    except Exception:
                        ch347 = ch347api.SPIDevice(0)
                else:
                    init_errors.append(f"ch347api attrs: {dir(ch347api)}")
            except Exception as e:
                init_errors.append(f"ch347api import/init: {e}")
                if "usb" in str(e).lower() or "backend" in str(e).lower():
                    init_errors.append(
                        "   -> Hint: For WinUSB, ensure you ran 'pip install pyusb libusb'"
                    )
                if "expected bytes" in str(e).lower() or "nonetype" in str(e).lower():
                    init_errors.append(
                        "   -> Hint: PyUSB bug! Windows WinUSB is blocking descriptors."
                    )
                    init_errors.append(
                        "   -> Fix: Open Device Manager, uninstall the CH347 WinUSB driver, and restore the WCH driver."
                    )

        if ch347 is None:
            try:
                import ch347 as ch347_mod

                if hasattr(ch347_mod, "CH347API"):
                    ch347 = ch347_mod.CH347API()
                elif hasattr(ch347_mod, "Ch347Api"):
                    ch347 = ch347_mod.Ch347Api()
                elif hasattr(ch347_mod, "CH347"):
                    ch347 = ch347_mod.CH347()
                elif hasattr(ch347_mod, "CH347SPI_Init"):
                    ch347 = ch347_mod
                else:
                    init_errors.append(f"ch347 attrs: {dir(ch347_mod)}")
            except Exception as e:
                init_errors.append(f"ch347 import/init: {e}")
                if "usb" in str(e).lower() or "backend" in str(e).lower():
                    init_errors.append(
                        "   -> Hint: For WinUSB, ensure you ran 'pip install pyusb libusb'"
                    )
                if "expected bytes" in str(e).lower() or "nonetype" in str(e).lower():
                    init_errors.append(
                        "   -> Hint: PyUSB bug! Windows WinUSB is blocking descriptors."
                    )
                    init_errors.append(
                        "   -> Fix: Open Device Manager, uninstall the CH347 WinUSB driver, and restore the WCH driver."
                    )

        if ch347 is None:
            print(
                "\nError: Failed to initialize CH347 device. Details:\n"
                + "\n".join(init_errors)
            )
            sys.exit(1)

        init_methods = ["spi_init", "init_SPI", "init_spi", "SPI_Init", "CH347SPI_Init"]
        tx_methods = [
            "spi_tx",
            "spi_write",
            "write_spi",
            "write",
            "SPI_Write",
            "CH347SPI_Write",
        ]

        init_func = next(
            (getattr(ch347, m) for m in init_methods if hasattr(ch347, m)), None
        )
        raw_tx_func = next(
            (getattr(ch347, m) for m in tx_methods if hasattr(ch347, m)), None
        )

        if not raw_tx_func:
            print(
                f"\nError: Could not find SPI TX method on CH347 class. Available: {dir(ch347)}"
            )
            sys.exit(1)
        if not init_func:
            init_func = lambda *args, **kwargs: True

        if hasattr(ch347, "CH347OpenDevice"):
            try:
                res = ch347.CH347OpenDevice(0)
                if res == -1 or res is False or res == 0:
                    print(f"Warning: CH347OpenDevice returned {res}")
            except Exception as e:
                print(f"CH347OpenDevice exception: {e}")

        def _try_init_cfg(cs_val):
            if hasattr(ch347, "SPI_CONFIG"):
                cfg = getattr(ch347, "SPI_CONFIG")()
                cfg.iMode = 0
                cfg.iClock = 1  # 30MHz
                cfg.iByteOrder = 1
                cfg.iSpiWriteReadInterval = 0
                cfg.iSpiOutDefaultData = 0xFF
                cfg.iChipSelect = cs_val
                cfg.CS1Polarity = 0
                cfg.CS2Polarity = 0
                cfg.iIsAutoDeativeCS = 1
                cfg.iActiveDelay = 0
                cfg.iDelayDeactive = 0
                return init_func(0, cfg)
            raise TypeError("No SPI_CONFIG")

        init_signatures = [
            lambda: init_func(clock_speed_hz=30000000, mode=0),
            lambda: _try_init_cfg(128),
            lambda: _try_init_cfg(0x80),
            lambda: _try_init_cfg(0x00),
            lambda: init_func(0, 0, 1, 1, 0, 0xFF, 128, 0, 0, 1, 0, 0),
            lambda: init_func(0, 0, 1, 1, 0, 0xFF, 0x80, 0, 0, 1, 0, 0),
            lambda: init_func(0, 0, 1, 1, 0, 0xFF, 0x00, 0, 0, 1, 0, 0),
            lambda: init_func(),
        ]

        res_init = None
        init_success = False
        init_errors_list = []
        for i, sig in enumerate(init_signatures):
            try:
                res_init = sig()
                if res_init is False or res_init == 0 or res_init == -1:
                    init_errors_list.append(f"#{i} Returned {res_init}")
                    continue
                init_success = True
                break
            except Exception as e:
                init_errors_list.append(f"#{i} {type(e).__name__}: {e}")
                continue

        if not init_success:
            print(
                f"[SPI] Warning: SPI_Init failed or rejected by wrapper! Errors: {' | '.join(init_errors_list)}"
            )

        def _spi_tx(data):
            b_data = bytes(data)
            c_buf = (ctypes.c_byte * len(b_data)).from_buffer_copy(b_data)

            sigs = [
                lambda: raw_tx_func(b_data),
                lambda: raw_tx_func(0, 128, len(b_data), b_data),
                lambda: raw_tx_func(0, 0x80, len(b_data), b_data),
                lambda: raw_tx_func(0, 0x00, len(b_data), b_data),
                lambda: raw_tx_func(0, 128, 4096, b_data),
                lambda: raw_tx_func(0, 0x80, 4096, b_data),
                lambda: raw_tx_func(0, 0x00, 4096, b_data),
                lambda: raw_tx_func(0, 128, len(b_data), len(b_data), b_data),
                lambda: raw_tx_func(0, 0x80, len(b_data), len(b_data), b_data),
                lambda: raw_tx_func(0, 0x00, len(b_data), len(b_data), b_data),
                lambda: raw_tx_func(0, 128, len(b_data), 4096, b_data),
                lambda: raw_tx_func(0, 0x80, len(b_data), 4096, b_data),
                lambda: raw_tx_func(0, 0x00, len(b_data), 4096, b_data),
                lambda: raw_tx_func(0, 128, len(b_data), c_buf),
                lambda: raw_tx_func(0, 0x80, len(b_data), c_buf),
                lambda: raw_tx_func(0, 0x00, len(b_data), c_buf),
                lambda: raw_tx_func(0, 128, 4096, c_buf),
                lambda: raw_tx_func(0, 0x80, 4096, c_buf),
                lambda: raw_tx_func(0, 0x00, 4096, c_buf),
                lambda: raw_tx_func(0, 128, len(b_data), len(b_data), c_buf),
                lambda: raw_tx_func(0, 0x80, len(b_data), len(b_data), c_buf),
                lambda: raw_tx_func(0, 0x00, len(b_data), len(b_data), c_buf),
                lambda: raw_tx_func(0, 128, len(b_data), 4096, c_buf),
                lambda: raw_tx_func(0, 0x80, len(b_data), 4096, c_buf),
                lambda: raw_tx_func(0, 0x00, len(b_data), 4096, c_buf),
                lambda: raw_tx_func(0x00, len(b_data), b_data),
                lambda: raw_tx_func(0x80, len(b_data), b_data),
                lambda: raw_tx_func(0x00, len(b_data), c_buf),
                lambda: raw_tx_func(0x80, len(b_data), c_buf),
            ]

            if ch347._tx_sig_index is None:
                errors = []
                for i, sig in enumerate(sigs):
                    try:
                        res_tx = sig()
                        ch347._tx_sig_index = i
                        if res_tx is False or res_tx == 0 or res_tx == -1:
                            print(
                                f"\n[SPI] TX Warning: Wrapper returned {res_tx}. If using WinUSB, the WCH DLL will fail!"
                            )
                        return
                    except Exception as e:
                        errors.append(f"#{i} {type(e).__name__}: {e}")
                err_str = " | ".join(errors)
                print(f"\n[SPI] TX Error: No compatible signature! {err_str}")
                sys.exit(1)
            else:
                res_tx = sigs[ch347._tx_sig_index]()
                if res_tx is False or res_tx == 0 or res_tx == -1:
                    time.sleep(0.002)  # USB Queue full, backoff and retry
                    res_tx = sigs[ch347._tx_sig_index]()
                    if res_tx is False or res_tx == 0 or res_tx == -1:
                        print(
                            f"\r[SPI] TX Warning: Wrapper returned {res_tx} (Hardware Error/WinUSB conflict?)    ",
                            end="",
                        )

        ch347.spi_tx = _spi_tx

    print(f"Starting upload from {'stdin' if args.file == '-' else args.file}...")

    # Read and upload file in chunks
    f = sys.stdin.buffer if args.file == "-" else open(args.file, "rb")
    total_size = os.path.getsize(args.file) if args.file != "-" else None
    bytes_sent = 0
    start_time = time.time()
    last_ui_update = 0
    mcu_fifo_level = 0
    telemetry_buffer = ""
    mcu_temp = None

    seq = 0
    sp_payload_size = args.size  # Keep Space Packets comfortably inside FIFO capacity

    try:
        while True:
            chunk = f.read(sp_payload_size)
            if not chunk:
                break

            packet = generate_space_packet(args.apid, seq, chunk)
            seq = (seq + 1) & 0x3FFF

            if args.spi:
                # Software flow control: Read CDC telemetry to prevent SPI buffer overrun
                while True:
                    if ser.in_waiting:
                        telemetry_buffer += ser.read_all().decode(
                            "utf-8", errors="ignore"
                        )
                        if "\n" in telemetry_buffer:
                            lines = telemetry_buffer.split("\n")
                            telemetry_buffer = lines[-1]
                            for line in lines[:-1]:
                                if "[MCU] FIFO:" in line:
                                    try:
                                        mcu_fifo_level = int(
                                            line.split("FIFO: ")[1].split("/")[0]
                                        )
                                    except ValueError:
                                        pass
                                if "Temp: " in line:
                                    mcu_temp = (
                                        line.split("Temp: ")[1].replace("C", "").strip()
                                    )
                    if mcu_fifo_level < 100000:
                        break
                    time.sleep(0.01)

                ch347.spi_tx(packet)
                mcu_fifo_level += len(
                    packet
                )  # Predict level to prevent instantly overfilling
            else:
                try:
                    ser.write(packet)
                except serial.SerialTimeoutException:
                    print(
                        "\n\nError: USB Write Timeout. The MCU stopped receiving data."
                    )
                    sys.exit(1)

                if ser.in_waiting:
                    telemetry_buffer += ser.read_all().decode("utf-8", errors="ignore")
                    if "\n" in telemetry_buffer:
                        lines = telemetry_buffer.split("\n")
                        telemetry_buffer = lines[-1]
                        for line in lines[:-1]:
                            if "Temp: " in line:
                                mcu_temp = (
                                    line.split("Temp: ")[1].replace("C", "").strip()
                                )

            bytes_sent += len(chunk)

            now = time.time()
            if now - last_ui_update > 0.2:  # Update console at 5Hz
                print_progress(bytes_sent, total_size, start_time, mcu_temp)
                last_ui_update = now

        print_progress(bytes_sent, total_size, start_time, mcu_temp)

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
            status, _ = get_status(ser)
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
