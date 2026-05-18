import argparse
import serial
import time
import sys
import os
import struct
import threading
import asyncio
import queue
import ctypes
from collections import defaultdict

# Max sizes prevent RAM exhaustion and natively exert TCP backpressure when full
# These defaults are dynamically overridden in main() based on the --qsize argument
HIGH_PRIORITY_QUEUE = queue.Queue()
NORMAL_PRIORITY_QUEUE = queue.Queue()
mcu_fifo_level = 0

seq_counters = defaultdict(int)


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
    global mcu_fifo_level
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
            ser = serial.Serial(port, baud, timeout=10, write_timeout=30.0)

            # Apply configuration parameters
            status, response = get_status(ser)
            retry_count = 0
            while not status and retry_count < 10:
                if b"[MCU]" in response:
                    print(
                        "[Serial] MCU is stuck in Binary Mode. Waiting 16 seconds for watchdog to timeout..."
                    )
                    time.sleep(16.0)
                else:
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
                status, _ = get_status(ser)

            print("\n[Serial] --- Final Modulator Status ---")
            for k in [
                "Symbol Rate",
                "RS (255,223)",
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
                    f"[Serial]   Usable Bandwidth: {usable_kbps:.2f} kbps ({usable_bytes_per_sec/1024:.2f} KB/s)"
                )
            except Exception:
                pass
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

            ch347 = None
            if args.spi:
                print(
                    "[Serial] Connecting to CH347 via High-Speed USB and setting up SPI Master..."
                )

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
                                    search_paths.extend(
                                        glob.glob(os.path.join(loc, "*.dll"))
                                    )
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
                            init_spi.argtypes = [
                                ctypes.c_ulong,
                                ctypes.POINTER(SPI_CONFIG),
                            ]
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
                                            dev_idx,
                                            success_cs,
                                            len(b_data),
                                            4096,
                                            b_data,
                                        ):
                                            time.sleep(0.002)
                                            write_spi(
                                                dev_idx,
                                                success_cs,
                                                len(b_data),
                                                4096,
                                                b_data,
                                            )

                                ch347 = CH347Native()
                                print(
                                    "[SPI] Native CH347 DLL Hook Initialized Successfully."
                                )
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
                        if (
                            "expected bytes" in str(e).lower()
                            or "nonetype" in str(e).lower()
                        ):
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
                        if (
                            "expected bytes" in str(e).lower()
                            or "nonetype" in str(e).lower()
                        ):
                            init_errors.append(
                                "   -> Hint: PyUSB bug! Windows WinUSB is blocking descriptors."
                            )
                            init_errors.append(
                                "   -> Fix: Open Device Manager, uninstall the CH347 WinUSB driver, and restore the WCH driver."
                            )

                if ch347 is None:
                    raise ImportError(
                        "\nFailed to initialize CH347 device. Details:\n"
                        + "\n".join(init_errors)
                    )

                # Dynamically map the SPI initialization and write functions
                init_methods = [
                    "spi_init",
                    "init_SPI",
                    "init_spi",
                    "SPI_Init",
                    "CH347SPI_Init",
                ]
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
                    raise AttributeError(
                        f"Could not find SPI TX method on CH347 class. Available: {dir(ch347)}"
                    )
                if not init_func:
                    init_func = lambda *args, **kwargs: True

                if hasattr(ch347, "CH347OpenDevice"):
                    try:
                        res = ch347.CH347OpenDevice(0)
                        if res == -1 or res is False or res == 0:
                            print(f"[SPI] Warning: CH347OpenDevice returned {res}")
                    except Exception as e:
                        print(f"[SPI] CH347OpenDevice exception: {e}")

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
                        init_success = True
                        if res_init is False or res_init == 0 or res_init == -1:
                            init_errors_list.append(
                                f"#{i} Executed but returned {res_init}"
                            )
                        break
                    except Exception as e:
                        init_errors_list.append(f"#{i} {type(e).__name__}: {e}")
                        continue

                if not init_success:
                    print(
                        f"[SPI] Warning: SPI_Init failed or rejected by wrapper! Errors: {' | '.join(init_errors_list)}"
                    )
                elif res_init is False or res_init == 0 or res_init == -1:
                    print(
                        f"[SPI] Warning: SPI_Init executed but returned {res_init}. (WinUSB conflict?)"
                    )

                ch347._tx_sig_index = None

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
                                        f"\r[SPI] TX Warning: Wrapper returned {res_tx} (Hardware Error/WinUSB conflict?)    ",
                                        end="",
                                    )
                            except Exception as e:
                                errors.append(f"#{i} {type(e).__name__}: {e}")
                        err_str = " | ".join(errors)
                        raise RuntimeError(
                            f"TX Error: No compatible signature! {err_str}"
                        )
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

                        if ch347:
                            ch347.spi_tx(packet)
                            mcu_fifo_level += len(packet)
                        else:
                            ser.write(packet)
                        continue

                # We have a valid user packet
                seq = seq_counters[apid]
                seq_counters[apid] = (seq + 1) & 0x3FFF

                packet = generate_space_packet(apid, seq, payload)

                if ch347:
                    # Software flow control backpressure
                    while mcu_fifo_level > 100000:  # Pause if MCU is >75% full
                        time.sleep(0.01)
                    ch347.spi_tx(packet)
                    mcu_fifo_level += len(packet)
                else:
                    # Software flow control backpressure for Serial
                    while mcu_fifo_level > 100000:  # Pause if MCU is >75% full
                        time.sleep(0.01)

                    ser.write(packet)
                    mcu_fifo_level += len(packet)

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
        "--rs",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Reed-Solomon",
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
        default=50000,
        help="Max items per priority queue (default 50000, ~50MB at 1KB/item)",
    )
    parser.add_argument(
        "--spi",
        action="store_true",
        help="Use CH347 SPI for high-speed payload transfer instead of Serial",
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
