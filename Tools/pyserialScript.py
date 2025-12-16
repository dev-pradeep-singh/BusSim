import sys
import threading
import time

import serial

PORT = "/dev/tty.usbserial-1130"  # replace with your COM port
BAUD = 115200  # match USART3 CLI baud
LINE_TERMINATOR = "\r\n"


def reader(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Continuously read from the serial port and print bytes received."""
    print("Reader thread started.")
    while not stop_event.is_set():
        try:
            data = ser.readlines()
        except serial.SerialException as exc:
            print(f"[RX] Serial error: {exc}")
            stop_event.set()
            return

        if data:
            # Use repr so non-printable bytes are visible.
            for line in data:
                try:
                    decoded_line = line.decode().rstrip()
                except UnicodeDecodeError:
                    decoded_line = line.hex()
                print(f"      {decoded_line}")
        else:
            # Avoid busy loop when nothing is waiting.
            time.sleep(0.05)


def main() -> int:
    try:
        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=0.1,
            write_timeout=0.5,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        print(f"Could not open {PORT}: {exc}")
        return 1

    # Clear any stale input that might have accumulated before we started.
    ser.reset_input_buffer()

    stop_event = threading.Event()
    reader_thread = threading.Thread(target=reader, args=(ser, stop_event), daemon=True)
    reader_thread.start()

    print(f"Connected to {PORT} @ {BAUD} baud. Type commands to send, or 'exit' to quit.")

    try:
        while True:
            try:
                cmd = input("> ")
            except EOFError:
                cmd = "exit"

            if cmd.lower() in {"exit", "quit"}:
                break

            if not cmd.strip():
                continue

            payload = (cmd + LINE_TERMINATOR).encode()
            try:
                ser.write(payload)
                ser.flush()
                #print(f"[TX] {payload!r}")
            except serial.SerialException as exc:
                print(f"Write failed: {exc}")
                break
    finally:
        stop_event.set()
        reader_thread.join(timeout=1)
        ser.close()

    print("Serial session ended.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
