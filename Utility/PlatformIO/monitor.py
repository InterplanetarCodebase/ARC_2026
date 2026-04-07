import serial
import threading
import sys

PORT = "/dev/ttyUSB0"

def read_serial(ser):
    while True:
        try:
            data = ser.readline()
            print(data.decode('utf-8', errors='replace'), end='')
        except Exception as e:
            print(f"[READ ERROR]: {e}")
            break

def main():
    # Get baud rate from command line argument or prompt user
    if len(sys.argv) == 3:
        port = sys.argv[1]
        baud = int(sys.argv[2])
    elif len(sys.argv) == 2:
        port = sys.argv[1]
        baud = int(input("Enter baud rate: "))
    else:
        port = input(f"Enter port (default {PORT}): ").strip() or PORT
        baud = int(input("Enter baud rate (e.g. 115200): ").strip())

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[Connected] {port} @ {baud} baud")
        print("[Type your input and press Enter to send | Ctrl+C to quit]\n")

        t = threading.Thread(target=read_serial, args=(ser,), daemon=True)
        t.start()

        while True:
            user_input = input()
            ser.write((user_input + '\n').encode('utf-8'))

    except serial.SerialException as e:
        print(f"[ERROR]: {e}")
    except KeyboardInterrupt:
        print("\n[Disconnected]")
        ser.close()

if __name__ == "__main__":
    main()