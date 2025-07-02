import serial
import socket

# ----- CONFIGURATION -----
SERIAL_PORT = '/dev/ttyUSB0'  # Change as per your Jetson serial port
BAUD_RATE = 115200
UDP_IP = '192.168.1.189'  # Receiver IP (your PC running robot_data_tranceiver.py)
UDP_PORT = 12345
# -------------------------

def main():
    # Setup serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Setup UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"[INFO] Listening on serial {SERIAL_PORT} at {BAUD_RATE} baud...")
    print(f"[INFO] Forwarding to UDP {UDP_IP}:{UDP_PORT}")

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"[SERIAL] {line}")
                sock.sendto(line.encode('utf-8'), (UDP_IP, UDP_PORT))
    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted by user.")
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()
