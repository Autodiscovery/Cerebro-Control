import socket
import struct
import time
from pymodbus.client import ModbusSerialClient as ModbusClient

# Initialize Modbus serial communication for both hands
def initialize_modbus(port, baudrate=115200):
    client = ModbusClient(port=port, baudrate=baudrate, stopbits=1, bytesize=8, parity='N', timeout=1)
    if client.connect():
        print(f"Connected to Modbus on {port}")
    else:
        print(f"Failed to connect to {port}")
    return client

# Function to check if a hand is responsive
def check_hand_connection(client):
    try:
        response = client.read_holding_registers(0x03E8, count=1, slave=1)  # Read HAND_ID (both are default 1)
        return response and not response.isError()
    except Exception:
        return False

# Function to send data to the hand via Modbus
def send_hand_data(client, values):
    try:
        response = client.write_registers(0x05CE, values, slave=1)  # Sending 6 joint values
        if response.isError():
            print("Error sending data")
        else:
            print("Successfully sent data:", values)
    except Exception as e:
        print("Error writing to hand:", e)

# Function to handle incoming data from the socket
def receive_and_send_hand_data():
    right_hand_client = initialize_modbus('/dev/ttyUSB0')
    left_hand_client = initialize_modbus('/dev/ttyUSB1')

    right_hand_connected = check_hand_connection(right_hand_client)
    left_hand_connected = check_hand_connection(left_hand_client)

    if not right_hand_connected and not left_hand_connected:
        print("No hands detected. Exiting.")
        return

    # Socket setup
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 5050))  # Listen for hand pose data
    server.listen(1)
    print("Listening for pose data on port 5050...")

    try:
        while True:
            conn, addr = server.accept()
            print(f"Connected to {addr}")

            while True:
                data = conn.recv(1024)
                if not data:
                    break

                try:
                    # Unpack the data (assuming 12 integers were sent)
                    pose_values = struct.unpack('>' + 'i' * 12, data)
                    left_angles, right_angles = pose_values[:6], pose_values[6:]
                    print(f"Received Left: {left_angles}, Right: {right_angles}")

                    if left_hand_connected:
                        send_hand_data(left_hand_client, left_angles)
                    else:
                        print("Left hand not connected, skipping.")

                    if right_hand_connected:
                        send_hand_data(right_hand_client, right_angles)
                    else:
                        print("Right hand not connected, skipping.")

                except Exception as e:
                    print("Data parsing error:", e)

            conn.close()
            print("Client disconnected.")

    except KeyboardInterrupt:
        print("Shutting down server...")

    finally:
        right_hand_client.close()
        left_hand_client.close()
        server.close()

# Run the server
if __name__ == "__main__":
    receive_and_send_hand_data()
