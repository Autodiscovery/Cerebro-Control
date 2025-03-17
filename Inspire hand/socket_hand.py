import socket
from pymodbus.client import ModbusSerialClient as ModbusClient

# Initialize Modbus serial communication
def initialize_modbus(port='/dev/ttyUSB0', baudrate=115200):
    client = ModbusClient(port=port, baudrate=baudrate, stopbits=1, bytesize=8, parity='N', timeout=1)
    client.connect()
    return client

# Function to write to holding registers (MODBUS function code 0x10)
def write_registers(client, address, values, slave_id):
    try:
        response = client.write_registers(address, values, slave=slave_id)
        if response.isError():
            print(f"Error writing to hand with address {slave_id}")
        else:
            print(f"Successfully written to hand with address {slave_id}")
    except Exception as e:
        print(f"Error writing to hand with address {slave_id}: {e}")

# Predefined hand positions
POSITIONS = {
    'open': [1000, 1000, 1000, 1000, 1000, 1000],  # Fully open hand
    'fist': [0, 0, 0, 0, 0, 1000],                # Closed fist
    'grasp': [500, 500, 500, 500, 1000, 0]        # Grasping position
}

# Check if a hand is connected and responsive at a given address
def check_hand_connection(client, slave_id):
    try:
        response = client.read_holding_registers(0x03E8, count=1, slave=slave_id)  # Read HAND_ID
        if response and not response.isError():
            print(f"Hand with address {slave_id} is connected.")
            return True
    except Exception as e:
        print(f"No response from hand with address {slave_id}: {e}")
    return False

# Function to receive custom position from the socket
def parse_custom_position(data):
    try:
        values = [int(x) for x in data.split(',')]
        if all(0 <= v <= 1000 for v in values) and len(values) == 6:
            return values
    except ValueError:
        pass
    return None

# Function to handle socket communication
def start_socket_server(client, connected_hands, host='0.0.0.0', port=5000):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(1)
    print(f"Socket server listening on {host}:{port}")

    try:
        while True:
            conn, addr = server.accept()
            print(f"Connection established with {addr}")

            while True:
                data = conn.recv(1024).decode().strip()
                if not data:
                    break

                print(f"Received command: {data}")

                if data == '1':  # Open hand
                    for addr in connected_hands:
                        print(f"Opening {connected_hands[addr]} hand...")
                        write_registers(client, 0x05CE, POSITIONS['open'], slave_id=addr)

                elif data == '2':  # Close hand (Fist)
                    for addr in connected_hands:
                        print(f"Closing {connected_hands[addr]} hand to a fist...")
                        write_registers(client, 0x05CE, POSITIONS['fist'], slave_id=addr)

                elif data == '3':  # Grasp position
                    for addr in connected_hands:
                        print(f"Setting {connected_hands[addr]} hand to grasp position...")
                        write_registers(client, 0x05CE, POSITIONS['grasp'], slave_id=addr)

                elif data.startswith('4:'):  # Custom position
                    values = parse_custom_position(data[2:])
                    if values:
                        for addr in connected_hands:
                            print(f"Setting {connected_hands[addr]} hand to custom position: {values}")
                            write_registers(client, 0x05CE, values, slave_id=addr)
                    else:
                        print("Invalid custom position format. Expected: 4:value1,value2,value3,value4,value5,value6")

                elif data == '5':  # Exit command
                    print("Exit command received. Closing connection.")
                    conn.close()
                    return

                else:
                    print("Invalid command received.")
            
            conn.close()
            print("Client disconnected.")
    
    except Exception as e:
        print(f"Socket error: {e}")
    finally:
        server.close()

# Main function
def main():
    client = initialize_modbus(port='/dev/ttyUSB0', baudrate=115200)  # Change to USB1 for left hand and USB0 for right hand
    address_map = {1: 'right', 2: 'left'}
    connected_hands = {}

    try:
        # Detect connected hands
        for addr in address_map:
            if check_hand_connection(client, addr):
                connected_hands[addr] = address_map[addr]

        if not connected_hands:
            print("No hands detected. Please check connections.")
            return

        print("Starting socket server for command reception...")
        start_socket_server(client, connected_hands)

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        client.close()

if __name__ == '__main__':
    main()
