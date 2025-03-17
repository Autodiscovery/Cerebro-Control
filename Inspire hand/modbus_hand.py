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
    'fist': [0, 0, 0, 0, 0, 1000],                   # Closed fist
    'grasp': [500, 500, 500, 500, 1000, 0]         # Grasping position
}

def get_user_input():
    print("\nAvailable commands:")
    print("1: Open Hand")
    print("2: Close Hand (Fist)")
    print("3: Grasp")
    print("4: Enter Custom Position")
    print("5: Exit")
    return input("Choose an option: ")

# Function to get custom position input with error handling
def get_custom_position():
    values = []
    for i in range(6):
        while True:
            try:
                angle = int(input(f"Enter angle for DOF {i + 1} (0-1000): "))
                if 0 <= angle <= 1000:
                    values.append(angle)
                    break
                else:
                    print("Please enter a value between 0 and 1000.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
    return values

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

def main():
    client = initialize_modbus(port='/dev/ttyUSB0', baudrate=115200)
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

        # Loop for user input
        while True:
            print("\nConnected hands:", connected_hands)
            choice = get_user_input()

            if choice == '1':
                for addr in connected_hands:
                    print(f"Opening {connected_hands[addr]} hand...")
                    write_registers(client, 0x05CE, POSITIONS['open'], slave_id=addr)
            elif choice == '2':
                for addr in connected_hands:
                    print(f"Closing {connected_hands[addr]} hand to a fist...")
                    write_registers(client, 0x05CE, POSITIONS['fist'], slave_id=addr)
            elif choice == '3':
                for addr in connected_hands:
                    print(f"Setting {connected_hands[addr]} hand to grasp position...")
                    write_registers(client, 0x05CE, POSITIONS['grasp'], slave_id=addr)
            elif choice == '4':
                # Custom position input for both hands
                values = get_custom_position()
                for addr in connected_hands:
                    print(f"Setting {connected_hands[addr]} hand to custom position...")
                    write_registers(client, 0x05CE, values, slave_id=addr)
            elif choice == '5':
                print("Exiting...")
                break
            else:
                print("Invalid option. Try again.")
    
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        client.close()

if __name__ == '__main__':
    main()

