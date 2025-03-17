from pymodbus.client import ModbusSerialClient as ModbusClient

# Initialize Modbus serial communication
def initialize_modbus(port='/dev/ttyUSB0', baudrate=115200):
    client = ModbusClient(port=port, baudrate=baudrate, stopbits=1, bytesize=8, parity='N', timeout=2)  # Increased timeout
    client.connect()
    return client

# Function to read a single register with error checking
def read_register(client, address, slave_id):
    try:
        response = client.read_holding_registers(address, count=1, slave=slave_id)
        if response is None:
            print(f"Failed to receive a response from address {address}")
            return None
        if response.isError():
            print("Error reading the register")
            return None
        return response.registers[0]
    except Exception as e:
        print(f"Error reading register: {e}")
        return None

# Function to write a single register with error checking
def write_register(client, address, value, slave_id):
    try:
        response = client.write_register(address, value, slave=slave_id)
        if response is None:
            print(f"Failed to receive a response when writing to address {address}")
            return False
        if response.isError():
            print("Error writing to register")
            return False
        print(f"Successfully written {value} to register {address}")
        return True
    except Exception as e:
        print(f"Error writing to register: {e}")
        return False

# Main function to change the hand address
def main():
    client = initialize_modbus(port='/dev/ttyUSB0', baudrate=115200)
    current_address_register = 0x03E8  # Address for HAND_ID

    try:
        # Step 1: Prompt user for the current address
        while True:
            try:
                current_address = int(input("Enter the current address (ID) of the hand (1-254): "))
                if 1 <= current_address <= 254:
                    break
                else:
                    print("Please enter a value between 1 and 254.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

        # Step 2: Read the current address to verify it
        print(f"Attempting to read the current address using slave ID: {current_address}")
        read_address = read_register(client, current_address_register, slave_id=current_address)
        if read_address is not None:
            print(f"Verified current hand address (ID): {read_address}")
        else:
            print("Unable to read the current address. Please check the connection or try a different address.")
            return

        # Step 3: Get new address from the user
        while True:
            try:
                new_address = int(input("Enter new address (ID) for the hand (1-254): "))
                if 1 <= new_address <= 254:
                    break
                else:
                    print("Please enter a value between 1 and 254.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

        # Step 4: Write the new address to the HAND_ID register
        print(f"Attempting to write new address {new_address} to HAND_ID register")
        if write_register(client, current_address_register, new_address, slave_id=current_address):
            print(f"New address {new_address} written successfully.")
        else:
            print("Failed to write the new address. Please check the connection.")
            return

        # Step 5: Confirm the address change by reading with the new address
        print(f"Attempting to read the new address using slave ID: {new_address}")
        confirmed_address = read_register(client, current_address_register, slave_id=new_address)
        if confirmed_address == new_address:
            print(f"Address successfully changed to: {confirmed_address}")
        else:
            print("Failed to confirm the address change. Please try again.")
    
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        client.close()

if __name__ == '__main__':
    main()

