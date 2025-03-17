import serial

def initialize_serial(port='/dev/ttyUSB0', baudrate=115200):
    return serial.Serial(port, baudrate, timeout=1)

def calculate_checksum(data):
    return sum(data) & 0xFF

def create_read_frame(hand_id, address, length):
    frame = [
        0xEB,  # Packet header
        0x90,  # Packet header
        hand_id,  # Hands_ID
        0x04,  # Length of the frame data
        0x11,  # Read register instruction flag
        address & 0xFF,  # Low-order 8 bits of Address
        (address >> 8) & 0xFF,  # High-order 8 bits of Address
        length  # Length of the register to be read
    ]
    checksum = calculate_checksum(frame)
    frame.append(checksum)
    return bytearray(frame)

def send_frame(ser, frame):
    print("Sending frame:", frame.hex())  # Debugging: Print the frame sent
    ser.write(frame)
    response = ser.read(30)  # Read a 30-byte response
    print("Raw response:", response.hex())  # Debugging: Print raw response received
    return response

def parse_response(response):
    if len(response) < 7:
        print("Invalid or no response received.")
        return None

    hand_id = response[2]
    data_length = response[3] - 3  # Length of the data section
    register_address = (response[6] << 8) | response[5]  # Combine low and high byte
    data = response[7:7 + data_length]  # Extract data values
    checksum = response[-1]

    print(f"Hand ID: {hand_id}")
    print(f"Register Address: 0x{register_address:04X}")
    print(f"Data Length: {data_length} bytes")
    print(f"Data: {data}")
    print(f"Checksum: 0x{checksum:02X}")

    return hand_id, register_address, data

def main():
    ser = initialize_serial(port='/dev/ttyUSB0', baudrate=115200)
    hand_id = 1
    address = 0x060A  # Example register address
    length = 12  # Length of data to read

    print("Reading data from the robotic hand...")

    try:
        read_frame = create_read_frame(hand_id, address, length)
        response = send_frame(ser, read_frame)
        
        if response:
            parse_response(response)
        else:
            print("No response received. Check the connection to the hand.")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        ser.close()

if __name__ == '__main__':
    main()

