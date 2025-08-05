import socket
import struct
import threading
import serial
import time
import sys
from collections import deque

class HandController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, min_delay=0.1, name="Hand"):
        self.name = name
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.min_delay = min_delay  # Minimum 100ms between commands
        self.last_send_time = 0
        self.command_queue = deque(maxlen=1)  # Only keep the latest command
        self.running = True
        
        # Start the processing thread
        self.thread = threading.Thread(target=self._process_commands)
        self.thread.daemon = True
        self.thread.start()
        print(f"Connected to {self.name} at {port}. Ready to send commands...")
    
    def add_command(self, command_str):
        """Add a command to the queue (replaces any pending command)"""
        try:
            # Split and convert to integers
            values = [int(x) for x in command_str.split(',')]
            if len(values) != 6:
                print(f"Error ({self.name}): Expected 6 comma-separated values")
                return
            
            # Add to queue (will automatically replace previous command)
            self.command_queue.append(values)
        except Exception as e:
            print(f"Error processing command ({self.name}): {e}")

    def _process_commands(self):
        """Process commands in a separate thread with timing control"""
        while self.running:
            if self.command_queue:
                current_time = time.time()
                time_since_last = current_time - self.last_send_time
                
                # Wait until min_delay has passed
                if time_since_last >= self.min_delay:
                    values = self.command_queue.popleft()
                    self._send_command(values)
                    self.last_send_time = current_time
                else:
                    # Wait for remaining time
                    time.sleep(self.min_delay - time_since_last)
            else:
                # No commands, sleep briefly
                time.sleep(0.01)
    
    def _send_command(self, values):
        """Send command to robotic hand"""
        try:
            # Format: "v1,v2,v3,v4,v5,v6"
            command_str = ','.join(str(v) for v in values) + '\n'
            self.ser.write(command_str.encode())
            print(f"Sent to {self.name}: {command_str.strip()}")
        except Exception as e:
            print(f"Error sending command to {self.name}: {e}")
    
    def close(self):
        """Clean up resources"""
        self.running = False
        self.thread.join()
        self.ser.close()

class DualHandCommandReceiver:
    def __init__(self, host='0.0.0.0', port=5050, left_hand_controller=None, right_hand_controller=None):
        self.host = host
        self.port = port
        self.left_hand_controller = left_hand_controller
        self.right_hand_controller = right_hand_controller
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        print(f"Dual hand command server listening on {host}:{port}")
    
    def start(self):
        """Start the TCP server in a separate thread"""
        self.thread = threading.Thread(target=self._listen_for_connections)
        self.thread.daemon = True
        self.thread.start()
    
    def _listen_for_connections(self):
        """Listen for incoming connections and handle them"""
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                print(f"Connection from {addr}")
                threading.Thread(target=self._handle_client, args=(client_socket,)).start()
            except Exception as e:
                if self.running:
                    print(f"Connection error: {e}")
    
    def _handle_client(self, client_socket):
        """Handle data from a connected client"""
        try:
            while self.running:
                # Receive 48 bytes (12 integers * 4 bytes each)
                data = client_socket.recv(48)
                if not data:
                    break
                
                # Unpack the binary data to 12 integers (big-endian)
                values = struct.unpack('>12i', data)
                
                # Extract left hand values (first 6 integers)
                left_hand_values = values[:6]
                # Extract right hand values (next 6 integers)
                right_hand_values = values[6:12]
                
                # Process left hand
                if self.left_hand_controller:
                    # Reorder fingers: pinky, ring, middle, index -> index, middle, ring, pinky
                    left_reordered = [left_hand_values[3], left_hand_values[2],
                                     left_hand_values[1], left_hand_values[0],
                                     left_hand_values[4], left_hand_values[5]]
                    
                    # Invert only fingers, not thumb
                    left_inverted_fingers = [1000 - max(0, min(1000, v)) for v in left_reordered[:4]]
                    left_thumb_values = left_reordered[4:]
                    left_final_values = left_inverted_fingers + left_thumb_values
                    
                    # Format as comma-separated string
                    left_command_str = ','.join(str(v) for v in left_final_values)
                    self.left_hand_controller.add_command(left_command_str)
                    print(f"LEFT - Received: {left_hand_values} -> Sent: {left_final_values}")
                
                # Process right hand
                if self.right_hand_controller:
                    # Reorder fingers: pinky, ring, middle, index -> index, middle, ring, pinky
                    right_reordered = [right_hand_values[3], right_hand_values[2],
                                      right_hand_values[1], right_hand_values[0],
                                      right_hand_values[4], right_hand_values[5]]
                    
                    # Invert only fingers, not thumb
                    right_inverted_fingers = [1000 - max(0, min(1000, v)) for v in right_reordered[:4]]
                    right_thumb_values = right_reordered[4:]
                    right_final_values = right_inverted_fingers + right_thumb_values
                    
                    # Format as comma-separated string
                    right_command_str = ','.join(str(v) for v in right_final_values)
                    self.right_hand_controller.add_command(right_command_str)
                    print(f"RIGHT - Received: {right_hand_values} -> Sent: {right_final_values}")
        
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            client_socket.close()
    
    def close(self):
        """Clean up resources"""
        self.running = False
        try:
            self.server_socket.close()
        except:
            pass
        self.thread.join()

def main():
    # Initialize hand controllers
    left_controller = None
    right_controller = None
    
    # Try to connect to left hand (USB0)
    try:
        left_controller = HandController(port='/dev/ttyUSB0', name="Left Hand")
    except Exception as e:
        print(f"Failed to connect to left hand on /dev/ttyUSB0: {e}")
    
    # Try to connect to right hand (USB1)
    try:
        right_controller = HandController(port='/dev/ttyUSB1', name="Right Hand")
    except Exception as e:
        print(f"Failed to connect to right hand on /dev/ttyUSB1: {e}")
    
    # Check if at least one hand is connected
    if not left_controller and not right_controller:
        print("No hands connected. Exiting...")
        sys.exit(1)
    
    if left_controller:
        print("Left hand connected successfully")
    if right_controller:
        print("Right hand connected successfully")
    
    # Initialize dual hand command receiver
    receiver = DualHandCommandReceiver(
        left_hand_controller=left_controller,
        right_hand_controller=right_controller
    )
    receiver.start()
    
    try:
        print("Running dual hand controller. Press Ctrl+C to exit...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        receiver.close()
        if left_controller:
            left_controller.close()
        if right_controller:
            right_controller.close()

if __name__ == "__main__":
    main()
