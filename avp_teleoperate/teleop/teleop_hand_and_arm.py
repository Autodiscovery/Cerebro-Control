import numpy as np
from pytransform3d import rotations
import struct

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig

from pathlib import Path
import time
import yaml
from multiprocessing import Process, shared_memory, Queue, Manager, Event, Lock

import cv2
import zmq
import pickle
import socket

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from robot_control.robot_hand import H1HandController
from teleop.robot_control.robot_arm_ik import Arm_IK

def send_data(server_socket, data):
    # Append a newline to mark the end of the message.
    message = data + "\n"
    try:
        server_socket.sendall(message.encode('utf-8'))
    except ConnectionResetError as e:
        print(f"Connection reset by peer: {e}")
        # Attempt to reconnect if necessary
        server_socket.close()
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.connect(("192.168.123.162", 8080))
        server_socket.sendall(message.encode('utf-8'))

def send_hand_data(hands_socket, left_angles, right_angles):
    # Convert angles to range 0-1000
    left_scaled = [max(0, min(1000, int(angle * 1000))) for angle in left_angles]
    right_scaled = [max(0, min(1000, int(angle * 1000))) for angle in right_angles]

    data = left_scaled + right_scaled
    byte_data = struct.pack('>' + 'i' * len(data), *data)

    try:
        hands_socket.sendall(byte_data)
    except ConnectionResetError as e:
        print(f"Connection reset by peer: {e}")
        hands_socket.close()
        hands_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        hands_socket.connect(("192.168.123.162", 5050))
        hands_socket.sendall(byte_data)

def image_receiver(image_queue, resolution, crop_size_w, crop_size_h):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    # socket.connect("tcp://100.82.243.58:5555")    # tailscale IP
    socket.connect("tcp://192.168.123.241:5555")

    while True:
        data = socket.recv()
        frame_data = pickle.loads(data)

        # Decode and display the image
        frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        sm.write_image(frame)


class SharedMemoryImage:
    def __init__(self, img_shape):
        self.resolution = img_shape #(720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        self.lock = Lock()

    def write_image(self, image):
        with self.lock:
            np.copyto(self.img_array, image)

    def read_image(self):
        with self.lock:
            image_copy = self.img_array.copy()
            return image_copy

    def cleanup(self):
        self.shm.close()
        self.shm.unlink()

class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (800,640) #(720, 1280)    #made to fit luxonis depth cams
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]
        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, ngrok=True)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()
    
    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        left_wrist_mat[2, 3] +=0.35     #change arm vertical offset
        right_wrist_mat[2,3] +=0.35
        # left_wrist_mat[0, 3] +=0.20       #change arm horizontal offset
        # right_wrist_mat[0,3] +=0.20

        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        return head_rmat, left_wrist_mat, right_wrist_mat, left_qpos, right_qpos


if __name__ == '__main__':
    manager = Manager()
    image_queue = manager.Queue()
    teleoperator = VuerTeleop('inspire_hand.yml')

    h1hand = H1HandController()
    arm_ik = Arm_IK()
    sm = SharedMemoryImage((800,640))
    image_process = Process(target=image_receiver, args=(sm, teleoperator.resolution, teleoperator.crop_size_w, teleoperator.crop_size_h))
    image_process.start()

    SEND_INTERVAL = 0.5  # Time interval to send control loop data in seconds (Do not increase unless made changes in C++ side)

    # H1_control socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle’s algorithm
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)  # Reduce buffer size
    server_address = ('192.168.123.162', 8080)
    # server_address = ('100.100.152.53', 8080)   # tailscale IP
    server_socket.connect(server_address)

    # Hands socket
    hands_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    hands_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # hands_socket.connect(("100.100.152.53", 5050))  # tailscale IP
    hands_socket.connect(("192.168.123.162", 5050))

    last_sent_time = time.time()
            
    try:
        user_input = input("Please enter the start signal (enter 's' to start the subsequent program):")
        if user_input.lower() == 's':   
            while True:
            
                frame = sm.read_image()
                np.copyto(teleoperator.img_array, np.array(frame))
                handstate = h1hand.get_hand_state()

                q_tau_ff=np.zeros(20)

                head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
                
                # left_pose, right_pose = teleoperator.step()
                
                sol_q ,tau_ff, flag = arm_ik.ik_fun(left_pose, right_pose, None,None)
                
                # Extract last 8 values, append a zero, format as space-separated string
                sol_q_values = [round(val, 2) for val in sol_q[-8:].tolist()]  # Get last 8 values
                sol_q_values.append(0)  # Append zero

                tau_ff_values = [round(val, 2) for val in tau_ff[-8:].tolist()]  # Get last 8 values
                tau_ff_values.append(0)  # Append zero

                # Combine into a single space-separated string
                combined_data = " ".join(map(str, sol_q_values + tau_ff_values))
                
                # Check if enough time has passed before sending new data
                current_time = time.time()
                if current_time - last_sent_time >= SEND_INTERVAL:
                    print('Sol_q: ', sol_q_values)
                    print('Tau_ff: ', tau_ff_values)
                    send_data(server_socket, combined_data)
                    last_sent_time = current_time  # Update the last sent time

                if right_qpos is not None and left_qpos is not None:
                    # 4,5: index 6,7: middle, 0,1: pinky, 2,3: ring, 8,9: thumb
                    right_angles = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                    right_angles.append(1.2 - right_qpos[8])
                    right_angles.append(0.5 - right_qpos[9])
                
                    left_angles = [1.7- left_qpos[i] for i in  [4, 6, 2, 0]]
                    left_angles.append(1.2 - left_qpos[8])
                    left_angles.append(0.5 - left_qpos[9])
                    h1hand.crtl(right_angles,left_angles)

                    send_hand_data(hands_socket, left_angles, right_angles)

                    print("Right hand:", right_angles)
                    print("Left hand:", left_angles)

    except KeyboardInterrupt:
        print("Shutting down...")
        exit(0)