import mujoco
import numpy as np
from pytransform3d import rotations

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices

from pathlib import Path
import time
import yaml
from multiprocessing import Process, shared_memory, Queue, Manager, Event, Lock

import cv2
import zmq
import pickle
import zlib
import socket

import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from robot_control.robot_hand import H1HandController
# from teleop.robot_control.robot_arm import H1ArmController
from teleop.robot_control.robot_arm_ik import Arm_IK

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

import mujoco
import mujoco.viewer

_HERE = Path(__file__).parent
_XML = _HERE / "unitree_h1" / "empty.xml"


def send_data(server_socket, data):
    try:
        server_socket.send(data.encode('utf-8'))
    except ConnectionResetError as e:
        print(f"Connection reset by peer: {e}")
        # Attempt to reconnect
        server_socket.close()
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.connect(("server_address", server_port))
        server_socket.send(data.encode('utf-8'))


def image_receiver(image_queue, resolution, crop_size_w, crop_size_h):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect("tcp://192.168.123.191:5555")

    while True:
        compressed_data = b''
        while True:
            chunk = socket.recv()
            compressed_data += chunk
            if len(chunk) < 60000:
                break
        data = zlib.decompress(compressed_data)
        frame_data = pickle.loads(data)

        # Decode and display the image
        frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        sm.write_image(frame)
        # Control receiving frequency
        time.sleep(0.01)


class SharedMemoryImage:
    def __init__(self, img_shape):
        self.resolution = img_shape  # (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0] - self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

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
        self.resolution = (800, 640)  # (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0] - self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, ngrok=True)
        self.processor = VuerPreprocessor()

        # RetargetingConfig.set_default_urdf_dir('../assets')
        # with Path(config_file_path).open('r') as f:
        #     cfg = yaml.safe_load(f)
        # left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        # right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        # self.left_retargeting = left_retargeting_config.build()
        # self.right_retargeting = right_retargeting_config.build()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        # left_wrist_mat[2, 3] +=0.45
        # right_wrist_mat[2,3] +=0.45
        # left_wrist_mat[0, 3] +=0.20
        # right_wrist_mat[0,3] +=0.20

        ## left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[0, 1, 2, 3, 4, 5, 6, 7]]
        # left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[]]
        ## right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[]]

        # return head_rmat, left_wrist_mat, right_wrist_mat, left_qpos, right_qpos
        return left_wrist_mat, right_wrist_mat


if __name__ == '__main__':
    manager = Manager()
    image_queue = manager.Queue()
    teleoperator = VuerTeleop('inspire_hand.yml')

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # mujoco loading
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    h1hand = H1HandController()
    # h1arm = H1ArmController()
    arm_ik = Arm_IK()
    sm = SharedMemoryImage((800, 640))
    image_process = Process(target=image_receiver, args=(sm, teleoperator.resolution, teleoperator.crop_size_w, teleoperator.crop_size_h))
    image_process.start()

    start_time = time.time()  # Capture script start time
    # server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server_address = ('192.168.123.162', 8080)
    # server_socket.connect(server_address)

    with mujoco.viewer.launch_passive(
            model=model, data=data, show_left_ui=True, show_right_ui=True
    ) as viewer:

        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
        mujoco.mj_forward(model, data)

        try:
            user_input = input("Please enter the start signal (enter 's' to start the subsequent program):")
            if user_input.lower() == 's':
                # Setup ZMQ publisher
                # context = zmq.Context()
                # socket_init = False
                # socket = None
                # socket = context.socket(zmq.PUB)
                # socket.bind("tcp://*:8080")  # Change the port if needed
                # socket.connect("tcp://192.168.123.162:8080")  # Uncomment and comment above line if running h1_joint on h1 PC2

                while viewer.is_running():
                    current_time = time.time()
                    elapsed_time = current_time - start_time

                    armstate = None
                    armv = None
                    frame = sm.read_image()
                    np.copyto(teleoperator.img_array, np.array(frame))
                    handstate = h1hand.get_hand_state()

                    q_poseList = np.zeros(20)
                    q_tau_ff = np.zeros(20)
                    # armstate,armv = h1arm.GetMotorState()

                    # hands uncomment# head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
                    # print pose as quaternion
                    # print(rotations.quaternion_from_matrix(right_pose[:3,:3]))
                    # print(rotations.quaternion_from_matrix(left_pose[:3,:3]))
                    # print pose translation
                    # print(right_pose[:3,3])
                    # print(left_pose[:3,3])

                    # Fix the urdf order mismatch between the hand and the arm
                    # import ipdb; ipdb.set_trace()
                    # original_armstate = armstate
                    # armstate = np.concatenate((armstate[4:8], armstate[0:4]))
                    # armv = np.concatenate((armv[4:8], armv[0:4]))

                    left_pose, right_pose = teleoperator.step()

                    sol_q, tau_ff, flag = arm_ik.ik_fun(left_pose, right_pose, None, None)

                    # print(sol_q)
                    # print(tau_ff)

                    # Extract last 8 values, append a zero, format as space-separated string
                    sol_q_values = sol_q[-8:].tolist()  # Get last 8 values
                    sol_q_values.append(0)  # Append zero
                    print(sol_q_values)

                    tau_ff_values = tau_ff[-8:].tolist()  # Get last 8 values
                    tau_ff_values.append(0)  # Append zero
                    print(tau_ff_values)

                    # Combine into a single space-separated string
                    combined_data = " ".join(map(str, sol_q_values + tau_ff_values))
                    # send_data(server_socket, combined_data)

                    # sol_q = np.concatenate((sol_q[4:8], sol_q[0:4]))
                    # tau_ff = np.concatenate((tau_ff[4:8], tau_ff[0:4]))

                    # if flag:
                    #     q_poseList[12:20] = sol_q
                    #     q_tau_ff[12:20] = tau_ff
                    # else:
                    #     q_poseList[12:20] = original_armstate
                    #     q_tau_ff = np.zeros(20)

                    # h1arm.SetMotorPose(q_poseList, q_tau_ff)
                    # print('Poselist:', q_poseList)

                    # if right_qpos is not None and left_qpos is not None:
                    #     # 4,5: index 6,7: middle, 0,1: pinky, 2,3: ring, 8,9: thumb
                    #     right_angles = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                    #     right_angles.append(1.2 - right_qpos[8])
                    #     right_angles.append(0.5 - right_qpos[9])
                    #
                    #     left_angles = [1.7- left_qpos[i] for i in  [4, 6, 2, 0]]
                    #     left_angles.append(1.2 - left_qpos[8])
                    #     left_angles.append(0.5 - left_qpos[9])
                    #     h1hand.crtl(right_angles,left_angles)
                    # time.sleep(0.2)

                    for i in [11,12,13,14,15,16,17,18]:  # model.nu is the number of actuators
                        # add corresponding joint q target and to motor commands
                        actuator_name = data.actuator(i).name
                        try:
                            correspond_joint = data.joint(actuator_name)
                        except KeyError:
                            continue
                        correspond_joint_id = correspond_joint.id
                        data.ctrl[i] = (
                                0  # feedforward term tau
                                + 200 * (sol_q_values[i - 11] - data.qpos[correspond_joint_id])  # position control kp
                                # + 200 * (keyframe_qpos[correspond_joint_id] - data.qpos[correspond_joint_id])  # position control kp
                                + 20 * (0 - data.qvel[correspond_joint_id])  # velocity control kd * dq
                        )
                        # send via websocket

                    mujoco.mj_step(model, data)

                    # Visualize at fixed FPS.
                    viewer.sync()

        except KeyboardInterrupt:
            print("Shutting down...")
            exit(0)
