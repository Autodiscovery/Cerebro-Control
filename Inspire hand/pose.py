import socket
import time

def send_pose_command(pose, wait_for_ack=True):
    host = '127.0.0.1'
    port = 8080
    pose_str = ' '.join(map(str, pose))

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(pose_str.encode('utf-8'))

        if wait_for_ack:
            data = s.recv(1024)
            if data.decode('utf-8') == "done":
                print("Acknowledgment received: Pose execution completed.")

if __name__ == "__main__":
    try:
        hands_straight_pose = [-2.0, 0.78, 0.78, 0.78, -2.0, -0.78, -0.78, 0.78, 0]
        t_pose = [0, 0.78, 0, 0.78, 0, -0.78, 0, 0.78, 0]

        send_pose_command(hands_straight_pose)
        send_pose_command(t_pose)
        send_pose_command(hands_straight_pose)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, sending exit command.")
        send_pose_command(["exit"], wait_for_ack=False)
    finally:
        # Ensure exit command is sent after last pose as well
        send_pose_command(["exit"], wait_for_ack=False)

