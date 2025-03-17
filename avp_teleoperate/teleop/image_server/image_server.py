import cv2
import zmq
import depthai as dai
import pickle
import zlib

def create_rgb_cam_pipeline():
    print("Creating pipeline: RGB CAM -> XLINK OUT")
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    xout_video = pipeline.create(dai.node.XLinkOut)

    # cam.setPreviewSize(540, 540)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)

    xout_video.setStreamName('rgb_video')
    cam.video.link(xout_video.input)

    return pipeline

def start_server():
    # Initialize DepthAI device and pipeline
    # device_info = dai.DeviceInfo("169.254.1.160")
    device_info = dai.DeviceInfo()
    with dai.Device(device_info) as device:
        pipeline = create_rgb_cam_pipeline()
        device.startPipeline(pipeline)

        # Set ZeroMQ context and socket
        context = zmq.Context()
        socket = context.socket(zmq.PUSH)
        socket.setsockopt(zmq.SNDHWM, 1)
        socket.bind("tcp://192.168.123.191:5555")
        print("The server has started, waiting for client connections...")

        q_video = device.getOutputQueue(name='rgb_video', maxSize=8, blocking=False)

        while True:
            in_video = q_video.get()
            frame = in_video.getCvFrame()

            # Resize frame to half of its original resolution
            height, width = frame.shape[:2]
            resized_frame = cv2.resize(frame, (width // 2, height // 2))

            # Encoding image
            ret, encoded_frame = cv2.imencode('.jpg', resized_frame)
            if not ret:
                continue

            # Compressing data using pickle and zlib
            data = pickle.dumps(encoded_frame)
            socket.send(data)
            # compressed_data = zlib.compress(data)
            #
            # # Sending data in pieces
            # chunk_size = 80000
            # num_chunks = len(compressed_data) // chunk_size + 1
            # for i in range(num_chunks):
            #     start = i * chunk_size
            #     end = start + chunk_size
            #     chunk = compressed_data[start:end]
            #     socket.send(chunk)

if __name__ == "__main__":
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getMxId()} {device.state}")
    start_server()
