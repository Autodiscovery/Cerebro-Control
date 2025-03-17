import zmq
import pickle
import cv2

def image_receiver():
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect("tcp://192.168.123.241:5555")  # Connect to PC1

    while True:
        data = socket.recv()
        frame_data = pickle.loads(data)
        frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow("Camera Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

image_receiver()
