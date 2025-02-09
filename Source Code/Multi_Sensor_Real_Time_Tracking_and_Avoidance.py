import cv2
import numpy as np
import serial

# Constants
NUM_ULTRASONIC_SENSORS = 7
MAX_DISTANCE = 30
state_space_dim = 4
distances = [0.0] * NUM_ULTRASONIC_SENSORS
tracks = {}
track_types = {}

# Serial connection
ser = serial.Serial('COMX', 9600)  # Replace 'COMX' with your Arduino port

# Kalman Filter initialization
def initialize_track(box):
    track = cv2.KalmanFilter(dynamParams=state_space_dim, measureParams=2, controlParams=0)
    track.statePre = np.array([box[2], box[3], 0, 0], dtype=np.float32)
    track.statePost = np.array([box[2], box[3], 0, 0], dtype=np.float32)
    return track

# Video capture
cap = cv2.VideoCapture('your_video_file.mp4')  # Replace with your video file

while cap.isOpened():
    ret, frame = cap.read()

    # Read ultrasonic sensor data
    try:
        sensor_data = ser.readline().decode('utf-8').rstrip().split(',')
        distances = [float(distance) for distance in sensor_data[:NUM_ULTRASONIC_SENSORS]]
    except ValueError:
        continue

    front_distances = distances[:5]

    # Perform object detection using YOLOv5
    results = model(frame)
    detections = results.pandas().xyxy[0]
    object_types = detections["name"].tolist()

    # Decision-making logic
    if any(distance < MAX_DISTANCE for distance in front_distances) or len(tracks) > 0:
        print("Decision: Stop the car. Obstacle detected.")
    else:
        print("Decision: Continue driving.")

    # Display frame
    cv2.imshow('YOLOv5 Object Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
