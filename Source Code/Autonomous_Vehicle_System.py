import cv2
import numpy as np
import serial
from collections import defaultdict
import torch

# Load YOLOv5 model for object detection
model = torch.hub.load('ultralytics/yolov5:v6.0', 'yolov5s')
model = model.autoshape()

# Set constants for data association and Kalman filter
MAX_DISTANCE = 50
MIN_IOU = 0.5
state_space_dim = 4
measurement_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
process_noise_cov = np.eye(state_space_dim) * 1e-3
measurement_noise_cov = np.eye(2) * 1e-1

# Initialize tracks dictionary and types
tracks = defaultdict(lambda: cv2.KalmanFilter(dynamParams=state_space_dim, measureParams=2, controlParams=0))
track_types = {}

# Set up serial communication for ultrasonic sensors
ser = serial.Serial('COM4', 9600)

# Open the video capture (USB camera)
usb_camera_index = 0
cap = cv2.VideoCapture(usb_camera_index)

while cap.isOpened():
    ret, frame = cap.read()

    try:
        # Read sensor data (distance readings from Arduino)
        sensor_data = ser.readline().decode('utf-8').rstrip().split(',')
        sensor_distances = [float(distance) for distance in sensor_data]
    except ValueError as e:
        continue

    # Object detection with YOLOv5
    results = model(frame)
    detections = results.pandas().xyxy[0]
    object_types = detections["name"].tolist()

    # Decision-making based on sensor distances
    if sensor_distances[0] < 50 or sensor_distances[1] < 50:
        print("Decision: Stop the car. Obstacle detected by front sensors.")
    else:
        print("Decision: Continue driving. No obstacle detected.")

    # Draw bounding boxes for detected objects
    for i, (box, obj_type) in enumerate(zip(detections.itertuples(), object_types)):
        xmin, ymin, xmax, ymax = box.xmin, box.ymin, box.xmax, box.ymax
        confidence = box.confidence
        cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        label = f"{obj_type} {confidence:.2f}"
        cv2.putText(frame, label, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    unmatched_detections = list(range(len(detections)))

    # Update Kalman filter for tracking
    for track_id, track_filter in tracks.items():
        if not track_filter.statePost.any():
            continue

        track_filter.predict()

        closest_detection = None
        min_distance = float("inf")

        # Find the closest detection to the track
        for i, (box, obj_type) in enumerate(zip(detections.itertuples(), object_types)):
            center_x, center_y = int(box[2]), int(box[3])
            distance = np.linalg.norm(track_filter.statePost[:2] - np.array([center_x, center_y]))

            if distance < min_distance and obj_type == track_types.get(track_id, None):
                min_distance = distance
                closest_detection = (box, i)

        # Correct track with closest detection
        if closest_detection and min_distance < MAX_DISTANCE:
            correction = np.array([closest_detection[0][2], closest_detection[0][3]], dtype=np.float32)
            track_filter.correct(correction)
            unmatched_detections = [i for i in unmatched_detections if i != closest_detection[1]]

    # Update detections and create new tracks for unmatched objects
    detections = detections.iloc[unmatched_detections]

    for i, (box, obj_type) in enumerate(zip(detections.itertuples(), object_types)):
        tracks[i] = cv2.KalmanFilter(dynamParams=state_space_dim, measureParams=2, controlParams=0)
        tracks[i].statePre = np.array([box[2], box[3], 0, 0], dtype=np.float32)
        tracks[i].statePost = np.array([box[2], box[3], 0, 0], dtype=np.float32)
        track_types[i] = obj_type

    # Draw tracked objects and their IDs
    for track_id, track_filter in tracks.items():
        center_x, center_y = int(track_filter.statePost[0]), int(track_filter.statePost[1])
        cv2.rectangle(frame, (center_x - 5, center_y - 5), (center_x + 5, center_y + 5), (0, 255, 0), 2)
        cv2.putText(frame, f"{track_id} ({track_types[track_id]})", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('YOLOv5 Object Tracking', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Escape key to exit
        break

cap.release()
cv2.destroyAllWindows()
