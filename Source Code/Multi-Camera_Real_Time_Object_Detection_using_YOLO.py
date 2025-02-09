import cv2
import numpy as np

def load_yolo_model():
    weights_file = "C:/Users/Dalbir Singh/Downloads/abcd/yolov3.weights"
    config_file = "C:/Users/Dalbir Singh/Downloads/abcd/yolov3.cfg"
    net = cv2.dnn_DetectionModel(config_file, weights_file)
    net.setInputSize(416, 416)
    net.setInputScale(1.0 / 255)
    net.setInputSwapRB(True)
    with open("C:/Users/Dalbir Singh/Downloads/abcd/coco.names", "rt") as f:
        names = f.read().rstrip('\n').split('\n')
    return net, names

def perform_object_detection(frame, net, classes):
    class_ids, scores, boxes = net.detect(frame, confThreshold=0.4, nmsThreshold=0.2)
    if isinstance(class_ids, np.ndarray):
        for class_id, score, box in zip(class_ids.flatten(), scores, boxes):
            class_id = int(class_id)
            color = (0, 255, 0)
            cv2.rectangle(frame, box, color, 2)
            cv2.putText(frame, classes[class_id-1].upper(), (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, color, 2)
    return frame

def main():
    num_cameras = 2
    cam_indexes = [0, 2]  # Camera indexes may need adjustment
    net, classes = load_yolo_model()

    capture_objects = [cv2.VideoCapture(cam_idx) for cam_idx in cam_indexes]

    # Check if cameras are opened correctly
    for i, capture in enumerate(capture_objects):
        if not capture.isOpened():
            print(f"Camera {i} could not be opened!")
            return

    while True:
        frames = []
        for capture in capture_objects:
            ret, frame = capture.read()
            if not ret:
                print("Failed to grab frame from camera.")
                break
            frames.append(frame)

        if len(frames) == num_cameras:
            combined_frame = np.hstack(frames)
            combined_frame = perform_object_detection(combined_frame, net, classes)
            cv2.imshow("Multiple Cameras", combined_frame)

        key = cv2.waitKey(1)
        if key == 27:  # Press 'Esc' to exit
            break

    for capture in capture_objects:
        capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
