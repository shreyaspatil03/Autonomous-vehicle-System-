import serial
import time
from picamera import PiCamera

# Set up serial connection
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Initialize camera
camera = PiCamera()

def capture_image():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    image_filename = f"image_{timestamp}.jpg"
    camera.capture(image_filename)
    print(f"Image captured: {image_filename}")

def process_sensor_data(sensor_data):
    for i, distance in enumerate(sensor_data):
        print(f"Sensor {i + 1}: {distance} cm")

def decision_making(distance_sensor1, distance_sensor2):
    if distance_sensor1 < 30 and distance_sensor2 < 30:
        print("Decision: Both sensors detected an obstacle.")

try:
    while True:
        sensor_data = ser.readline().decode('utf-8').rstrip().split(',')
        process_sensor_data(sensor_data)

        distance_sensor1 = float(sensor_data[0])
        distance_sensor2 = float(sensor_data[1])

        capture_image()
        decision_making(distance_sensor1, distance_sensor2)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
