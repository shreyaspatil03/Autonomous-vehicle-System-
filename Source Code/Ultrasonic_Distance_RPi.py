import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
trig_pin = 23
echo_pin = 24

GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

def get_distance():
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    start_time = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        start_time = time.time()

    stop_time = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        stop_time = time.time()

    duration = stop_time - start_time
    distance = (duration * 34300) / 2

    return distance

try:
    while True:
        dist = get_distance()
        print("Distance:", dist, "cm")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
