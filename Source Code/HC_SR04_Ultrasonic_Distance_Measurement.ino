// Define pin numbers for ECHO and TRIG
#define ECHO_PIN 5
#define TRIG_PIN 6

// Variables to store duration and distance values
long duration;
int distance;

void setup() {
  // Set pin modes for TRIG and ECHO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Begin serial communication at 9600 bps
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test");
  Serial.println("Using Arduino UNO R3 Board");
}

void loop() {
  // Ensure TRIG is LOW for 2 microseconds before triggering pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);

  // Send a 10-microsecond HIGH pulse to TRIG
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the duration from ECHO pin
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in centimeters using the duration
  distance = duration * 0.034 / 2;

  // Output the measured distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Add a small delay before the next measurement
  delay(500);
}
