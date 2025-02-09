const int THROTTLE_PIN = 9;  // PWM pin for motor controller

int desiredSpeed = 0;        
bool stopCar = false;        

void setup() {
  pinMode(THROTTLE_PIN, OUTPUT);
  analogWrite(THROTTLE_PIN, 0);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String inputCommand = Serial.readStringUntil('\n');
    handleInputCommand(inputCommand);
  }
  delay(100);
}

void handleInputCommand(String command) {
  if (command == "start") {
    stopCar = false;
    analogWrite(THROTTLE_PIN, desiredSpeed);
    printSpeed("Current Speed");
  } 
  else if (command == "X") {
    stopCar = true;
    analogWrite(THROTTLE_PIN, 0);
    Serial.println("Car Stopped.");
  } 
  else {
    int newSpeed = command.toInt();
    desiredSpeed = mapSpeed(newSpeed);
    if (!stopCar) {
      analogWrite(THROTTLE_PIN, desiredSpeed);
    }
    printSpeed("Desired Speed");
  }
}

int mapSpeed(int speedPercent) {
  return constrain(map(speedPercent, 0, 100, 0, 255), 0, 255);
}

void printSpeed(const String &label) {
  Serial.print(label + " (%): ");
  Serial.println(map(desiredSpeed, 0, 255, 0, 100));
}
