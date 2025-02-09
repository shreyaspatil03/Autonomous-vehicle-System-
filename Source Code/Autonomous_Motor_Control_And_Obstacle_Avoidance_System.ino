const int throttlePin = 9;       
const int ultrasonicTriggerPin = 2; 
const int ultrasonicEchoPin = 3;    

int desiredSpeed = 0;  
int previousSpeed = 0; 
bool stopCar = false;  

void setup() {
  pinMode(throttlePin, OUTPUT);
  analogWrite(throttlePin, 0);          
  pinMode(ultrasonicTriggerPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);    
  Serial.begin(9600);                   
}

void loop() {
  if (Serial.available() > 0) {
    String inputCommand = Serial.readStringUntil('\n');  

    if (inputCommand == "start") {
      stopCar = false;
      analogWrite(throttlePin, desiredSpeed);
      Serial.print("Current Speed (%): ");
      Serial.println(map(desiredSpeed, 0, 255, 0, 100));
    }
    else if (inputCommand == "stop") {
      stopCar = true;
      analogWrite(throttlePin, 0);
    }
    else {
      int newSpeed = inputCommand.toInt();
      desiredSpeed = map(newSpeed, 0, 100, 0, 255);
      desiredSpeed = constrain(desiredSpeed, 0, 255);
      if (!stopCar) analogWrite(throttlePin, desiredSpeed);
      Serial.print("Desired Speed (%): ");
      Serial.println(map(desiredSpeed, 0, 255, 0, 100));
    }
  }

  digitalWrite(ultrasonicTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW);
  
  long duration = pulseIn(ultrasonicEchoPin, HIGH);
  float distance = duration * 0.034 / 2.0;

  if (distance <= 50) {
    stopCar = true;
    analogWrite(throttlePin, 0);
    Serial.println("Object detected within 50cm! Car stopped.");
  }
  else if (stopCar) {
    analogWrite(throttlePin, previousSpeed);
    Serial.print("Resumed Speed (%): ");
    Serial.println(map(previousSpeed, 0, 255, 0, 100));
    stopCar = false;
  }

  previousSpeed = desiredSpeed;
  delay(100);  
}
