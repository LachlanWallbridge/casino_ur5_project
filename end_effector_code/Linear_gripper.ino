#include <Servo.h>

Servo myServo;        
int servoPin = 9;               // PWM-capable pin on Teensy
int currentSensorPin = A0;      // analog pin for ACS712
int currentPos = 90;            // start at mid position
int desiredPos = 90;


void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(currentPos);
  
  Serial.println("Teensy Servo Controller with Current Sensing");
  Serial.println("Enter desired servo angle (0–180):");
}

void loop() {
  // Check if new data is available over serial
  if (Serial.available() > 0) {
    desiredPos = Serial.parseInt();   // read integer from Serial
    if (desiredPos >= 0 && desiredPos <= 180) {
      Serial.print("Moving to: ");
      Serial.println(desiredPos);
      moveServoSmooth(currentPos, desiredPos, 5); // move with small steps
      currentPos = desiredPos;
    }
    else if (desiredPos == 200) {
      myServo.detach();
      Serial.println("Servo detached.");
    } else {
      Serial.println("Invalid input! Enter value 0–180.");
    }
  }

  delay(100); // small delay for readable current output
}

// Move servo smoothly between two positions
void moveServoSmooth(int start, int end, int stepDelay) {
  if (end > start) {
    for (int pos = start; pos <= end; pos++) {
      myServo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = start; pos >= end; pos--) {
      myServo.write(pos);
      delay(stepDelay);
    }
  }
}
