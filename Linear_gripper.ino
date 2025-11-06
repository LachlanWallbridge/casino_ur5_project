#include <Servo.h>

Servo myServo;        
int servoPin = 9;               // PWM-capable pin on Teensy
int currentSensorPin = A0;      // analog pin for ACS712
int currentPos = 90;            // start at mid position
int desiredPos = 90;

// Calibration constants for ACS712 30A
const float sensorVcc = 5.0;        // ACS712 powered by 5V supply
const float Vzero = sensorVcc / 2;  // 2.5V at 0 A
const float sensitivity = 0.066;    // 66 mV/A for 30A version
const int adcMax = 4095;            // 12-bit ADC on Teensy 4.1
const float adcRef = 3.3;           // Teensy ADC reference voltage
const float divider = 3.3 / 5.0;    // voltage divider scaling 0–5V → 0–3.3V

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(currentPos);
  
  Serial.println("Teensy Servo Controller with Current Sensing");
  Serial.println("Enter desired servo angle (0–180):");
}

void loop() {
  // Read current continuously
  float current = readCurrent();
  Serial.print("Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

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

// Function to read and convert ACS712 analog voltage to current
float readCurrent() {
  int sensorValue = analogRead(currentSensorPin);
  
  // Convert ADC reading to Teensy voltage
  float voltageADC = (sensorValue * adcRef) / adcMax;
  
  // Undo voltage divider to get actual sensor voltage
  float voltageSensor = voltageADC / divider;
  
  // Convert sensor voltage to current
  float current = (voltageSensor - Vzero) / sensitivity;

  
  return current;
}

