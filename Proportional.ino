// Define the pins for the sensors
int leftSensorPin = A0;   // Left sensor connected to Analog pin 0
int rightSensorPin = A1;  // Right sensor connected to Analog pin 1

// Define the motor pins
int leftMotorPin1 = 9;   // Left motor pin 1 connected to pin 9
int leftMotorPin2 = 6;   // Left motor pin 2 connected to pin 6
int rightMotorPin1 = 5;  // Right motor pin 1 connected to pin 5
int rightMotorPin2 = 3;  // Right motor pin 2 connected to pin 3
int enAPin = 10;         // enA connected to pin 10
int enBPin = 11;         // enB connected to pin 11

// Define constants for proportional control
float Kp = 0.3;  // Proportional constant
// Define the buzzer pin
const int buzzerPin = 12;

// Define the notes and durations of the melody
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
int noteDurations[] = {4, 4, 4, 4, 4, 4, 4, 4}; // 4 indicates a quarter note

void setup() {
  // Initialize the motor pins as outputs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Set PWM pins as outputs
  pinMode(enAPin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
}

//  A function when the line follower leaves the line pero di pani working 
void motor_drive(int leftSpeed, int rightSpeed) {
    // Set motor directions based on speed signs
    if (leftSpeed > 0) {
        digitalWrite(leftMotorPin1, HIGH);
        digitalWrite(leftMotorPin2, LOW);
    } else {
        digitalWrite(leftMotorPin1, LOW);
        digitalWrite(leftMotorPin2, HIGH);
    }

    if (rightSpeed > 0) {
        digitalWrite(rightMotorPin1, HIGH);
        digitalWrite(rightMotorPin2, LOW);
    } else {
        digitalWrite(rightMotorPin1, LOW);
        digitalWrite(rightMotorPin2, HIGH);
    }

    // Set the motor speeds using PWM
    analogWrite(enAPin, abs(leftSpeed));
    analogWrite(enBPin, abs(rightSpeed));
}


void loop() {
  // Read the sensor values
  int leftSensorValue = analogRead(leftSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  // Calculate the error (difference between sensor readings)
  int error = rightSensorValue - leftSensorValue;
  playMelody();
  // Check if both sensors are not on the line
  if (leftSensorValue >= 970 && rightSensorValue >= 970) {
    // A case when the line follower leaves the line
    if (error > 0) {
      // Turn left if the line was to the left before
      motor_drive(-150, 150);  // Adjust the motor speed values as needed
    } else {
      // Else turn right
      motor_drive(150, -150);  // Adjust the motor speed values as needed
    }
  } else {
    // Calculate the motor speeds using proportional control
    int leftMotorSpeed = 100 + Kp * error;
    int rightMotorSpeed = 100 - Kp * error;

    // Make sure motor speeds are within limits
    leftMotorSpeed = constrain(leftMotorSpeed, 40, 120); //never change this value
    rightMotorSpeed = constrain(rightMotorSpeed, 40, 120); //never change this value

    // Set motor directions
    if (leftMotorSpeed > 0) {
      digitalWrite(leftMotorPin1, HIGH);
      digitalWrite(leftMotorPin2, LOW);
    } else {
      digitalWrite(leftMotorPin1, LOW);
      digitalWrite(leftMotorPin2, HIGH);
    }

    if (rightMotorSpeed > 0) {
      digitalWrite(rightMotorPin1, HIGH);
      digitalWrite(rightMotorPin2, LOW);
    } else {
      digitalWrite(rightMotorPin1, LOW);
      digitalWrite(rightMotorPin2, HIGH);
    }

    // Set the motor speeds using PWM
    analogWrite(enAPin, abs(leftMotorSpeed));
    analogWrite(enBPin, abs(rightMotorSpeed));
  }
}
void playMelody() {
  // Iterate through the melody array and play each note
  for (int i = 0; i < 8; i++) {
    int noteDuration = 1000 / noteDurations[i];
    tone(buzzerPin, melody[i], noteDuration);
    delay(noteDuration * 1.30); // Add a small delay between notes
    noTone(buzzerPin); // Stop the tone
    delay(50); // Add a short pause between notes
  }
  delay(1000); // Delay between melody repetitions
}