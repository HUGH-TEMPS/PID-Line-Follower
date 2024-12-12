#include <QTRSensors.h>

#define IN1 9    // Motor 1 forward control pin
#define IN2 6   // Motor 1 reverse control pin
#define IN3 3   // Motor 2 forward control pin
#define IN4 5   // Motor 2 reverse control pin
#define PWMA 10   // Motor 1 speed (PWM) pin
#define PWMB 11   // Motor 2 speed (PWM) pin

const int offsetA = 1;
const int offsetB = 1;
const int startButtonPin = 2; // Replace with your actual start button pin
int buttonState = 0;
//int lastButtonState = 0;
const int ledPin = 7;
QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 2.5;
float Ki = 0.01;
float Kd = 100;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 100;

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1}, SensorCount);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(500);
  Serial.begin(9600);
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 2 sensors
  // * 10 reads per calibrate() call = ~8 ms per calibrate() call.
  // Call calibrate() 1250 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 1000; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(ledPin, LOW);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    //pinMode(startButtonPin, INPUT_PULLUP);
    
}

void loop() {
  robot_control();
}
void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 2000 (for a white line, use readLineWhite() instead)
  position = qtr.readLineBlack(sensorValues);
  error = 500 - position;
  while(sensorValues[0]>=900 && sensorValues[1]>=900 ){ // A case when the line follower leaves the line
    if(previousError>0){        //Turn left if the line was to the left before
      motor_drive(-120,120);
    }
    else{
      motor_drive(120,-120); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }
  
  PID_Linefollow(error);
  //PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    motor_drive(lsp,rsp);
}

void motor_drive(int leftSpeed, int rightSpeed) {
    // Control left motor direction
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    // Control right motor direction
    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }

    // Control speed (using absolute value in case of negative speed)
    analogWrite(PWMA, abs(leftSpeed));
    analogWrite(PWMB, abs(rightSpeed));
}
