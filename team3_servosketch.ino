#include <Servo.h>

Servo servoRight;
Servo servoLeft;

byte pinServoLeft = 3;
byte pinServoRight = 5;
byte pinFeedbackLeft = 2;
byte pinFeedbackRight = 4;

int speedCW = 88;
int speedCCW = 100;
int speedStop = 94;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight = 0, lowLeft = 0;
char keyCommand;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;
int PcontrolRight = 0, PcontrolLeft = 0;
const double wheelPerimeter = 65.0 * 3.14159 / 10.0;  // cm

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller(int dir);
void move(double distance, int speedR, int speedL);
void printInfos();

void setup(){
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0; totalThetaLeft = 0;
}

void loop(){
  if (Serial.available() > 0){
    keyCommand = Serial.read();

    switch(keyCommand){
      case 'f': move(20.0, speedCW, speedCCW); break;
      case 'b': move(20.0, speedCCW, speedCW); break;
      case 's':
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(speedStop);
        servoLeft.write(speedStop);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      case 'p':
        calculateThetaRight();
        calculateThetaLeft();
        printInfos();
        break;
      case 'c':
        for (int i = 0; i < 100; i++) { calculateThetaRight(); calculateThetaLeft(); Serial.print(thetaRight); Serial.print(" "); Serial.println(thetaLeft); delay(50); }
        break;
      case 'g':
        move(200.0, speedCW, speedCCW);
        delay(500);
        move(200.0, speedCCW, speedCW);
        break;
      case 'r': move(7.07, speedCCW, speedCCW); break;   
      case 'l': move(7.07, speedCW, speedCW); break;
      case 'd': move(40.0, speedCW, speedCCW); break;
      default: break;
    }
  }
}

void Pcontroller(int dir){
  calculateThetaRight();
  calculateThetaLeft();
  printInfos();
}

void printInfos(){
  Serial.print("ThetaRight: ");
  Serial.print(thetaRight);
  Serial.print(" , ThetaLeft: ");
  Serial.println(thetaLeft);
  Serial.print("deltaThetaRight: ");
  Serial.print(deltaThetaRight);
  Serial.print(" , deltaThetaLeft: ");
  Serial.println(deltaThetaLeft);
  Serial.print("totalThetaRight: ");
  Serial.print(totalThetaRight);
  Serial.print(" , totalThetaLeft: ");
  Serial.println(totalThetaLeft);
  Serial.print("PcontrolRight: ");
  Serial.print(PcontrolRight);
  Serial.print(" , PcontrolLeft: ");
  Serial.println(PcontrolLeft);
  Serial.print("distRight: ");
  Serial.print(-totalThetaRight * wheelPerimeter / 360.0);
  Serial.print(" , distLeft: ");
  Serial.println(totalThetaLeft * wheelPerimeter / 360.0);
}

void calculateThetaRight(){
  while(1){
    highRight = pulseIn(pinFeedbackRight, HIGH);
    lowRight = pulseIn(pinFeedbackRight, LOW);
    cycleTimeRight = highRight + lowRight;
    if (cycleTimeRight > 950 && cycleTimeRight < 1200) break;
  }
  dutyCycleRight = (highRight * 100.0) / cycleTimeRight;
  thetaRightPrev = thetaRight;
  thetaRight = 359.0 - ((dutyCycleRight - 2.9) * 360.0) / (97.1 - 2.9 + 1);
  deltaThetaRight = thetaRight - thetaRightPrev;
  if (deltaThetaRight > 180)  deltaThetaRight -= 360;
  if (deltaThetaRight < -180) deltaThetaRight += 360;
  totalThetaRight += deltaThetaRight;
}

void calculateThetaLeft(){
  while(1){
    highLeft = pulseIn(pinFeedbackLeft, HIGH);
    lowLeft = pulseIn(pinFeedbackLeft, LOW);
    cycleTimeLeft = highLeft + lowLeft;
    if (cycleTimeLeft > 950 && cycleTimeLeft < 1200) break;
  }
  dutyCycleLeft = (highLeft * 100.0) / cycleTimeLeft;
  thetaLeftPrev = thetaLeft;
  thetaLeft = 359.0 - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1);
  deltaThetaLeft = thetaLeft - thetaLeftPrev;
  if (deltaThetaLeft > 180)  deltaThetaLeft -= 360;
  if (deltaThetaLeft < -180) deltaThetaLeft += 360;
  totalThetaLeft += deltaThetaLeft;
}

void move(double distance, int speedR, int speedL){
  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft = 0;
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);
  servoRight.write(speedR);
  servoLeft.write(speedL);
  while ((fabs(totalThetaRight) + fabs(totalThetaLeft)) / 2.0 * wheelPerimeter / 360.0 < distance) {  
    calculateThetaRight();
    calculateThetaLeft();
    double err = (fabs(totalThetaRight) - fabs(totalThetaLeft)) * wheelPerimeter / 360.0;
    int p = constrain((int)(3.0 * fabs(err)), 0, 5);
    PcontrolRight = 0;
    PcontrolLeft = 0;
    if (err > 0) PcontrolLeft = (speedL > speedStop) ? p : -p;
    else PcontrolRight = (speedR > speedStop) ? p : -p;
    servoRight.write(speedR + PcontrolRight);
    servoLeft.write(speedL + PcontrolLeft);
  }                                                                                                      
  servoRight.detach();
  servoLeft.detach();
  printInfos();
}
