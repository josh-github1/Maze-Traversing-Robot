
/* FINAL CODE
  Matt Russell, Joshua Ramayrat
  This is the code that will guide our robot through the obstacle course.
*/
#include <QTRSensors.h>

#define MINGUESS 5000
//how many samples to take for calibration
#define CALNUM 100
//how long to wait between calibration
#define CALWAIT 1000
//basespeed of motors
#define BASESPEED 50
//how many data points to take for line-following
#define FOLNUM 10
//arbitrary constant for line following
#define ARBC 500


int stage = 1;
int goPin = 12, swPin = 30, readyPin = 26, statusPin = 24;
//QTR Sensor Calibration
//Pins
int QTRLEDon = 22, QTRmod1 = 41, QTRmod2 = 42, QTRmod3 = 43, QTRmod4 = 44, QTRmod5 = 45, QTRmod6 = 46, QTRmod7 = 47, QTRmod8 = 48;
//Structures
QTRSensorsRC qtrrc((unsigned char[]) {
  41, 42, 43, 44, 45, 46, 47, 48
}, 8);

/*unsigned int maxVal[] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int minVal[] = {MINGUESS, MINGUESS, MINGUESS, MINGUESS, MINGUESS, MINGUESS, MINGUESS, MINGUESS};
  volatile unsigned int wMin[] = {0, 0, 0, 0, 0, 0, 0, 0};
  volatile unsigned int wMax[] = {0, 0, 0, 0, 0, 0, 0, 0};
  volatile unsigned int wAvg[] = {0, 0, 0, 0, 0, 0, 0, 0};
  volatile unsigned int bMin[] = {0, 0, 0, 0, 0, 0, 0, 0};
  volatile unsigned int bMax[] = {0, 0, 0, 0, 0, 0, 0, 0};
  volatile unsigned int bAvg[] = {0, 0, 0, 0, 0, 0, 0, 0};*/
unsigned int sensorValues[8];  /*sensor1[100], sensor2[100], sensor3[100], sensor4[100], sensor5[100], sensor6[100], sensor7[100], sensor8[100];*/
//avg[] = {lValavg, rValavg};
//arrays for calibration
long avg[] = {0, 0}, blackVal[8], whiteVal[8];


//Motors
// Left motor
int enLeft = 5, in1 = 6, in2 = 7;
// Right motor
int enRight = 10, in3 = 8, in4 = 9;
//Encoder
long count = 0;
int encLeft = 3, encRight = 4;
//Line-following
unsigned int stage1_spd = BASESPEED, stage2_spd = 70, stage4_spd = stage2_spd + 20;
int lVal, rVal;
long threshold, sensorSum = 0, allBlack = 0, allWhite = 0;
int diff[CALNUM];
//Stepper Motor
int stepPin = 34, dirPin = 35, trig = LOW;



void setup() {
  digitalWrite(statusPin, LOW);
  //Initialize Serial Console only for debugging
  Serial.begin(9600);
  //Setup for calibration
  pinMode(QTRmod1, INPUT);
  pinMode(QTRmod2, INPUT);
  pinMode(QTRmod3, INPUT);
  pinMode(QTRmod4, INPUT);
  pinMode(QTRmod5, INPUT);
  pinMode(QTRmod6, INPUT);
  pinMode(QTRmod7, INPUT);
  pinMode(QTRmod8, INPUT);
  pinMode(readyPin, OUTPUT);
  pinMode(statusPin, OUTPUT);
  pinMode(goPin, OUTPUT);
  pinMode(QTRLEDon, OUTPUT);
  //Setup for line-following
  pinMode(swPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enLeft, OUTPUT);
  pinMode(enRight, OUTPUT);
  pinMode(encLeft, INPUT);
  pinMode(encRight, INPUT);
  //Setup for button
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  //calibrate for the line
  digitalWrite(statusPin, HIGH);
  while (digitalRead(swPin) != LOW) {
  }
  digitalWrite(statusPin, LOW);
  digitalWrite(QTRLEDon, HIGH);
  lCal();
  digitalWrite(statusPin, HIGH);
  rdelay(CALWAIT);
  //calibrate for black
  while (digitalRead(swPin) !=  LOW) {
  }
  digitalWrite(statusPin, LOW);
  bCal();
  digitalWrite(statusPin, HIGH);
  rdelay(CALWAIT);
  //Calibrate for white
  while (digitalRead(swPin) != LOW) {
  }
  digitalWrite(statusPin, LOW);
  wCal();
  //Ready to begin
  digitalWrite(readyPin, HIGH);
  while (digitalRead(swPin) != LOW) {
  }
  digitalWrite(readyPin, LOW);
  setLeft(BASESPEED, 0);
  setRight(BASESPEED + 15, 0);
  delay(700);
  setLeft(BASESPEED, 1);
  setRight(BASESPEED + 15, 1);
  delay(700);
}

void loop() {
  if (stage == 1) {
    follow_stg1();
    if (sensorSum > allBlack - 4000) {
      Brake();
      stage = 2;
    }
  }
  if (stage == 2) {
    rdelay(500);
    freespace_stg2();
    stage = 3;
  }
  if (stage == 3) {
    findLine();
    Brake();
    rdelay(250);
  }
  if (stage == 4) {
    /*follow_stg4();
      if (sensorSum > 20000) {
      Brake();*/
    stage = 5;
  }
  if (stage == 5) {
    follow_stg4();
    if (sensorSum > allBlack - 15000) {
      Brake();
      rdelay(500);
      setLeft(stage1_spd, 1);
      setRight(stage1_spd + 10, 1);
      stage = 6;
    }
  }
  if (stage == 6) {
    stage1_spd = BASESPEED - 10;
    follow_stg6();
    if (sensorSum > allBlack - 4000) {
      Brake();
      stage = 7;
    }
  }
  if (stage == 7){
    rdelay(1000);
    Forward(BASESPEED);
    rdelay(250);
    Brake();
    stage = 8;
  }
}

//CALIBRATION
//Calibration for line
void lCal() {
  for (int i = 0; i < CALNUM; i++) {
    lVal = 0, rVal = 0;
    qtrrc.read(sensorValues);
    for (int j = 0; j < 3; j++) {
      lVal = lVal + sensorValues[j + 4];
      rVal = rVal + sensorValues[j + 1];
    }
    diff[i] = lVal - rVal;
    threshold = threshold + diff[i];
  }
  threshold = threshold / CALNUM;
  Serial.println(threshold);
}

//Calibration for black, gives us value when each sensor is looking at black
void bCal() {
  for (int i = 0; i < 8; i ++) {
    blackVal[i] = 0;
  }
  for (int i = 0; i < CALNUM; i++) {
    qtrrc.read(sensorValues);
    for (int j = 0; j < 8; j++) {
      blackVal[j] = blackVal[j] + sensorValues[j];
    }
  }
  for (int i = 0; i < 8; i++) {
    blackVal[i] = blackVal[i] / CALNUM;
    Serial.println(blackVal[i]);
    allBlack = allBlack + blackVal[i];
  }
}

//Calibration for white, gives us value when each sensor is looking at white
void wCal() {
  for (int i = 0; i < 8; i ++) {
    whiteVal[i] = 0;
  }
  for (int i = 0; i < CALNUM; i++) {
    qtrrc.read(sensorValues);
    for (int j = 0; j < 8; j++) {
      whiteVal[j] = whiteVal[j] + sensorValues[j];
    }
  }
  for (int i = 0; i < 8; i++) {
    whiteVal[i] = whiteVal[i] / CALNUM;
    Serial.println(whiteVal[i]);
    allWhite = allWhite + whiteVal[i];
  }
}

//LINE-FOLLOWING FUNCTIONS
//Function to compute sensor values
void follow_stg1() {
  //lVal for 5-7, rVal for 1-3
  lVal = 0, rVal = 0;
  qtrrc.read(sensorValues);
  for (int i = 0; i < 3; i++) {
    lVal = lVal + sensorValues[i + 4];
    rVal = rVal + sensorValues[i + 1];
  }
  //Serial.println(rVal - lVal);
  //check if drifting right
  if (lVal > rVal) {
    //Serial.println("Drifting Right");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, 0);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd + 10);
  }
  //check if drifting left
  if (rVal > lVal) {
    //Serial.println("Drifting Left");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd + 10);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, 0);
  }
  //drifting 'straight'
  if (lVal - rVal < threshold + ARBC && lVal - rVal > threshold - ARBC) {
    //Serial.println("Drifting Straight");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd);
  }
  //Exception handling
  //robot needs to turn to its right
  if (rVal > blackVal[1] + blackVal[2] + blackVal[3] - 1500) {
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd);
    //Right motor
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enRight, stage1_spd + 60);
  }
  //Left Turn
  if ((lVal > blackVal[4] + blackVal[5] + blackVal[6] - 1500)) {
    //Left motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enLeft, stage1_spd + 60);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd);
  }
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    sensorSum = sensorSum + sensorValues[i];
  }
}

void follow_stg4() {
  lVal = 0, rVal = 0;
  qtrrc.read(sensorValues);
  for (int i = 0; i < 3; i++) {
    lVal = lVal + sensorValues[i + 4];
    rVal = rVal + sensorValues[i + 1];
  }
  //Serial.println(rVal - lVal);
  //check if drifting right
  if (lVal > rVal) {
    //Serial.println("Drifting Right");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage4_spd - 35);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage4_spd);
  }
  //check if drifting left
  if (rVal > lVal) {
    //Serial.println("Drifting Left");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage4_spd);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage4_spd - 35);
  }
  //drifting 'straight'
  if (lVal - rVal < threshold + ARBC && lVal - rVal > threshold - ARBC) {
    //Serial.println("Drifting Straight");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage4_spd);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage4_spd);
  }
  //Exception handling
  //robot needs to turn to its right
  if (rVal > blackVal[1] + blackVal[2] + blackVal[3] - 1500) {
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage4_spd);
    //Right motor
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enRight, stage4_spd + 60);
  }
  //Left Turn
  if ((lVal > blackVal[4] + blackVal[5] + blackVal[6] - 1500)) {
    //Left motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enLeft, stage4_spd + 60);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage4_spd);
  }
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    sensorSum = sensorSum + sensorValues[i];
  }
}

void follow_stg6() {
  //lVal for 5-7, rVal for 1-3
  lVal = 0, rVal = 0;
  qtrrc.read(sensorValues);
  for (int i = 0; i < 3; i++) {
    lVal = lVal + sensorValues[i + 4];
    rVal = rVal + sensorValues[i + 1];
  }
  //Serial.println(rVal - lVal);
  //check if drifting right
  if (lVal > rVal) {
    //Serial.println("Drifting Right");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, 0);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd);
  }
  //check if drifting left
  if (rVal > lVal) {
    //Serial.println("Drifting Left");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, 0);
  }
  //drifting 'straight'
  if (lVal - rVal < threshold + ARBC && lVal - rVal > threshold - ARBC) {
    //Serial.println("Drifting Straight");
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd);
  }
  //Exception handling
  //robot needs to turn to its right
  if (rVal > blackVal[1] + blackVal[2] + blackVal[3] - 1500) {
    //Left motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enLeft, stage1_spd);
    //Right motor
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enRight, stage1_spd + 60);
  }
  //Left Turn
  if ((lVal > blackVal[4] + blackVal[5] + blackVal[6] - 1500)) {
    //Left motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enLeft, stage1_spd + 60);
    //Right motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enRight, stage1_spd);
  }
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    sensorSum = sensorSum + sensorValues[i];
  }
}

//'FREE-SPACE" FUNCTIONS
void freespace_stg2() {
  setRight(stage1_spd + 25, 1);
  setLeft(stage1_spd, 1);
  rdelay(1000);
  setLeft(stage1_spd + 15, 1);
  setRight(stage1_spd, 1);
  rdelay(200);
  stage = 3;
}

void findLine() {
  //assumes we overshoot most of the time
  qtrrc.read(sensorValues);
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    sensorSum = sensorSum + sensorValues[i];
  }
  //line found!
  if (sensorSum > allWhite + 500) {
    stage = 4;
  }
  else {
    rdelay(250);
    Brake();
    rdelay(500);
    setLeft(stage2_spd, 1);
    setRight(stage2_spd + 20, 1);
    rdelay(250);
    Brake();
    rdelay(250);
  }
}

//MOTOR FUNCTIONS
void Forward(int spd) {
  //Left motor
  setLeft(spd, 1);
  //Right motor
  setRight(spd, 1);
}

void Reverse(int spd) {
  //Left motor
  setLeft(spd, 0);
  //Right motor
  setRight(spd, 0);
}

void Brake() {
  brakeLeft();
  brakeRight();
}

void PivotRight(int spd) {
  setLeft(spd, 0);
  setRight(spd, 1);
}

void PivotLeft(int spd) {
  setLeft(spd, 1);
  setRight(spd, 0);
}

void TurnInPlace(int spd) {
  //Turn in place to the right, right is Reverse, left is Forward
  setRight(spd, 0);
  setLeft(spd, 1);
}

//MOTOR FUNCTION BUILDING BLOCKS
//Sets left motor
void setLeft(int spd, int dir) {
  //0 for Reverse, 1 for Forward, 2 for Brake
  //reverse
  if (dir == 0) {
    setLeftReverse();
  }
  //forward
  if (dir == 1) {
    setLeftForward();
  }
  //brake
  if (dir == 2) {
    brakeLeft();
  }
  //left motor runs a little slower than right naturally
  analogWrite(enLeft, spd + 25);
}

//Sets right motor
void setRight(int spd, int dir) {
  //0 for Reverse, 1 for Forward, 2 for Brake
  //reverse
  if (dir == 0) {
    setRightReverse();
  }
  //forward
  if (dir == 1) {
    setRightForward();
  }
  //brake
  if (dir == 2) {
    brakeRight();
  }
  analogWrite(enRight, spd);
}

void setLeftForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void setRightForward() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void setLeftReverse() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void setRightReverse() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void brakeLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void brakeRight() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void stepper(){
 digitalWrite(dirPin,trig);
 for(int i=0; i<250; i++){
  digitalWrite(13,HIGH);
  delay(1);
  digitalWrite(13, LOW);
  delay(1);
  }
  delay(1000);
  trig = !trig;
 digitalWrite(dirPin,trig);
 for(int i=0; i<100; i++){
  digitalWrite(stepPin,HIGH);
  delay(1);
  digitalWrite(stepPin, LOW);
  delay(1);
  }
}


//General-purpose functions
void rdelay(int dt) {
  int ct, pt;
  pt = millis();
  ct = pt;
  while (ct - pt < dt) {
    ct = millis();
  }
}
