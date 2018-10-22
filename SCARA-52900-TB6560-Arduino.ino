#include <MultiStepper.h>
#include <AccelStepper.h>

String processing_position;
int ledPin = 13;
const int stepperElbowStep = 8;
const int stepperElbowDir = 9;
const int maxElbow = 4000;
int isElbowHomed = false;

const int stepperShoulderStep = 12;
const int stepperShoulderDir = 13;
const int maxShoulder = 6320;
int isShoulderHomed = false;


const int stepperWaistStep = 10;
const int stepperWaistDir = 11;
const int maxWaist = 2000;//need to discover actual value
int waistHomeOscillation = 40;
int isWaistHomed = false;

const int stepperWristStep = 6;
const int stepperWristDir = 7;


int posShoulder = 0;
int pos2 = -10000;

const int limitElbowL = A0; //left
const int limitElbowR = A1; //right

const int limitShoulderT = A2; //top
const int limitShoulderB = A3; //bottom

const int limitWaist = A4; // only one sensor for waist as it is 360 degrees rotation.



// Define two steppers and the pins they will use
AccelStepper stepperShoulder(AccelStepper::DRIVER, stepperShoulderStep, stepperShoulderDir);
AccelStepper stepperElbow(AccelStepper::DRIVER, stepperElbowStep, stepperElbowDir); //driver, step, direction
AccelStepper stepperWaist(AccelStepper::DRIVER, stepperWaistStep, stepperWaistDir); //driver, step, direction
AccelStepper stepperWrist(AccelStepper::DRIVER, stepperWristStep, stepperWristDir); //driver, step, direction
void setup()
{
  Serial.begin(9600);
  stepperShoulder.setMaxSpeed(3000);
  stepperShoulder.setAcceleration(500);

  stepperElbow.setMaxSpeed(3000);
  stepperElbow.setAcceleration(500);

  stepperWaist.setMaxSpeed(3000);
  stepperWaist.setAcceleration(500);

  stepperWrist.setMaxSpeed(200);
  stepperWrist.setAcceleration(1000);

  //PIN MODES
  pinMode(limitElbowL , INPUT_PULLUP);
  pinMode(limitElbowR , INPUT_PULLUP);
  pinMode(stepperElbowStep, OUTPUT);
  pinMode(stepperElbowDir, OUTPUT);


  pinMode(limitShoulderT, INPUT_PULLUP);
  pinMode(limitShoulderB, INPUT_PULLUP);
  pinMode(stepperShoulderStep, OUTPUT);
  pinMode(stepperShoulderDir, OUTPUT);

  pinMode(limitWaist, INPUT_PULLUP);
  pinMode(stepperWaistStep, OUTPUT);
  pinMode(stepperWaistDir, OUTPUT);


  //initial values
  digitalWrite(stepperElbowDir, LOW);
  digitalWrite(stepperShoulderDir, LOW);
  digitalWrite(stepperWaistDir, LOW);

  homeAll();
}

void loop()
{
    elbowLimitsCheck();
    stepperElbow.run();

    shoulderLimitsCheck();
    stepperShoulder.run();

    waistLimitCheck();
    stepperWaist.run();

    if (!stepperElbow.isRunning() && !stepperShoulder.isRunning() && isElbowHomed == true && isShoulderHomed == true && isWaistHomed == false && stepperWaist.isRunning() == false) { // these have been homed.
     waistHome();
    }

    //all homing done.
    if (!stepperElbow.isRunning() && !stepperShoulder.isRunning() && !stepperWaist.isRunning() && isElbowHomed == true && isShoulderHomed == true && isWaistHomed == true) { // these have been homed.
     //stepperWrist.setSpeed(4100);
     //stepperWrist.runSpeed();

    }
    if(isElbowHomed == true && isShoulderHomed == true && isWaistHomed == true){
      //TAKE POSITION FROM PROCESSING APP.
      if (Serial.available()) { // If data is available to read,
        processing_position = Serial.readStringUntil('\n'); // read it and store it in val
        //processing_position = "A:50:B:50";
        String waistVal = getValue(processing_position, ':', 1); 
        String elbowVal = getValue(processing_position, ':', 3);
        //Serial.print("waistval");
        //Serial.println(waistVal);
        //Serial.print("elbowVal");
        //Serial.println(elbowVal);
        stepperWaist.moveTo(waistVal.toInt());
        stepperWaist.run();
        stepperElbow.moveTo(elbowVal.toInt());
        stepperElbow.run();
        delay(10); // Wait 10 milliseconds for next reading
      }

     
    }
  

  
}

void elbowHome() {
  Serial.println("Homing Elbow...");
  stepperElbow.moveTo(-5000);
}

void shoulderHome() {
  Serial.println("Homing Shoulder...going down....");
  stepperShoulder.moveTo(-15000);
  stepperShoulder.run();
}

void waistHome() {
  Serial.println("Homing Waist.");
  stepperWaist.moveTo(-waistHomeOscillation);
  stepperWaist.run();
  waistHomeOscillation = -waistHomeOscillation * 2;
}

void elbowLimitsCheck() {
  //stepperElbow.setMaxSpeed(2000);
  //stepperElbow.setAcceleration(200);
  if (digitalRead(limitElbowL) == HIGH) {
    Serial.println("Left Elbow Limit hit....");
    stepperElbow.setCurrentPosition(0);
    int elbowHome = 0;
    while (digitalRead(limitElbowL)) {
      stepperElbow.moveTo(elbowHome);
      stepperElbow.run();
      elbowHome++;
      delay(5);
      Serial.println("Nudging Elbow Home");
    }
    stepperElbow.setCurrentPosition(0);
    pos2 = maxElbow / 2;
    stepperElbow.moveTo(pos2);
    stepperElbow.runToPosition();
    stepperElbow.setCurrentPosition(0);
    stepperElbow.moveTo(-1800);
    isElbowHomed = true;
    //stepperElbow.moveTo(0);
  }

  if (digitalRead(limitElbowR) == HIGH && isElbowHomed == true) {
    if (stepperElbow.currentPosition() < 3900) {
      Serial.println("False Right Switch is Suspect....");
    } else {
      Serial.print("Right Elbow Limit hit at ");
      Serial.println( stepperElbow.currentPosition());
      Serial.println("Going Home.");
      //should never hit right if homed to left....start over and home
      //stepperElbow.setCurrentPosition(0);
      stepperElbow.moveTo(0);
    }
  }
}

void shoulderLimitsCheck() {
  if (isShoulderHomed == false) {
    //stepperShoulder.setMaxSpeed(500);
    //stepperShoulder.setAcceleration(500);
    if (digitalRead(limitShoulderB) == HIGH) {
      Serial.println("Bottom Shoulder Limit hit. Moving to maxShoulder.");
      stepperShoulder.setCurrentPosition(0);
      int shoulderHome = 2;
      while (digitalRead(limitShoulderB)) {
        stepperShoulder.moveTo(shoulderHome);
        stepperShoulder.run();
        shoulderHome++;
        delay(5);
        Serial.println("Nudging Shoulder Home");
      }
      //posShoulder = maxShoulder;
      isShoulderHomed = true;
      stepperShoulder.moveTo(maxShoulder-30);
      Serial.print("Current Shoulder Position:");
      Serial.println(stepperShoulder.currentPosition());
    }

    if (digitalRead(limitShoulderT) == HIGH && isShoulderHomed == true) {
      Serial.print("Top Shoulder Limit hit at ");
      Serial.println( stepperShoulder.currentPosition());
      Serial.println("Sending Shoulder Home.");
      //should never hit top if homed to left....start over and home
      stepperShoulder.moveTo(0);
    }
  }
}

void waistLimitCheck() {

  if (digitalRead(limitWaist) == HIGH && isWaistHomed == false) {
    Serial.println("Waist Shoulder Limit hit. Setting Home.");
    stepperWaist.setCurrentPosition(0);
    isWaistHomed = true;

    Serial.print("Current Waist Position:");
    Serial.println(stepperWaist.currentPosition());


    stepperWaist.moveTo(10000);
    stepperWaist.run();
    delay(2000);
  }

  if(digitalRead(limitWaist) == HIGH && isWaistHomed == true && stepperWaist.currentPosition() > 1000){//checking upper limit
    Serial.print("waist upper limit is: ");
    Serial.println(stepperWaist.currentPosition());
    stepperWaist.moveTo(stepperWaist.currentPosition()/2);
    Serial.println("moving to half/waist");
    stepperWaist.setCurrentPosition(0);
  }
}

void homeAll() {
  shoulderHome();
  elbowHome();
}



String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }     
  
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
