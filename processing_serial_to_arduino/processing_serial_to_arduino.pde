//modified from source found here: https://forum.arduino.cc/index.php?topic=513444.0

import processing.serial.*;
Serial myPort;

import controlP5.*;

ControlP5 cp5;

boolean elbowup = false; // true=elbow up, false=elbow down
boolean mouseCCircle = false;

int Xoffset = 400;
int Yoffset = 400;

//variables
float theta1 = 0.0; // target angles as determined through inverse kinematics
float theta2 = 0.0;

float c2 = 0.0; // is btwn -1 and 1
float s2 = 0.0;

float joint1X;  //Correspond to jointA
float joint1Y;

float joint2X;  //Correspond to jointB
float joint2Y;

int jointAStepPos;
int jointBStepPos;

float a1 = 200; // shoulder-to-elbow "bone" length from Solidworks (mm)
float a2 = 150; // elbow-to-wrist "bone" length from Solidworks (mm) - longer c bracket

int gearA = 20; // Pulley gear ratio motor rev pr. arm rev.
int gearB = 12;
int gearC = 4;
//int gearZ = 30;
int gearZ = 5;//???
//if the visual does not match the movement of the robot, probably your microstep values need to be adjusted. try 2, 4, 8 or 16.
int microStepA = 2; // stepper drivers microstepping setting puls pr. step 
int microStepB = 2;
int microStepC = 2;
int microStepZ = 2;
int motorStepA = 200;   // Motor step pr. rev.
int motorStepB = 200;
int motorStepC = 200;
int motorStepZ = 200;

int posA;
int posB;
int posC;
int posZ;



void setup() {
  String portName = Serial.list()[1]; //change the 0 to a 1 or 2 etc. to match your port
  println(Serial.list());
  myPort = new Serial(this, portName, 9600);

  PFont font = createFont("arial", 20);

  cp5 = new ControlP5(this);

  cp5.addTextfield("           Serial")
    .setPosition(800, 700)
    .setSize(200, 40)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 0, 0))
    ;


  size(1200, 800);
  noSmooth();
  background(0);

  // Draw gray box
  stroke(200);

  stroke(200, 50);


  drawArm();
}

void draw() {
  calculate();
}

void mouseClicked() {
  if (mouseX < 800) {
    mouseCCircle = true;
  }
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  if (e>0) {
    elbowup = true;
    println("elbow: true");
  } else {
    elbowup = false;
    println("elbow: false");
  }
}

void calculate() {
  if (mouseCCircle) {
    get_angles(mouseX-Xoffset, -mouseY+Xoffset);

    get_xy();
    println("mousex: " + (mouseX-Xoffset) + "  mouseY: " + (-mouseY+Xoffset));

    drawArm();
    armCalc();
    mouseCCircle = false;
  }
}

void armCalc() {
  float OneDegToStepA = ((gearA*microStepA*motorStepA)/360);
  float OneDegToStepB = ((gearB*microStepB*motorStepB)/360);

  posA = int(OneDegToStepA*-theta1);
  posB = int(OneDegToStepB*-theta2);

  text(posA, 0, 20);
  text(posB, joint1X, joint1Y+20);

  String Position = "A:"+(-posA)+ ":B:"+posB+"\n"  ;

  text(Position, 50, -50);
  println(Position);
  myPort.write(Position);
}

float Angle360Converter(float angle) {
  if (angle < 0) {
    angle = angle + 360;
  }
  return angle;
}

void drawArm() {
  background(0);
  ellipse(0, 0, 10, 10);
  noFill();
  stroke(200, 50);
  translate(Xoffset, Yoffset);  //Offset to get into the center of our drawing

  stroke(200);
  line(0, 0, joint1X, joint1Y);  //Draw arm shoulder to elbow
  stroke(255, 204, 0);
  line(joint1X, joint1Y, joint2X, joint2Y);  //Draw arm elbow to wrist

  stroke(200, 123, 19);  
  ellipse(0, 0, 700, 700); //outer limit cricle
  ellipse(0, 0, 200, 200); //inner limit cricle

  text(theta1, 0, 0);  //write the angle of the first joint
  text(theta2, joint1X, joint1Y);  //write the angle of the second joint
}

// Given target(Px, Py) solve for theta1, theta2 (inverse kinematics)
void get_angles(float Px, float Py) {
  // first find theta2 where c2 = cos(theta2) and s2 = sin(theta2)
  c2 = (pow(Px, 2) + pow(Py, 2) - pow(a1, 2) - pow(a2, 2))/(2*a1*a2); // is btwn -1 and 1

  if (elbowup == false) {
    s2 = sqrt(1 - pow(c2, 2)); // sqrt can be + or -, and each corresponds to a different orientation
  } else if (elbowup == true) {
    s2 = -sqrt(1 - pow(c2, 2));
  }
  theta2 = degrees(atan2(s2, c2)); // solves for the angle in degrees and places in correct quadrant

  // now find theta1 where c1 = cos(theta1) and s1 = sin(theta1)
  theta1 = degrees(atan2(-a2*s2*Px + (a1 + a2*c2)*Py, (a1 + a2*c2)*Px + a2*s2*Py));
}

void get_xy() {
  joint1X = a1*cos(radians(theta1));
  joint1Y = -(a1*sin(radians(theta1)));

  joint2X = a1*cos(radians(theta1)) + a2*cos(radians(theta1+theta2));
  joint2Y = -(a1*sin(radians(theta1)) + a2*sin(radians(theta1+theta2)));
}

public void clear() {
  //cp5.get(Textfield.class, "textValue").clear();
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isAssignableFrom(Textfield.class)) {
  myPort.write(theEvent.getStringValue());
    println("From Luke: controlEvent: accessing a string from controller '"
            +theEvent.getName()+"': "
            +theEvent.getStringValue()
           );
  }
}
