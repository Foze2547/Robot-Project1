#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// Create an object for controlling the servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Create BluetoothSerial object
BluetoothSerial SerialBT;

// Define constants for servo pulse values
#define servoMin  150   // Minimum pulse value for the servo
#define servoMax  1000  // Maximum pulse value for the servo

// Define Bluetooth commands for driving and arm control
#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

// Define motor pins
#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 0
#define MOTOR2_PIN1 17
#define MOTOR2_PIN2 16
#define MOTOR3_PIN1 15
#define MOTOR3_PIN2 2
#define MOTOR4_PIN1 18
#define MOTOR4_PIN2 19

Servo servo1;
Servo servo2;
Servo servo3;
Servo baseServo;

// Control mode variable (false = Drive Mode, true = Arm Mode)
bool isArmMode = false;

// Variables to store the current angle of the servos
//int currentServoAngle1 = 90;  // Servo channel 1
int currentServoAngle2 = 90;  // Servo channel 2
int currentServoAngle3 = 90;  // Servo channel 3
int baseServoAngle = 90;       // Base servo initial angle (centered at 90 degrees)
int Speed = 150;               // Motor speed

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);
  
  // Set motor pins as output
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);
  pinMode(MOTOR4_PIN1, OUTPUT);
  pinMode(MOTOR4_PIN2, OUTPUT);

  // Start Bluetooth serial communication
  SerialBT.begin("MotorControl111"); // Name of the Bluetooth device
  Serial.println("Bluetooth Device is Ready to Pair");

  //servo1.attach(27);  // ขาควบคุม Servo 1
  servo2.attach(26);  // ขาควบคุม Servo 2
  servo3.attach(25);  // ขาควบคุม Servo 3
  baseServo.attach(33); // ขาควบคุม Base Servo
  //controlServoDegree(1, 90);
  controlServoDegree(2, 90);
  controlServoDegree(3, 90);
  controlServoDegree(4, 90);
}

void loop() {
  // Check if Bluetooth data is available to read
  if (SerialBT.available()) {
    char command = SerialBT.read(); // Read the incoming command
    Serial.println(command); // Print the command for debugging

    // Handle commands based on the current mode (Drive or Arm)
    switch (command) {
      case START:  // Switch to Arm Control Mode
        isArmMode = true;
        Serial.println("Switched to Arm Control Mode");
        break;

      case PAUSE:  // Switch to Drive Control Mode
        isArmMode = false;
        Serial.println("Switched to Drive Control Mode");
        break;
      
      case FORWARD:  // Move Arm X Forward or Drive Forward
        isArmMode ? moveArmXForward() : driveForward();
        break;

      case BACKWARD:  // Move Arm X Backward or Drive Backward
        isArmMode ? moveArmXBackward() : driveBackward();
        break;

      case TRIANGLE:  // Move Arm Y Forward (increase servo angle)
        isArmMode ? Jointdown() : BackturnLeft();
        break;

      case CROSS:  // Decrease servo angle by 3 degrees
        isArmMode ? Jointup() : BackturnRight();
        break;

      case CIRCLE:  // Increase the angle of both servos by 3 degrees
        isArmMode ? Gripperout() : rotateBaseRight();
        break;

      case SQUARE:  // Decrease the angle of both servos by 3 degrees
        isArmMode ? Gripperin() : rotateBaseLeft();
        break;

      case LEFT:  // Turn Left or move Arm Y Forward
        isArmMode ? moveArmYForward() : turnLeft();
        break;

      case RIGHT:  // Turn Right or move Arm Y Backward
        isArmMode ? moveArmYBackward() : turnRight();
        break;

      case '0':  // Stop all motors and arm movement
        stopMotors();
        stopmoveArm();
        break;

      default:  // Handle any undefined command
        Serial.println("Invalid command received");
        break;
    }
  }
}

// ------------------- Motor Control Functions -------------------

// Function to drive motors forward
void driveForward() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
  Serial.println("Motors moving forward");
}

// Function to drive motors backward
void driveBackward() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
  Serial.println("Motors moving backward");
}

// Function to turn left
void turnLeft() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);  // Stop left motor
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH); // Run right motor forward
  Serial.println("Turning left");
}

// Function to turn right
void turnRight() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH); // Run left motor forward
  digitalWrite(MOTOR2_PIN1, LOW);  // Stop right motor
  digitalWrite(MOTOR2_PIN2, LOW);
  Serial.println("Turning right");
}

// Function to turn left
void BackturnRight() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);  // Stop left motor
  digitalWrite(MOTOR2_PIN1, HIGH); // Run right motor forward
  digitalWrite(MOTOR2_PIN2, LOW);
  Serial.println("BackTurning right");
}

// Function to turn right
void BackturnLeft() {
  digitalWrite(MOTOR1_PIN1, HIGH); // Run left motor forward
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);  // Stop right motor
  digitalWrite(MOTOR2_PIN2, LOW);
  Serial.println("BackTurning left");
}

// Function to stop motors
void stopMotors() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  Serial.println("Motors stopped");
}

// Rotate base to the left
void rotateBaseLeft() {
  if (baseServoAngle > 40)
    controlServoDegree(4, baseServoAngle - 10);
}

// Rotate base to the right
void rotateBaseRight() {
  if (baseServoAngle < 140)
    controlServoDegree(4, baseServoAngle + 10);
}

// ------------------- Arm Control Functions -------------------

// Function to move Arm X forward
void moveArmXForward() {
  digitalWrite(MOTOR3_PIN1, Speed);
  digitalWrite(MOTOR3_PIN2, LOW);
  Serial.println("Arm X moving forward");
}

// Function to move Arm X backward
void moveArmXBackward() {
  digitalWrite(MOTOR3_PIN1, LOW);
  digitalWrite(MOTOR3_PIN2, Speed);
  Serial.println("Arm X moving backward");
}

// Function to move Arm Y forward
void moveArmYForward() {
  digitalWrite(MOTOR4_PIN1, Speed);
  digitalWrite(MOTOR4_PIN2, LOW);
  Serial.println("Arm Y moving forward");
}

// Function to move Arm Y backward
void moveArmYBackward() {
  digitalWrite(MOTOR4_PIN1, LOW);
  digitalWrite(MOTOR4_PIN2, Speed);
  Serial.println("Arm Y moving backward");
}

// Function to stop arm movement
void stopmoveArm() {
  digitalWrite(MOTOR3_PIN1, LOW);
  digitalWrite(MOTOR3_PIN2, LOW);
  digitalWrite(MOTOR4_PIN1, LOW);
  digitalWrite(MOTOR4_PIN2, LOW);
  Serial.println("Arm stopped");
}

// Function to control the servo to a specified degree
void controlServoDegree(int channel, int targetDegree) {
  // ทำให้ค่าองศาอยู่ระหว่าง 0 ถึง 180
  targetDegree = constrain(targetDegree, 0, 180);
  
  int currentDegree = //(channel == 1) ? currentServoAngle1 :
                      (channel == 2) ? currentServoAngle2 :
                      (channel == 3) ? currentServoAngle3 :
                      baseServoAngle;

  int step = 1;           // จำนวนองศาที่เคลื่อนที่ต่อครั้ง
  int delayTime = 20;     // หน่วงเวลาในแต่ละขั้น (มิลลิวินาที)

  if (targetDegree > currentDegree) {
    // เคลื่อนที่จาก currentDegree ไปยัง targetDegree
    for (int degree = currentDegree; degree <= targetDegree; degree += step) {
      setServoPosition(channel, degree);  // ส่งค่าองศาให้เซอร์โว
      Serial.print("Servo moved to ");
      Serial.print(degree);
      Serial.println(" degrees");
      delay(delayTime);                   // หน่วงเวลา
    }
  } else {
    // เคลื่อนที่จาก currentDegree ไปยัง targetDegree ในทิศทางตรงกันข้าม
    for (int degree = currentDegree; degree >= targetDegree; degree -= step) {
      setServoPosition(channel, degree);
      Serial.print("Servo moved to ");
      Serial.print(degree);
      Serial.println(" degrees");
      delay(delayTime);
    }
  }

  // อัพเดตค่าองศาปัจจุบัน
  /*if (channel == 1) currentServoAngle1 = targetDegree;
  else */
  if (channel == 2) currentServoAngle2 = targetDegree;
  else if (channel == 3) currentServoAngle3 = targetDegree;
  else baseServoAngle = targetDegree;
}

// Function to move the arm down
void Jointdown() {
  controlServoDegree(3, currentServoAngle3 + 3);
}

// Function to move the arm up
void Jointup() {
    controlServoDegree(3, currentServoAngle3 - 3);
}

// Function to move the gripper out
void Gripperout() {
  //controlServoDegree(1, currentServoAngle1 + 5);
  controlServoDegree(2, currentServoAngle2 + 10);
}


// Function to move the gripper in
void Gripperin() {
  //controlServoDegree(1, currentServoAngle1 - 5);
  controlServoDegree(2, currentServoAngle2 - 10);
}


void setServoPosition(int channel, int degree) {
  // ฟังก์ชันนี้ใช้สำหรับควบคุมเซอร์โวที่ระบุ
  /*if (channel == 1) servo1.write(degree);
  else */
  if (channel == 2) servo2.write(degree);
  else if (channel == 3) servo3.write(degree);
  else baseServo.write(degree);
}
