/**
 * Project - Paintball person tracker
 * Name: main
 * Purpose: PID-controller for stepper motors and a trigger servo
 * @author Sebastian Olsson
 * @version 0.1 04/03/2023
*/
#include <AccelStepper.h>
#include <Servo.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(AccelStepper::DRIVER, 15, 14); // (Typeof driver: with 2 pins, STEP, DIR)
Servo triggerServo;

void setup() {

  stepper1.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepper1.setAcceleration(500); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  triggerServo.attach(6);  
  triggerServo.write(0);

}

void loop() {

  stepper1.moveTo(800); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
  triggerServo.write(100);
  // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    //
    //
  }
  triggerServo.write(0);
  delay(500);
}
// #include <Servo.h>
// #include <AccelStepper.h>
// #include <MultiStepper.h>

// /* HARDWARE PINS */
// const uint8_t Y_STEP_PIN = 14; //TBD
// const uint8_t Y_DIR_PIN = 15; //TBD
// const uint8_t X_STEP_PIN = 2; //TBD
// const uint8_t X_DIR_PIN = 3; //TBD
// const uint8_t TRIGGER_SERVO_PIN = 11; //TBD


// /* MOTOR SETTINGS */
// const uint16_t STEPPER_MAX_SPEED = 600;
// const uint8_t STEPS_PER_ROTATION = 200;
// const float DEGREES_PER_STEP = 1.8f;
// long stepperGoToPositions[2]; // [X, Y]

// AccelStepper stepperX;
// AccelStepper stepperY;
// AccelStepper steppers[2] = {stepperX, stepperY};
// MultiStepper steppersController;

// Servo triggerServo;
// enum TriggerServoSettings {
//   FIRE_POS = 180, // half rotation with servo
//   RESET_POS = 0,
//   FIRE_RATE_DELAY = 800, // ms for delay() call. 600?
//   FIRING_THRESHOLD_DEGREES = 5
// };

// /* SERIAL INTERFACE SETTINGS */
// const uint32_t BAUDRATE = 115200; //57600, 115200
// const uint8_t BUFFER_SIZE = 3; // 3 ASCII Characters max (will not send bigger than 3 digit number)
// const uint8_t COMPLETE_MESSAGE_LENGTH = BUFFER_SIZE * 2 + 2;
// char buf[BUFFER_SIZE]; // [X, Y] One byte int of range [-128, 127] should be enough to represent degree error
// int16_t recievedAngularErrors[2];
// enum Axis {
//   X = 0,
//   Y = 1
// };


// void setup() {
//   // Initiate all hardware components
//   initSteppers();
//   triggerServo.attach(TRIGGER_SERVO_PIN);  
//   triggerServo.write(TriggerServoSettings::RESET_POS);
//   Serial.begin(BAUDRATE);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
  
//   if (Serial.available() == COMPLETE_MESSAGE_LENGTH) {
//     readRecievedMessage();
//     #ifdef DEBUG
//       triggerServo.write(recievedAngularErrors[0]);
//       delay(800);
//       triggerServo.write(recievedAngularErrors[1]);
//       delay(800);
//       return;
//     #endif


//     int16_t xAngleError = recievedAngularErrors[Axis::X];
//     int16_t yAngleError = recievedAngularErrors[Axis::Y];

//     int16_t deltaBaseAngularErrors[2];
//     convertAnglesToDeltaBase(xAngleError, yAngleError, deltaBaseAngularErrors);

//     int16_t xDeltaAngleError = deltaBaseAngularErrors[Axis::X];
//     int16_t yDeltaAngleError = deltaBaseAngularErrors[Axis::Y];

//     speedPIDController(xDeltaAngleError, yDeltaAngleError);

//     calculateStepsToMove(xDeltaAngleError, yDeltaAngleError);
    
//     steppersController.moveTo(stepperGoToPositions);

//     if (shouldFire(xAngleError, yAngleError)) {
//       fire();
//     } 
//   }

//   steppersController.run();
// }

// /**
//  * Initiates all the stepper objects with their respective pins and adds them to the MultiStepper object.
// */
// void initSteppers() {
//   stepperX = AccelStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
//   stepperY = AccelStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//   for (AccelStepper stepper : steppers) {
//     stepper.setMaxSpeed(STEPPER_MAX_SPEED);
//     steppersController.addStepper(stepper);
//   }
// }

// /**
//  * Read and convert a message from the Serial interface to integers. 
//  * Prerequisite: Expects 8 bytes to be stored on the Serial connection (BBB'\n'BBB'\n')
// */
// void readRecievedMessage() {
//   // Read the angular errors from the serial interface
//   readSerialBytes(Axis::X);
//   readSerialBytes(Axis::Y);

//   #ifdef DEBUG 
//     Serial.print("X: ");
//     Serial.println(recievedAngularErrors[0]);
//     Serial.print("Y: ");
//     Serial.println(recievedAngularErrors[1]);
//   #endif
// }

// /**
//  * Read and convert the serial bytes until '\n' into an int16_t and insert them into index of recievedAngularErrors 
// */
// void readSerialBytes(uint8_t index) {
//   Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
//   Serial.read(); // Used to remove the '\n' from the previous message
//   recievedAngularErrors[index] = atoi(buf);
// }


// /**
//  * Converts cartesian angular errors to the coordinate system of the delta kinematics rig.
//  * @param xCartesianAngleError angle error in cartesian x-axis 
//  * @param yCartesianAngleError angle error in cartesian y-axis
//  * @param deltaBaseAngularErrors return array of calculated coordinates in new coordinate system
// */
// void convertAnglesToDeltaBase(int16_t xCartesianAngleError, int16_t yCartesianAngleError, int16_t deltaBaseAngularErrors[2]) {
//   // TODO
// }
  
// /**
//  * PID-controls the speeds of stepper motors given the angular errors in the delta coordinate system.
//  * @param xAngleError angle error in x-axis (delta)
//  * @param yAngleError angle error in y-axis (delta) 
// */
// void speedPIDController(int16_t xAngleError, int16_t yAngleError) {
//   // TODO
//   for (AccelStepper stepper : steppers) {
//   }
// }

// /**
//  * Calculates the steps needed for the stepper drivers to correct the current angular error in the delta coordinate system. 
//  * Sets the required steps in the global stepperGoToPositions array
//  * @param xAngleError angle error in x-axis (delta)
//  * @param yAngleError angle error in y-axis (delta) 
// */
// void calculateStepsToMove(int16_t xAngleError, int16_t yAngleError) {
  
//   int16_t xAxisStepsToMove = round(xAngleError / DEGREES_PER_STEP);
//   int16_t yAxisStepsToMove = round(yAngleError / DEGREES_PER_STEP);

//   // TODO: More complicated calculations?
//   stepperGoToPositions[Axis::X] = xAxisStepsToMove;
//   stepperGoToPositions[Axis::Y] = yAxisStepsToMove;
// }

// /**
//  * Return if the gun should fire based on cartesian angular errors
//  * @param xAngleError angle error in x-axis (cartesian)
//  * @param yAngleError angle error in y-axis (cartesian) 
//  * @return if the gun should fire
// */
// bool shouldFire(int16_t xAngleError, int16_t yAngleError) {
//   // Allow more leighway in the y-axis since that is not as crucial as x-axis
//   bool allowFireX = xAngleError <= TriggerServoSettings::FIRING_THRESHOLD_DEGREES;
//   bool allowFireY = yAngleError <= 3 * TriggerServoSettings::FIRING_THRESHOLD_DEGREES;
//   return allowFireX && allowFireY;
// }

// /**
//  * Drives a half rotation of the servo both ways to pull the connected trigger and fire off a paintball
// */
// void fire() {
//   triggerServo.write(TriggerServoSettings::FIRE_POS);
//   delay(TriggerServoSettings::FIRE_RATE_DELAY / 2);
//   triggerServo.write(TriggerServoSettings::RESET_POS);
//   delay(TriggerServoSettings::FIRE_RATE_DELAY / 2);
// }

// void stopFire() {
//   triggerServo.write(TriggerServoSettings::RESET_POS);
// }

