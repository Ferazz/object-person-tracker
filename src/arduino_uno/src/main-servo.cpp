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
// AccelStepper stepper1(AccelStepper::DRIVER, 13, 12); // (Typeof driver: with 2 pins, STEP, DIR)
Servo triggerServo;

void setup() {

  // stepper1.setMaxSpeed(1000); // Set maximum speed value for the stepper
  // stepper1.setAcceleration(500); // Set acceleration value for the stepper
  // stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  triggerServo.attach(6);  
  triggerServo.write(0);
  Serial.begin(115200);
}

void loop() {
  // stepper1.moveTo(800); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  // stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
  triggerServo.write(0);
  triggerServo.read();
  // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
  // stepper1.moveTo(0);
  // while (stepper1.currentPosition() != 0) {
  //   stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
  // }

  delay(500);
}