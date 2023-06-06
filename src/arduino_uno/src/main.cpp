#include <main.hpp>

void setup() {
  // Initiate all hardware components
  initSteppers();
  initTimer();
  triggerServo.attach(TRIGGER_SERVO_PIN);
  triggerServo.write(TriggerServoSettings::RESET_POS);
  Serial.begin(BAUDRATE);
  Serial.println("<Arduino is ready>");
}

void loop() {
  // put your main code here, to run repeatedly:
  recvWithStartEndMarkers();
  if (newData == true) {
    parseReceivedMessage();
    switch (receivedPackageId)
    {
    case PackageId::ANGULAR_ERROR: {
        //Serial.println("ANGULAR");
        int16_t xAngleError = receivedAngularErrors[Axis::X];
        int16_t yAngleError = receivedAngularErrors[Axis::Y];

        int16_t deltaBaseAngularErrors[2];
        convertAnglesToDeltaBase(xAngleError, yAngleError, deltaBaseAngularErrors);

        int16_t xDeltaAngleError = deltaBaseAngularErrors[Axis::X];
        int16_t yDeltaAngleError = deltaBaseAngularErrors[Axis::Y];

        speedPIDController(xDeltaAngleError, yDeltaAngleError);

        calculateStepsToMove(xDeltaAngleError, yDeltaAngleError);

        //steppersController.moveTo(stepperGoToPositions);
        break;
      }
    case PackageId::PID_UPDATE: {
        Serial.println("PID");
        break;
      }
    default:
     // Serial.print("DEFAULT: ");
      //Serial.println(receivedPackageId);
      break;
    }

    newData = false;
  }

  handleFiring();

  //steppersController.run();
  
}


void initSteppers() {
  stepperX = AccelStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
  stepperY = AccelStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

  for (AccelStepper stepper : steppers) {
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    steppersController.addStepper(stepper);
  }
}

void initTimer() {
    // From: https://projecthub.arduino.cc/Marcazzan_M/internal-timers-of-arduino-6c0f66
    TCCR0A=(1<<WGM01);    //Set the CTC mode   
    OCR0A=0xF9; //Value for ORC0A for 1ms 

    TIMSK0|=(1<<OCIE0A);   //Set  the interrupt request
    sei(); //Enable interrupt

    TCCR0B|=(1<<CS01);    //Set the prescale 1/64 clock
    TCCR0B|=(1<<CS00);
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '[';
    char endMarker = ']';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseReceivedMessage() {
  char tempChars[numChars]; 
  strcpy(tempChars, receivedChars); // Make copy of char[] to not destroy it.
  uint8_t parsedMessageIndex = 0;
  for (char * token = strtok(tempChars, ","); token != NULL; token = strtok(NULL, ",")) {
    parsedMessage[parsedMessageIndex] = atoi(token);
    parsedMessageIndex++;
  }
  //Serial.println(receivedChars);
  extractMessageInformation();
}

void extractMessageInformation() {
  receivedPackageId = parsedMessage[0];

  switch (receivedPackageId)
  {
  case PackageId::ANGULAR_ERROR:
    receivedAngularErrors[Axis::X] = parsedMessage[1];
    receivedAngularErrors[Axis::Y] = parsedMessage[2];
    break;
  case PackageId::FIRE:
    shouldFire = parsedMessage[1];
    Serial.println(shouldFire);
    break;
  
  case PackageId::PID_UPDATE:
    receivedPidValues[Pid::Kp] = parsedMessage[1];
    receivedPidValues[Pid::Ki] = parsedMessage[2];
    receivedPidValues[Pid::Kd] = parsedMessage[3];
    break;
  
  default:
    break;
  }
  //memset(receivedChars, 0, sizeof(receivedChars));


  // TODO: Is clearing the array necessary?

  // #ifdef DEBUG
  //   Serial.print("X: ");
  //   Serial.println(receivedAngularErrors[0]);
  //   Serial.print("Y: ");
  //   Serial.println(receivedAngularErrors[1]);
  // #endif
}


void convertAnglesToDeltaBase(int16_t xCartesianAngleError, int16_t yCartesianAngleError, int16_t deltaBaseAngularErrors[2]) {
  // TODO
}


void speedPIDController(int16_t xAngleError, int16_t yAngleError) {
  // TODO
  for (AccelStepper stepper : steppers) {
  }
}


void calculateStepsToMove(int16_t xAngleError, int16_t yAngleError) {

  int16_t xAxisStepsToMove = round(xAngleError / DEGREES_PER_STEP);
  int16_t yAxisStepsToMove = round(yAngleError / DEGREES_PER_STEP);

  // TODO: More complicated calculations?
  stepperGoToPositions[Axis::X] = xAxisStepsToMove;
  stepperGoToPositions[Axis::Y] = yAxisStepsToMove;
}



void handleFiring() {
  if (timerMs >= TriggerServoSettings::FIRE_RATE_DELAY_MS / 2) {
    if (shouldFire) {
      if (triggerServoFiringState == 0) {
        fire();
      }
      else {
        stopFire();
      }
      triggerServoFiringState = !triggerServoFiringState;
      
    }
    else {
      if (triggerServoFiringState == 1) {
        stopFire();
      }
    }

    timerMs = 0;
  }
}
void fire() {
  triggerServo.write(TriggerServoSettings::FIRE_POS);
}

void stopFire() {
  triggerServo.write(TriggerServoSettings::RESET_POS);
}

ISR(TIMER0_COMPA_vect){
  timerMs++;
}
