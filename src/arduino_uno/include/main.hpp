#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


/* HARDWARE PINS */
const uint8_t Y_STEP_PIN = 14; //TBD
const uint8_t Y_DIR_PIN = 15; //TBD
const uint8_t X_STEP_PIN = 2; //TBD
const uint8_t X_DIR_PIN = 3; //TBD
const uint8_t TRIGGER_SERVO_PIN = 6;

/* MOTOR SETTINGS */
const uint16_t STEPPER_MAX_SPEED = 600;
const uint8_t STEPS_PER_ROTATION = 200;
const float DEGREES_PER_STEP = 1.8f;
long stepperGoToPositions[2]; // [X, Y]

AccelStepper stepperX;
AccelStepper stepperY;
AccelStepper steppers[2] = {stepperX, stepperY};
MultiStepper steppersController;

Servo triggerServo;
bool triggerServoFiringState = 0; // 0: Reset position, 1: Fire position. Used for the timer to switch between the two.
enum TriggerServoSettings {
  FIRE_POS = 170, // half rotation with servo
  RESET_POS = 0,
  FIRE_RATE_DELAY_MS = 800, // ms for delay() call. 600?
  FIRING_THRESHOLD_DEGREES = 5
};

/* SERIAL INTERFACE SETTINGS */
const uint32_t BAUDRATE = 115200; //57600, 115200
//const uint8_t BUFFER_SIZE = 3; // 3 ASCII Characters max (will not send bigger than 3 digit number)
//const uint8_t COMPLETE_MESSAGE_LENGTH = BUFFER_SIZE * 4 + 2;
const uint8_t numChars = 64;
char receivedChars[numChars];
const uint8_t PACKAGE_LENGTH = 4;
int8_t parsedMessage[PACKAGE_LENGTH];
//char buf[BUFFER_SIZE]; // [X, Y] One byte int of range [-128, 127] should be enough to represent degree error
bool newData = false;
int16_t receivedPackageId = -1;
int16_t receivedAngularErrors[2];
bool shouldFire = false;
int16_t receivedPidValues[3];

/* TIMER SETTINGS */
int timerMs = 0;


enum Axis {
  X = 0,
  Y = 1,
};

enum Pid {
  Kp = 0,
  Ki = 1,
  Kd = 2,
};

enum PackageId {
  UNDEFINED = 0,
  ANGULAR_ERROR = 1,
  FIRE = 2,
  PID_UPDATE = 3,
};


void loop();
void setup();

/**
 * Initiates all the stepper objects with their respective pins and adds them to the MultiStepper object.
*/
void initSteppers();

/**
 * Initiates the internal TIMER0 to trigger an interrupt every 1 ms.
 * Used for the firing of the gun and to avoid using delay() to not block incoming serial requests.
*/
void initTimer();

/**
 * Read a message from the serial interface enclosed by start and end markers.
*/
void recvWithStartEndMarkers();

/**
 * Convert the read serial message to integers and store them in the parsedMessage variable to be
 *  later used for determining what kind of information was received.
*/
void parseReceivedMessage();

/**
 * Extract the package ID and corresponding data from the parsedMessage list.
*/
void extractMessageInformation();

/**
 * Converts cartesian angular errors to the coordinate system of the delta kinematics rig.
 * @param[in] xCartesianAngleError The angle error in cartesian x-axis.
 * @param[in] yCartesianAngleError The angle error in cartesian y-axis.
 * @param[out] deltaBaseAngularErrors The return array of calculated coordinates in new coordinate system.
*/
void convertAnglesToDeltaBase(int16_t xCartesianAngleError, int16_t yCartesianAngleError, int16_t deltaBaseAngularErrors[2]);

/**
 * PID-controls the speeds of stepper motors given the angular errors in the delta coordinate system.
 * @param[in] xAngleError The angle error in x-axis (delta-base).
 * @param[in] yAngleError The angle error in y-axis (delta-base).
*/
void speedPIDController(int16_t xAngleError, int16_t yAngleError);

/**
 * Calculates the steps needed for the stepper drivers to correct the current angular error in the delta coordinate system.
 * Sets the required steps in the global stepperGoToPositions array
 * @param[in] xAngleError angle error in x-axis (delta)
 * @param[in] yAngleError angle error in y-axis (delta)
*/
void calculateStepsToMove(int16_t xAngleError, int16_t yAngleError);

/**
 * Return if the gun should fire based on cartesian angular errors
 * @param xAngleError angle error in x-axis (cartesian)
 * @param yAngleError angle error in y-axis (cartesian)
 * @return if the gun should fire
 * TODO: Remove?
*/
// bool shouldFire(int16_t xAngleError, int16_t yAngleError) {
//   // Allow more leighway in the y-axis since that is not as crucial as x-axis
//   bool allowFireX = xAngleError <= TriggerServoSettings::FIRING_THRESHOLD_DEGREES;
//   bool allowFireY = yAngleError <= 3 * TriggerServoSettings::FIRING_THRESHOLD_DEGREES;
//   return allowFireX && allowFireY;
// }


/**
 * Drives a half rotation of the servo both ways to pull the connected trigger
*/
void handleFiring();

/**
 * Move the servo to the firing position, pulling the trigger.
*/

void fire();
/**
 * Return the servo to the starting position, releasing the trigger.
*/
void stopFire();