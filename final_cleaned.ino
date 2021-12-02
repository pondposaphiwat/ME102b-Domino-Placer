// Libraries for motor control
#include <AccelStepper.h>
#include <ESP32Servo.h>

// Pattern import
#include "patterns.h"

#define motorInterfaceType 1
#define dirPin 14
#define stepPin 32
#define dirPin2 15
#define stepPin2 33

#define servoPin1 12
#define servoPin2 25    //A1
#define servoPin3 26    //A0

// Switches
#define BTN 27   
#define limitX 34       //A2
#define limitY 39       //A3

// Ultrasonic sensor
#define trigPin 23
#define echoPin 22

// Servo timer Init
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Constants
const int refillThreshold = 600;
const int calibrateMovePosition = 3000;
const int refillPosition = 100;

const int servoHome_chute = 120;
const int servoHome_rotation = 1;
const int servoHome_pusher = 0;

const int servoExtended_chute = 180;
const int servoExtended_pusher = 180;

// Initialize variables (flags)
volatile bool reachedLimitX = false;
volatile bool reachedLimitY = false;
volatile bool startButtonPressed = false;
bool stackNeedsRefill = false;

//bool placerRotComplete = false;
//bool pusherRotComplete = false;
//bool chuteRotComplete = false;
//bool wholePlacerReturnComplete = false;

// General first, if this doesn't work will do 3 diff timers for 3 servos
bool servoRotComplete = false; 

int state = 0;
int ultrasonicDistance;
int currentStep = 0;
int returnStep = 0;

// Pattern declaration
int *x_pattern = pattern_test_x;
int *y_pattern = pattern_test_y;
int *rot_pattern = pattern_test_rot;

AccelStepper stepperX = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepperY = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
Servo myServo_rotation;
Servo myServo_pusher;
Servo myServo_chute;

// Limit switch interrupt
void IRAM_ATTR isr_limit_x() {  // the function to be called when interrupt is triggered
  reachedLimitX = true;
}
void IRAM_ATTR isr_limit_y() {  // the function to be called when interrupt is triggered
  reachedLimitY = true;
}

// Init Button: back to idle mode immediately
void IRAM_ATTR isr_init() {  // the function to be called when interrupt is triggered
  startButtonPressed = true;
}

// Timer interrupt for timing servo completion
void IRAM_ATTR onTime() {
  portENTER_CRITICAL_ISR(&timerMux);
  servoRotComplete = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Timer interrupt hook
void TimerInterruptInit() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer, 3000000, true);
  timerAlarmEnable(timer);
}

void setup() {
  // put your setup code here, to run once:

  // Interrupt Setup
  pinMode(BTN, INPUT);
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  attachInterrupt(BTN, isr_init, RISING);
  attachInterrupt(limitX, isr_limit_x, RISING);
  attachInterrupt(limitY, isr_limit_y, RISING);
  
  // Stepper Setup
  stepperX.setMaxSpeed(200);
  stepperX.setAcceleration(30);

  stepperY.setMaxSpeed(200);
  stepperY.setAcceleration(30);

  // Servo Setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo_rotation.setPeriodHertz(50);// Standard 50hz servo
  myServo_rotation.attach(servoPin1, 500, 2500);   // for 90 deg range use 1000 to 2000, for 180 use 500 to 2500

  myServo_pusher.setPeriodHertz(50);// Standard 50hz servo
  myServo_pusher.attach(servoPin2, 500, 2500); 
  myServo_chute.setPeriodHertz(50);// Standard 50hz servo
  myServo_chute.attach(servoPin3, 500, 2500); 

  // Ultrasonic Sensor Setup
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  // Timer Setup
  TimerInterruptInit();
  
  Serial.begin(9600);
}

void loop() {
  // State checker
  Serial.print("  Current step: ");
  Serial.print(currentStep);
  Serial.print(";  State: ");
  Serial.print(state);
  Serial.println(';');

  // put your main code here, to run repeatedly:
  switch (state) {
    
    // Idle
    case 0:
      if (startButtonPressedChecker()) {
        startPressedAtIdleService();
        // startButtonPressed = false;
        // state = 1;
      }
      break;
      
    // Calibration nation
    case 1:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }

      calibrationStateService();
//      rotateStepper(&stepperX, calibrateMovePosition);
//      rotateStepper(&stepperY, calibrateMovePosition);

      if (LimitSwitchChecker('s')) {
        limitSwitchesReachedService();
//        reachedLimitX = false;
//        reachedLimitY = false;
//        calibrateStepper(&stepperX);
//        calibrateStepper(&stepperY);
//        state = 2;
//        delayMicroseconds(1000000);
        
      } else {
        if (LimitSwitchChecker('x')) {
          calibrateStepper(&stepperX);
        }
        if (LimitSwitchChecker('y')) {
          calibrateStepper(&stepperY);
        }
      }
      break;

    // Move XY
    case 2:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }

      if (doneWithPatternChecker()) {
//        Serial.println("Done with pattern");
//        currentStep = 0;
//        state = 0;
        doneWithPatternService();
      }
      
      moveXYService();
      // rotateStepper(&stepperX, x_pattern[currentStep]);
      // rotateStepper(&stepperY, y_pattern[currentStep]);

      if (atTargetXYChecker(&stepperX, &stepperY)) {
//      if (!stepperX.isRunning() & !stepperY.isRunning()) {
        atTargetXYService();
//        stepperX.stop();
//        stepperY.stop();
//        delayMicroseconds(10000);
//        state = 3;
      }
      break;

//    // Post-calibration, pre-motion (Idle 2) (unimplemented)
//    case 11: 
//      // E-Stop
////      if (startButtonPressedChecker()) {
////        startButtonPressed = false;
////        state = 0;
////      }
//      
//      // Idle until button is pressed again 
//      if (startButtonPressedChecker()) {
//        startButtonPressed = false;
//        state = 2;
//      }
    
    //////////////////////////////////////////
    /////// CASE 3, 4, 5: PLACE DOMINO ///////
    //////////////////////////////////////////
    
    // Rotate whole placer
    case 3:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }
      
//      myServo_rotation.write(rot_pattern[currentStep]);
      rotateServo(&myServo_rotation, rot_pattern[currentStep]);
      if (placerRotCompleteChecker()) {
        placerRotCompleteService();
        // servoRotComplete = false;
        // state = 4;
        // timerRestart(timer);
      }
      break;
      
    // Lower chute
    case 4:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }
      
//      myServo_chute.write(180);
      rotateServo(&myServo_chute, servoExtended_chute);
    
      if (chuteRotCompleteChecker()) {
        chuteRotCompleteService();
        // servoRotComplete = false;
        // state = 5;
        // timerRestart(timer);
      }
      break;
    
    // Push domino
    case 5:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }

      rotateServo(&myServo_pusher, servoExtended_pusher);
      
//      myServo_pusher.write(180);
      if (pusherRotCompleteChecker()) {
        pusherRotCompleteService();
      }
      break;
    //////////////////////////////////////////
    
    ////////////////////////////////////////////////////////
    ///CASE 8, 9, 10: RETURN PLACER TO ZERO CONFIG STATE ///
    ////////////////////////////////////////////////////////

    // Return chute
    case 8:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }

      rotateServo(&myServo_chute, servoHome_chute);
//      myServo_chute.write(0);
      if (chuteReturnCompleteChecker()) {
        chuteReturnCompleteService();
        // servoRotComplete = false;
        // state = 9;
        // timerRestart(timer);
      }
      break;
      
    // Return placer rotation
    case 9:
      // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }
//      myServo_rotation.write(0);
      rotateServo(&myServo_rotation, servoHome_rotation);
      
      if (placerReturnCompleteChecker()) {
        placerReturnCompleteService();
        // servoRotComplete = false;
        // state = 10;
        // timerRestart(timer);
      }
      break;

    // Retract pusher
    case 10:
          // E-Stop
      if (startButtonPressedChecker()) {
        eStopService();
      }
      
//      myServo_pusher.write(0);
      rotateServo(&myServo_pusher, servoHome_pusher);
      if (pusherReturnCompleteChecker()) {
        pusherReturnCompleteService();
        // servoRotComplete = false;
        // currentStep += 1;
        if (stackNeedsRefillChecker()) {
          stackNeedsRefillService();
          // stackNeedsRefill = false;
          // state = 6;
        } else {
          stackDoesNotNeedRefillService();
          // state = 2;
        }
      }
      break;
    //////////////////////////////////////////
   
    // Refill - move to closest y edge
    case 6:
      rotateStepper(&stepperY, refillPosition);

      if (startButtonPressedChecker()) {
        doneRefillService();
        // startButtonPressed = false;
        // state = 2;
      }

      break;
      
    // Error state
    case 7:
      Serial.println("Error");
      if (startButtonPressedChecker()){
        errorStateOverService();
        // startButtonPressed = false;
        // state = 0;
      }
      break;
  }
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


// Event checkers
bool startButtonPressedChecker() {
  return startButtonPressed;
}

bool stackNeedsRefillChecker() {
  checkRefill();
  return stackNeedsRefill;
}

bool motorIsNotRunningChecker(AccelStepper stepperX, AccelStepper stepperY) {
  return (!stepperX.isRunning() & !stepperY.isRunning());
}

bool placerIsAtEdgeChecker() {
  // Check if it is in the allowed range
  if (-1 < stepperY.currentPosition() < 1) {
    return true;
  } return false;
}

bool LimitSwitchChecker(char axis) {
  if (axis == 'x') {
    return reachedLimitX;
  } else if (axis == 'y') {
    return reachedLimitY;
  } else if (axis == 's') {
    return (reachedLimitX && reachedLimitY);
  }
}

// To check whether stepper has reached target position
bool stepperRunningChecker(AccelStepper stepper) {
  return stepper.isRunning();
}

bool placerRotCompleteChecker() {
  return servoRotComplete;
}

bool chuteRotCompleteChecker() {
  return servoRotComplete;
}

bool pusherRotCompleteChecker() {
  return servoRotComplete;
}

bool chuteReturnCompleteChecker() {
  return servoRotComplete;
}

bool placerReturnCompleteChecker() {
  return servoRotComplete;
}

bool pusherReturnCompleteChecker() {
  return servoRotComplete;
}

bool doneWithPatternChecker() {
  return (currentStep >= sizeof x_pattern);
}

bool atTargetXYChecker(AccelStepper *stepperX, AccelStepper *stepperY) {
  return (!(stepperX->isRunning()) && !(stepperY->isRunning()));
}

// Services
void checkRefill() {
  int durationSum = 0;
  
  for (int i = 0; i < 10; ++i) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    durationSum += duration;
  }

  float durationAvg = durationSum/10.0;
  Serial.println(durationAvg);
  if (durationAvg > refillThreshold) {
    stackNeedsRefill = true; 
  }
}

void stopStepper(AccelStepper *stepper) {
  stepper->stop();  
}

void calibrateStepper(AccelStepper *stepper) {
  stepper->stop();
  stepper->setCurrentPosition(0);
}

void rotateStepper(AccelStepper *stepper, int goal_position) {
  stepper->moveTo(goal_position);
  stepper->run();
}

// Check again if blocking is ok, might need timer interrupt to help see if done
void rotateServo(Servo *servo, int goal_angle) {
  servo->write(goal_angle);
}

// E-Stop service
void eStopService() {
  startButtonPressed = false;
  state = 0;
}

void pusherRotCompleteService() {
  servoRotComplete = false;
  state = 8;
  timerRestart(timer);
}

void limitSwitchesReachedService() {
  reachedLimitX = false;
  reachedLimitY = false;
  calibrateStepper(&stepperX);
  calibrateStepper(&stepperY);
  state = 2;
  delayMicroseconds(1000000);
}

void atTargetXYService() {
  stepperX.stop();
  stepperY.stop();
  delayMicroseconds(10000);
  state = 3;
}

void doneWithPatternService() {
  Serial.println("Done with pattern");
  currentStep = 0;
  state = 0;
}

void calibrationStateService() {
  rotateStepper(&stepperX, calibrateMovePosition);
  rotateStepper(&stepperY, calibrateMovePosition);
}

void placerReturnCompleteService() {
  servoRotComplete = false;
  state = 10;
  timerRestart(timer);
}

void placerRotCompleteService() {
  servoRotComplete = false;
  state = 4;
  timerRestart(timer);
}

void doneRefillService() {
  startButtonPressed = false;
  state = 2;
}

void startPressedAtIdleService() {
  startButtonPressed = false;
  state = 1;
}

void moveXYService() {
  rotateStepper(&stepperX, x_pattern[currentStep]);
  rotateStepper(&stepperY, y_pattern[currentStep]);
}

void chuteRotCompleteService() {
  servoRotComplete = false;
  state = 5;
  timerRestart(timer);
}

void chuteReturnCompleteService() {
  servoRotComplete = false;
  state = 9;
  timerRestart(timer);
}

void pusherReturnCompleteService() {
  servoRotComplete = false;
  currentStep += 1;
}

void stackNeedsRefillService() {
  stackNeedsRefill = false;
  state = 6;
}

void stackDoesNotNeedRefillService() {
  state = 2;
}

void errorStateOverService() {
  startButtonPressed = false;
  state = 0;
}