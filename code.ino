#include <Servo.h>

#include <Wire.h>
#include <VL53L0X.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define MAXSPEED 950
#define ACCELERATION 800
#define ROTATION_FULL 1600
#define WHEEL_PERIMETER 314
#define COLLISION_DISTANCE 250
#define STATE_STANDBY 0
#define STATE_RUNNING 1
#define STATE_STOPPING 2
#define STATE_STOPPED 3
#define STATE_RESUMING 4
#define START_PIN 10
#define TEAM_PIN 7
#define LED_PIN 3
#define LEFT_DRIVER_PIN 36
#define RIGHT_DRIVER_PIN 37

long target1 = 0L;
long target2 = 0L;

AccelStepper stepper1(AccelStepper::DRIVER,8,9);
AccelStepper stepper2(AccelStepper::DRIVER,6,5);
Servo servoLeft;
Servo servoRight;
VL53L0X sensor;

MultiStepper steppers;

bool obstacleHere = false;

int state = STATE_STANDBY;

int distanceToSteps(float distMM) {
  float wheelRotations = distMM / float(WHEEL_PERIMETER);
  return int(ROTATION_FULL * wheelRotations);
}

int lastTime = 0;
int lastMeasure = 9999;

bool hasObstacle() {
  return false;
  int currentTime = millis();
  if(lastTime+200 < currentTime) {
    lastTime = currentTime;
    lastMeasure = sensor.readRangeContinuousMillimeters();
  }
  return lastMeasure < COLLISION_DISTANCE;
}

void stop() {
  if(state != STATE_RUNNING) return;
  target1 = stepper1.targetPosition();
  target2 = stepper2.targetPosition();
  stepper1.stop();
  stepper2.stop();
  state = STATE_STOPPING;
}

void resume() {
  if(state != STATE_STOPPED) return;
  stepper1.moveTo(target1);
  stepper2.moveTo(target2);
  state = STATE_RUNNING;
}

void runWithObstacles() {
  while(state != STATE_STANDBY) {
    if(hasObstacle()) {
      Serial.println("Obstacle");
      if(state == STATE_RUNNING) {
        Serial.println("Stopping");
        stop();
      }
    } else {
      if(state == STATE_STOPPED) {
        Serial.println("Resuming");
        resume();
      }
    }
    step();
  }
}

void moveForward(float distMM) {
  if(state != STATE_STANDBY) return;
  
  int steps = distanceToSteps(distMM);
  stepper1.setMaxSpeed(MAXSPEED);
  stepper2.setMaxSpeed(MAXSPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper2.setAcceleration(ACCELERATION);
  stepper1.move(steps);
  stepper2.move(-steps);
  state = STATE_RUNNING;

  runWithObstacles();
}

void moveRight(float distMM) {
  if(state != STATE_STANDBY) return;
  int steps = distanceToSteps(distMM);
  
  stepper1.setMaxSpeed(MAXSPEED * 1.181);
  stepper1.setAcceleration(ACCELERATION * 1.181);
  stepper2.setMaxSpeed(MAXSPEED);
  stepper2.setAcceleration(ACCELERATION);
  stepper1.move(steps * 1.181);
  stepper2.move(-steps);
  state = STATE_RUNNING;

  runWithObstacles();
}

void moveLeft(float distMM) {
  if(state != STATE_STANDBY) return;
  int steps = distanceToSteps(distMM);
  
  stepper2.setMaxSpeed(MAXSPEED * 1.181);
  stepper2.setAcceleration(ACCELERATION * 1.181);
  stepper1.setMaxSpeed(MAXSPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper2.move(steps * 1.181);
  stepper1.move(-steps);
  state = STATE_RUNNING;

  runWithObstacles();
}

void step() {
  if(!stepper1.isRunning() && !stepper2.isRunning()) {
    if(state == STATE_RUNNING) {
      state = STATE_STANDBY;
    } else if(state == STATE_STOPPING) {
      state = STATE_STOPPED;
    }
  } else {
    if(state == STATE_RESUMING) {
      state = STATE_RUNNING;
    }
    stepper1.run();
    stepper2.run();
  }
}

void drivers(bool enable) {
  if(enable) {
    digitalWrite(LEFT_DRIVER_PIN, LOW);
    digitalWrite(RIGHT_DRIVER_PIN, LOW);
  } else {
    digitalWrite(LEFT_DRIVER_PIN, HIGH);
    digitalWrite(RIGHT_DRIVER_PIN, HIGH);
  }
  delay(5);
}

void flapsUp() {
  servoLeft.write(180);
  servoRight.write(0);
  delay(1000);
}

void flapsDown() {
  servoLeft.write(0);
  servoLeft.write(180);
  delay(1000);
}

bool isActivatorPresent() {
  return digitalRead(START_PIN) == 0;
}

bool isActivated() {
  return digitalRead(START_PIN) == 1;
}

void waitActivator() {
  // wait for activator
  while(!isActivatorPresent()) {
    delay(100);
  }
}

void waitActivation() {
  // wait for activator
  while(!isActivated()) {
    delay(100);
  }
}

bool isTeamPurple() {
  return digitalRead(TEAM_PIN) == 0;
}

void playPurple() {
  moveForward(2000);
}

void playYellow() {
  moveLeft(10000);
}

void setup() {
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Waiting activator!");
  waitActivator();
  Serial.println("Activator here!");
  waitActivation();
  Serial.println("Activated!");
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  servoLeft.attach(16);
  servoRight.attach(17);
  
  pinMode(LEFT_DRIVER_PIN, OUTPUT);
  pinMode(RIGHT_DRIVER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEAM_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  } else {
    sensor.startContinuous();
    drivers(true);
    if(isTeamPurple()) playPurple();
    else playYellow();
    drivers(false);
  }
}



void loop() {
  //digitalWrite(3, hasObstacle()? HIGH : LOW);
  //flapsUp();
  //delay(2000);
  //flapsDown();
} 
