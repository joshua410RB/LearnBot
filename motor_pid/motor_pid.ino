#include<DRV8833.h>

// Just driver configs
const int leftMotor0 = 12; // Yellow
const int leftMotor1 = 11; // Green
const int rightMotor0 = 9; // White
const int rightMotor1 = 10; // Yellow
const int leftEncoder = 3;
const int rightEncoder = 2;

int speed = 0; // -255 to 255
int steer = 0; // Goes from -128 to 127

int incr = 1; // Used for motor control

volatile int leftDemand = 0, rightDemand = 0; // Demand variables

volatile bool toggle = false;
bool buttonPress;

// Encoder stuff
const float ROT_DIST = 10.2; // Dist (in mm) travelled per trigger of the interrupt
const float TOP_SPEED = 400; // Keep things proportional for now

// Control Stuff
const float cf_prop = 5.0; // Proportional const in PID
const float cf_int = 0.01;
const float cf_diff = 0.01;
float leftError, rightError, leftErrorAcc, rightErrorAcc, leftErrorDiff, rightErrorDiff, leftWrite, rightWrite; 
// Accumulators for left and right motor error and differential
long lastControl=0, controlDelta=0; // time delta for control
int leftWriteSpeed, rightWriteSpeed;

volatile long rightRots = 0, leftRots = 0;  // Distance Travelled
volatile float rightSpeed, leftSpeed; // Dist (m/s)
volatile long lastLeft = 0, lastRight = 0, lastLeftDiff = 0, lastRightDiff = 0; // Used to keep speed measurements

int currTime = 0;

void steerFn(int speed, int steer);
void speedControl(); // Control speed of robot
DRV8833 driver = DRV8833();

void setup() {
  Serial.begin(115200);
  pinMode(leftEncoder, INPUT);
  pinMode(rightEncoder, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, INPUT_PULLUP);

  //set timer3 interrupt at 10Hz for speed reset
  TCCR3A = 0; // set entire TCCR2A register to 0
  TCCR3B = 0; // same for TCCR2B
  TCNT3  = 0; //initialize counter value to 0
  OCR3A = 781; // (16MHz/(10Hz*1024 Prescaler)) = 1563 in compare register
  TCCR3A |= (1 << WGM21); // turn on CTC mode
  TCCR3B |= (1 << CS12) | (1 << CS10); // Set CS10 and CS12 bits for 1024 prescaler
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  
  attachInterrupt(digitalPinToInterrupt(leftEncoder), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoder), rightEncoderISR, RISING);
  
  driver.attachMotorA(leftMotor0, leftMotor1);
  driver.attachMotorB(rightMotor0, rightMotor1);
}

void loop() {

  int delta = millis() - currTime;
  steerFn(speed, 0);  // PID needs to react quickly w/o latency

  if (!digitalRead(7)){
    buttonPress = true;
  }
  
  // Nonblocking delay start
  if (delta > 1000) {
    Serial.print("Speed demand: ");
    Serial.print(speed);
    Serial.print(" | Control delta: ");
    Serial.println(controlDelta);
//    Serial.print(" | Last Ctrl: ");
//    Serial.print(lastControl);
//    Serial.print(" | Current Time: ");
//    Serial.println(millis());
    Serial.print("Right speed: ");
    Serial.print(rightSpeed);
    Serial.print(" | Left speed: ");
    Serial.println(leftSpeed);
    Serial.print("Right last: ");
    Serial.print(lastRightDiff);
    Serial.print(" | Left last: ");
    Serial.println(lastLeftDiff);
    Serial.print("Right Error: ");
    Serial.print(rightError);
    Serial.print(" | Left Error: ");
    Serial.println(leftError);
    Serial.print("Right Error Acc: ");
    Serial.print(rightErrorAcc);
    Serial.print(" | Left Error Acc: ");
    Serial.println(leftErrorAcc);
    Serial.print("Right Error Diff: ");
    Serial.print(rightErrorDiff);
    Serial.print(" | Left Error Diff: ");
    Serial.println(leftErrorDiff);
    
    Serial.print("Right Write Demand: ");
    Serial.print(rightWrite);
    Serial.print(" | Left Write Demand: ");
    Serial.println(leftWrite);
    Serial.println("");

    Serial.print("Right Write Speed: ");
    Serial.print(rightWriteSpeed);
    Serial.print(" | Left Write Speed: ");
    Serial.println(leftWriteSpeed);
    Serial.println("");

    // Go in the opposite direction so both motors spin(go figure)
    if (buttonPress) speed -= TOP_SPEED/8;
    if (speed < -(TOP_SPEED+1)) speed = 0;
    buttonPress = false; // ButtonPress is the flag

//    speed += incr*5;
//    if (speed > 255) {
//      incr = -1;
//    }
//    else if (speed < 5) {
//      incr = 1;
//    }
//    
    currTime = millis();
  }
  // Nonblocking delay end
  
}

void steerFn(int speed, int steer){
  if (steer > 0){
    leftDemand = speed;
    rightDemand = speed * (-2*steer + 127) / 127;
  } else {
    rightDemand = speed;
    leftDemand = speed * (2*steer + 127) / 127;
  }
//  leftDemand>0 ? driver.motorAForward(leftDemand) : driver.motorAReverse(-leftDemand);
//  rightDemand>0 ? driver.motorBForward(rightDemand) : driver.motorBReverse(-rightDemand);

  speedControl(); // Feedback control for speed
}

void speedControl(){
  // Takes in speed demand values and performs PID control
  controlDelta = (micros()-lastControl)/1000000;
  if (controlDelta!=0){
    leftErrorDiff = ((leftDemand-leftSpeed)-leftError)/controlDelta;  // as it is in microseconds
    rightErrorDiff = ((rightDemand-rightSpeed)-rightError)/controlDelta; 
//    Serial.print("Right Error d/dt: ");
//    Serial.print(rightErrorDiff);
//    Serial.print(" | Left Error d/dt: ");
//    Serial.println(leftErrorDiff);
  }
  leftError = leftDemand - leftSpeed;
  rightError = rightDemand - rightSpeed;
  leftErrorAcc = constrain(leftError+leftErrorAcc, -20000, 20000);
  rightErrorAcc = constrain(rightError+rightErrorAcc, -20000, 20000);

  leftWrite = leftDemand + leftError*cf_prop + leftErrorAcc*cf_int + leftErrorDiff*cf_diff; // Proportional control
  rightWrite = rightDemand + rightError*cf_prop + rightErrorAcc*cf_int + rightErrorDiff*cf_diff;

  if(leftDemand==0 || leftWrite>0 && leftDemand<0 || leftWrite<0 && leftDemand>0)         leftWrite = 0;
  if(rightDemand==0 || rightWrite>0 && rightDemand<0 || rightWrite<0 && rightDemand>0)    rightWrite = 0;
  // Don't cross over to write the other way (limits control, but sensing can't do this)
  
  leftWriteSpeed = map(int(leftWrite), -TOP_SPEED, TOP_SPEED, -255, 255);
  rightWriteSpeed = map(int(rightWrite), -TOP_SPEED, TOP_SPEED, -255, 255);
  leftWriteSpeed = constrain(leftWriteSpeed, -255, 255);
  rightWriteSpeed = constrain(rightWriteSpeed, -255, 255);  // map and constrain
  
 // Write motor speeds (ternary function yay)
  leftWriteSpeed>0 ? driver.motorAForward(leftWriteSpeed) : driver.motorAReverse(-leftWriteSpeed);
  rightWriteSpeed>0 ? driver.motorBForward(rightWriteSpeed) : driver.motorBReverse(-rightWriteSpeed);
  
  lastControl = micros(); // Update time delta
}

void leftEncoderISR(){
  noInterrupts();
  if ((millis() - lastLeft)==0) return;
  // Exit immediately if there is a double triggering
  lastLeftDiff = millis() - lastLeft;

  leftSpeed = 1000*ROT_DIST / lastLeftDiff;
  if (leftDemand > 0) {
    leftRots++;
  } else {
    leftRots--;
    leftSpeed *= -1;
  }
  
  lastLeft = millis();

  interrupts();
}

void rightEncoderISR(){
  noInterrupts();
  if ( (millis() - lastRight)==0) return;
  lastRightDiff = millis() - lastRight;

  rightSpeed = 1000*ROT_DIST / lastRightDiff;
  if (rightDemand > 0) {
    rightRots++;
  } else {
    rightRots--;
    rightSpeed *= -1;
  }
    
  lastRight = millis();

  interrupts();
}

// Reset speeds to zero
ISR(TIMER3_COMPA_vect){
  // visualise
//  toggle = !toggle;
//  digitalWrite(LED_BUILTIN, toggle);
  
  if (millis()-lastLeft > 100){
    leftSpeed = 0;
//    lastLeftDiff = 0;
  }
  if (millis()-lastRight > 100){
    rightSpeed = 0;
//    lastRightDiff = 0;
  }
  // ISR resets speed vars to 0
  // if no external interrupt triggers in the last 50ms
}
