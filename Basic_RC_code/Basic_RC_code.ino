#include <WiFiRemoteRx.h>
#include <DRV8833.h>

//Code for Due (but will work with other arduinos that multiple UARTs)
WiFiRemoteRx rx(Serial1);
DRV8833 driver=DRV8833();

const int inputA1=5, inputA2=6, inputB1=9, inputB2=10;

int motorSpeedA=0;
int motorSpeedB=0;

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,0);
  delay(1000);
  rx.begin(); //Initialize the receiver connection
  while(!rx.getNetworkStatus()) {
    delay(100);
  }

  //Initialize motor driver pins 
  driver.attachMotorA(inputA1, inputA2);
  driver.attachMotorB(inputB1, inputB2);
  driver.motorAStop();
  driver.motorBStop();

  //flash LED on pin 13 while there is no connection
  while(!rx.getConnectionStatus()) {
    digitalWrite(13,1);
    delay(100);
    digitalWrite(13,0);
    delay(100);
  }
  digitalWrite(13,1);
}

void loop() {
  if(rx.getConnectionStatus())  {
    motorSpeedA=rx.getChannel(1)-127;
    motorSpeedB=rx.getChannel(3)-127;
    if(motorSpeedA>0) {
      driver.motorAForward(2*motorSpeedA);
    }
    else  {
      driver.motorAReverse(2*abs(motorSpeedA));
    }
    if(motorSpeedB>0) {
      driver.motorBForward(motorSpeedB);
    }
    else  {
      driver.motorBReverse(2*abs(motorSpeedB));
    }
  }
  else  {
    driver.motorAStop();
    driver.motorBStop();
    while(!rx.getConnectionStatus()) {
      digitalWrite(13,1);
      delay(100);
      digitalWrite(13,0);
      delay(100);
    }
    digitalWrite(13,1);
  }
}
