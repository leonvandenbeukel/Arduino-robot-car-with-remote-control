/*

Leon van den Beukel - 2014

Homemade Arduino robot car with remote control:
https://www.youtube.com/watch?v=ashG1SpF95w

This code is for the robot car (receiver)

*/

/***************************************************
  H-Bridge connections:
  Enable Motor    HIGH – Enable  LOW – Disable Motor
  Direction 1     IN1  – HIGH    IN2 – LOW
  Direction 2     IN1  – LOW     IN2 – HIGH
  Coasting        IN1  – LOW     IN2 – LOW
  Break           IN1  – HIGH    IN2 – HIGH
 ***************************************************/
#include <NewPing.h>
#include <Servo.h> 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(9,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
int commandsize = 4;

Servo steeringservo;     // Servo object for steering
Servo distanceservo;     // Servo object for distance sensor
#define pinA1 2          // H-bridge pin1
#define pinA2 3          // H-bridge pin2
#define TRIGGER_PIN  31  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     33  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
#define maxDistance 75   // Maximum distance away from an object (cm)
int distanceservoIncr=5; // The increment movement value of the distance servo 

/**************
  LCD display
 **************/
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

#define lightPIN 22

/*************
 Setup routine
 *************/
void setup() 
{ 
  lcd.begin (16,2);
  printlcd("Ready.");
  
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();      
  
  // Serial for debugging to pc
  Serial.begin(57600);
  printf_begin();
  printf("\r\nRF24/RECEIVER/\r\n");

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size. seems to improve reliability
  radio.setPayloadSize(8);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();
  radio.printDetails();  

  // Serial1 is used for bluetooth communication 
  Serial1.begin(9600);  
   
  // Attach the servos
  steeringservo.attach(A3);  
  distanceservo.attach(A4);  

  // Init the h-bridge  
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  
  // Center the steering wheels and distance servo
  steerCenter();
  distanceservo.write(90);
  
  pinMode(lightPIN, OUTPUT);  
  
} 

/************
 Main routine
 ************/
void loop() 
{   
  
  // if there is data ready
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    uint8_t commands[commandsize];
    bool done = false;
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read( &commands, commandsize );
      
      // Up/Down and lights on
      if (commands[0] == HIGH && commands[1] == HIGH) {
        digitalWrite(lightPIN, HIGH); 
      } else if (commands[1] == HIGH && commands[0] == LOW) {
        moveBackward();
      } else if (commands[0] == HIGH && commands[1] == LOW) {
        moveForward();
      } else if (commands[0] == LOW && commands[1] == LOW) {       
        stopMoving();
      }

      // Left/Right and lights off
      if (commands[2] == HIGH && commands[3] == HIGH) {
        digitalWrite(lightPIN, LOW);
      } else if (commands[2] == HIGH && commands[3] == LOW) {
        steerLeft();
      } else if (commands[3] == HIGH && commands[2] == LOW) {
        steerRight();
      } else if (commands[2] == LOW && commands[3] == LOW) {       
        steerCenter();
      }      

      //printf("Commands: %i,%i,%i,%i\r\n", commands[0], commands[1], commands[2], commands[3]);
      char buf[256];
      snprintf(buf, sizeof buf, "[%i,%i,%i,%i]", commands[0], commands[1], commands[2], commands[3]);
      printlcd(buf);

      // Delay just a little bit to let the other unit make the transition to receiver
      delay(20);
    }

    // First, stop listening so we can talk
    radio.stopListening();    

    // Send the final one back.
    radio.write( &commands, commandsize );
    //printf("Sent response.\r\n");

    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }
  
  // Use this when connected to a bluetooth receiver
  while (Serial1.available() > 0) {
    
    char input = Serial1.read();
    Serial1.write(input);  
    Serial1.println("");
    
    if (input == 'l') {
      steerLeft(); 
      printlcd("Move left");
    } else if (input == 'r') {
      steerRight(); 
      printlcd("Move right");
    } else if (input == 'c') {
      steerCenter(); 
      printlcd("Center");
    } else if (input == 'f') {
      moveForward(); 
      printlcd("Move forward");      
    } else if (input == 'b') {
      moveBackward(); 
      printlcd("Move backward");      
    } else if (input == 's') {
      stopMoving();       
      printlcd("Stop moving");      
    }    
  }
  
 // delay(250);
}

void printlcd(String msg) {
  lcd.clear();
  lcd.home();
  lcd.print(msg);
}

/******************************************************************
  Checks if the robot is within the defined distance from an object
 ******************************************************************/
boolean nearobject() {  
  boolean result;  
  
  int dist = getdist();
  if (dist > 0 && dist < maxDistance) {
    result = true;
  } else {
    result = false;
  }  

  // Move the distanceservo to the next position  
  int servoposition = distanceservo.read();
  lcd.clear();
  lcd.home();
  lcd.print("servo pos.: ");
  lcd.print(servoposition);
  
  if (servoposition <= 50 || servoposition >= 140) {
	distanceservoIncr *= -1;
  }  
  if (!result) { 
    distanceservo.write(servoposition + distanceservoIncr);
  }
  delay(50);
  return result;
}

/**********************************
  Get's the distance from an object
 **********************************/
int getdist() {
  unsigned int uS = sonar.ping(); 		// Send ping, get ping time in microseconds (uS).
  int distcm = uS / US_ROUNDTRIP_CM; 	// Convert to cm
  //delay(50);
  return distcm;
}

void steerCenter() {
	steeringservo.write(115);  
}

void steerRight() {
	steeringservo.write(55);  
}

void steerLeft() {
	steeringservo.write(165);  
}

void moveForward() {
	// forward motor 
	digitalWrite(pinA2, LOW);
	analogWrite(pinA1, 50);
	delay(100);
	analogWrite(pinA1, 150);
	delay(100);
	analogWrite(pinA1, 255);
}

void moveBackward() {
	// backward motor 
	digitalWrite(pinA1, LOW);
	digitalWrite(pinA2, HIGH);
}

void stopMoving() {
	// stop motor 
	digitalWrite(pinA1, HIGH);
	digitalWrite(pinA2, HIGH);
}