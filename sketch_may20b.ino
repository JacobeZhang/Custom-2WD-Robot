#include <ESP32Encoder.h>
#include <math.h>
#include <Arduino.h>

// IMU INITIALIZATIONS
#include "Adafruit_BNO08x_RVC.h"
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
#define RXD2 16
#define TXD2 17
BNO08x_RVC_Data heading;
bool IMU_working = false;
int IMU_fail_count = 0;

int encoderL_A = 34;
int encoderL_B = 35;
int encoderR_A = 26;
int encoderR_B = 27;

ESP32Encoder encoderL;
ESP32Encoder encoderR;

int EN_L = 23;
int IN_1_L = 18;
int IN_2_L = 19;

int EN_R = 25;
int IN_1_R = 33;
int IN_2_R = 32;

// Setting PWM properties
const int freq = 80000;
const int resolution = 8;
int dutyCycle = 200;

double wheelCircumference = 7.2 * M_PI;
double ticksPerCm = 1437.09 / wheelCircumference;

int BUTTON = 4;

int count = 0;

#define LED 2
#define RST_IMU 22

void setup(){
	
	Serial.begin(115200);
  // SET UP IMU STUFF
	while (!Serial)
    delay(10);

  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2); // This is the baud rate specified by the datasheet
  while (!Serial1)
    delay(10);

  if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
    while (1)
      delay(10);
  }

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 19 and 18 for the first encoder
	encoderL.attachFullQuad(encoderL_A, encoderL_B);
	// use pin 17 and 16 for the second encoder
	encoderR.attachFullQuad(encoderR_A, encoderR_B);
		
	// set starting count value after attaching
	encoderL.setCount(0);
  encoderR.setCount(0);

  pinMode(IN_1_L, OUTPUT);
  pinMode(IN_2_L, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(IN_1_R, OUTPUT);
  pinMode(IN_2_R, OUTPUT);
  pinMode(EN_R, OUTPUT);

  ledcAttachChannel(EN_L, freq, resolution, 0);
  ledcAttachChannel(EN_R, freq, resolution, 1);

  pinMode(BUTTON, INPUT);

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  pinMode(RST_IMU, OUTPUT);
  digitalWrite(RST_IMU, HIGH);

}

void loop(){
  if (digitalRead(BUTTON) == HIGH) {
    delay(5000);

    moveDist(50, 2.5);
    turn(true, 2.5); // TURN RIGHT
    moveDist(50, 2.5);
    turn(false, 2.5); // TURN LEFT
    moveDist(50, 2.5);
    moveDist(-50, 2.5);
    turn(true, 2.5); // TURN RIGHT
    moveDist(-50, 2.5);
    turn(false, 2.5); // TURN LEFT
    moveDist(-50, 2.5);
    
    /*
    moveDist(25 + 9.5, 2.5); // Correction Start

    moveDist(25, 2);
    moveDist(50, 2.5);
    turn(true, 1); // TURN RIGHT
    turn(false, 1); // TURN LEFT

    moveDist(25, 2); // INTO GATE ZONE 
    moveDist(-25, 2); // OUT OF GATE ZONE

    moveDist(50 - 9.5, 3); // Correction End
    */

  } else {
    brake();
    
    // Print out telemetry every once in a while
    count++;
    if (count > 100000) {
      printTicks();
      if (IMU_working) {
        Serial.print("Yaw: ");
        Serial.print(heading.yaw);
        Serial.println(" ");
      }
      count = 0;
    }

    // Read IMU data
    if (rvc.read(&heading)) {
      // IMU data is available
      IMU_working = true;
      IMU_fail_count = 0;
      digitalWrite(LED,HIGH);
    }

    // Code below is necessary
    // Basically, if the IMU ever stops outputting data (sometimes doesn't output data on startup), reset the IMU
    else {
      IMU_fail_count++;
      if (IMU_fail_count > 100000) {
        digitalWrite(RST_IMU, LOW);
        delay(1000);
        digitalWrite(RST_IMU, HIGH);
        IMU_fail_count = 0;
        Serial.println("IMU has been reset.");
      }
    }
  }
}