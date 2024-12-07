

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros
#include "Adafruit_TCS34725.h"

void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cPWMRes = 8;                                // bit resolution for PWM
const int cMinPWM = 0;                                // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;              // PWM value for maximum speed
const int cPWMFreq = 20000;                           // frequency of PWM signal
const int cServoPin1 = 15;                            // GPIO Pin for sorting servo motor
const int cTCSLED = 23;                               // GPIO pin for LED on TCS34725
const long cMinDutyCycle = 1650;                      // duty cycle for 0 degrees 
const long cMaxDutyCycle = 8175;                      // duty cycle for 180 degrees 
int servo1 = 90;                                      // sorting servo, start in neutral middle position
bool servoMoved = false;                              // Flag to check if the servo has moved
uint32_t lastHeartbeat = 0;                           // time of last heartbeat state change
uint32_t lastTime = 0;                                // last time of motor control was updated
uint32_t prevTime = 0;                                // last time of the sorting slider update
const int cINOUTPin[] = {17, 19};
const int cINOUTPin2[]= {16,18};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // start with the tcs flag at 0

void setup() {

Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  while (!Serial) {                                   // wait for Serial to start
    delay(10);                                        // okay to delay during setup
  }
  // put your setup code here, to run once:
  ledcAttach(cServoPin1, 50, 16);                      // setup servo pin for 50 Hz, 16-bit resolution
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO for control of LED on TCS34725

  for (int k = 0; k < 2; k++) {
    ledcAttach(cINOUTPin[k], cPWMFreq, cPWMRes);          // setup INT1 GPIO PWM channel
    ledcAttach(cINOUTPin2[k], cPWMFreq, cPWMRes);
  }


  if (tcs.begin()) {                                  // if the tcs sensor successfully initializes
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;                                   // set flag to true
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {                                                              //otherwise set flag to false and display message
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {

  setMotor(1, cMaxPWM, 17, 19);
  setMotor(-1, 255, 16, 18);

 uint16_t r, g, b, c;                                // RGBC values from TCS34725

  
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
   
   if(r>19&&r<45&&g>35&&g<63&&b>26&&b<53) {                                   //if colour is within range
      if (servoMoved == false) {
      servo1 = 0;                                                             // send marble to container
      ledcWrite(cServoPin1, degreesToDutyCycle(servo1));
      prevTime = millis();                                                    // track time to reset flag later on instead of using delay function
      servoMoved = true;                                                      // set servoMoved flag to true so that the motor won't be moved until a set time has passed 
      Serial.printf("Good Marble Deposit");
      }
    }
    else if (r>207&&r<240&&g>266&&g<315&&b>220&&b<275&&c>700&&c<820) {        // if the colour sensor detects white background of the 3d printed wall when it is pressed against the sensor
      if (servoMoved == false) {
      servo1 = 90;                                                            // keep it centered
      ledcWrite(cServoPin1, degreesToDutyCycle(servo1));
      prevTime = millis();
      servoMoved = true;
      Serial.printf("Done Depositing, center");
      }
    }
    else if (r>40&&r<60&&g>52&&g<66&&b>45&&b<60&&c>160&&c<190) {        // if the colour sensor detects white background of the 3d printed wall when it is centered and empty
      if (servoMoved == false) {
      servo1 = 90;                                                      // keep it centered
      ledcWrite(cServoPin1, degreesToDutyCycle(servo1));
      prevTime = millis();
      servoMoved = true;
      Serial.printf("Empty, center");
      }
    }
    else {                                                              // if the marble is wrong colour, dispose off the side of chassis
      if (servoMoved == false) {
      servo1 = 120;
      ledcWrite(cServoPin1, degreesToDutyCycle(servo1));
      prevTime = millis();
      servoMoved = true;
      Serial.printf("Bad Marble, Dispose");
     }
    }    
    
    if (servoMoved && millis() - prevTime >= 500) {                        // reset the servoMoved flag if 0.5 seconds have passed
        servoMoved = false;
    }

  } 
  #ifdef PRINT_COLOUR    
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);               // for troubleshooting and calibrating
#endif
  doHeartbeat();
}


// blink heartbeat LED
void doHeartbeat() {
  uint32_t curMillis = millis();                      // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                     // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else{                                              // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}


long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle
#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0015259;              // dutyCycle / 65535 * 100
  
  //Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif
  return dutyCycle;

}
