// 
// MME 4487 Lab 4 Drive
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Michael Naish
//  Date:     2024 10 07
//

// #define SERIAL_STUDIO                                 // print formatted string, that can be captured and parsed by Serial-Studio
// #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros
#include "Adafruit_TCS34725.h"

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

// Structs

// Control data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
  double speed;                                       // to read potentiometer value
  double leftright;                                   // Reads the left right potentiometer
} __attribute__((packed)) esp_now_control_data_t;

// Drive data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
} __attribute__((packed)) esp_now_drive_data_t;

// Encoder structure
struct Encoder {
  const int chanA;                                    // GPIO pin for encoder channel A
  const int chanB;                                    // GPIO pin for encoder channel B
  int32_t pos;                                        // current encoder position
};

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void failReboot();
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cNumMotors = 2;                             // Number of DC motors
const int cIN1Pin[] = {17, 19};                       // GPIO pin(s) for INT1
const int cIN2Pin[] = {16, 18};                       // GPIO pin(s) for INT2
const int cPWMRes = 8;                                // bit resolution for PWM
const int cMinPWM = 0;                                // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;              // PWM value for maximum speed
const int cPWMFreq = 20000;                           // frequency of PWM signal
const int cCountsRev = 1096;                          // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                   // maximum encoder counts/sec
const int cMaxChange = 14;                            // maximum increment in counts/cycle
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const float cKp = 1.5;                                // proportional gain for PID
const float cKi = 0.2;                                // integral gain for PID
const float cKd = 0.8;                                // derivative gain for PID

// Variables
uint32_t lastHeartbeat = 0;                           // time of last heartbeat state change
uint32_t lastTime = 0;                                // last time of motor control was updated
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position
                     {32, 33, 0}};                    // encoder 1 on GPIO 32 and 33, 0 position
int32_t target[] = {0, 0};                            // target encoder count for motor
int32_t lastEncoder[] = {0, 0};                       // encoder count at last control cycle
float targetF[] = {0.0, 0.0};                         // target for motor as float
int servo;                                            // holds the servo value before it goes through duty cycle function

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = {0xAC,0x15,0x18,0xD5,0x0D,0x1C};  // MAC address of controller 00:01:02:03:04:05
esp_now_control_data_t inData;                        // control data packet from controller
esp_now_drive_data_t driveData;                       // data packet to send to controller

// Classes
//
// The ESP_NOW_NetworkPeer class inherits from ESP_NOW_Peer and implements the _onReceive and _onSent methods.
// For more information about the ESP_NOW_Peer class, see the ESP_NOW_Peer class in the ESP32_NOW.h file.
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Network_Peer(const uint8_t *mac_addr, const uint8_t *lmk = NULL)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk) {}

  ~ESP_NOW_Network_Peer() {}

  bool begin() {
    // Assumes that the ESP-NOW protocol is already initialized
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }
    // Call the parent class method to send the data
    return send(data, len);
  }

  // callback function for when data is received
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == 0)                                       // if empty packet
    {
      return;                                           // return
    }
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller+

#ifdef PRINT_INCOMING
    Serial.printf("%d, %d\n", inData.dir, inData.time);
#endif
  }

  
  // callback function for when data is sent
  void onSent(bool success) {
    if (success) {
#ifdef PRINT_SEND_STATUS
      log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
#endif
      commsLossCount = 0;
    }
    else {
      commsLossCount++;
    }
  }
};

// Peers
ESP_NOW_Network_Peer *peer;

void setup() {
  Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  while (!Serial) {                                   // wait for Serial to start
    delay(10);                                        // okay to delay during setup
  }
  WiFi.mode(WIFI_STA);                                // use WiFi in station mode
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               // set WiFi channel to use with peer
  while (!WiFi.STA.started()) {                       // wait for WiFi to start
    delay(100);                                       // okay to delay during setup
  }
  Serial.print("MAC address for drive "); 
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  
  ledcAttach(cServoPin, 50, 16);                      // setup servo pin for 50 Hz, 16-bit resolution
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);        // setup INT1 GPIO PWM channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);        // setup INT2 GPIO PWM channel
    pinMode(encoder[k].chanA, INPUT);                 // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                 // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
  }
  
  // add controller as peer
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the controller peer\n");
    failReboot();
  }
  else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                 receiverMacAddress[2], receiverMacAddress[3], 
                                                                 receiverMacAddress[4], receiverMacAddress[5]);
  }
  memset(&inData, 0, sizeof(inData));                 // clear controller data
  memset(&driveData, 0, sizeof(driveData));           // clear drive data
}

  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

void loop() {


  float deltaT = 0;                                   // time interval
  int32_t pos[] = {0, 0};                             // current motor positions
  int32_t e[] = {0, 0};                               // position error
  float velEncoder[] = {0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0};                         // change in position for set speed
  float ePrev[] = {0, 0};                             // previous position error
  float dedt[] = {0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0};                         // integral of error 
  float u[] = {0, 0};                                 // PID control signal
  int pwm[] = {0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1};                                 // direction that motor should turn
  int lrcheck;                                        // left right check
  int servo = 90;                                     // start with the servo motor in middle position
  ledcWrite(cServoPin, degreestodutycycle(servo));
 
 
  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
   if (commsLossCount > cMaxDroppedPackets) {
    failReboot();
  }


  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                     // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          // read and store current motor position
  }
  interrupts();                                       // turn interrupts back on

  uint32_t curTime = micros();                        // capture current time in microseconds

  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    if(r>120&&g>90&&g<115&&b>90&&b<115){              //if colour is within range
      servo = 0;
      ledcWrite(cServoPin, degreestodutycycle(servo));
    }
    else if (r>120&&g>90&&g<115&&b>90&&b<115){        // if the colour sensor detects white background of the 3d printed wall
      servo = 90;
      ledcWrite(cServoPin, degreestodutycycle(servo));
    }
    else {                                            // if the marble is wrong colour, go to the right
      servo = 180;
      ledcWrite(cServoPin, degreestodutycycle(servo));
    }


  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle
    driveData.time = curTime;                         // update transmission time
    for (int k = 0; k < cNumMotors; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      if (inData.leftright<=2046) {                                          // if leftright is less than 2046, we want to turn left
        if (inData.speed<=2046) {                                            // if the speed is less than 2046, we want to go in reverse 
          pwm[1] = map(inData.speed,2046,0,cMinPWM,cMaxPWM);                 // map read speed from potentiometer to be used by setmotor
          pwm[0] = map(inData.leftright,0,2046,cMinPWM, pwm[1]);             // map the left motor to scale between 0 and the other motor speed depending on how far the leftright potentiometer is turned 
          dir[1]=1;                                                          // because one motor is upsidedown compared to the other, motor [0] must be -1 and [1] must be 1 for it to drive reverse
          dir[0]=-1;

      }
        else if (inData.speed>=2048){                                        // if speed is greater than 2048 we want to go forward
          pwm[1] = map(inData.speed,2048,4095,cMinPWM,cMaxPWM);              // map read speed from potentiometer to be used by setmotor
          pwm[0] = map(inData.leftright,0,2046,cMinPWM, pwm[1]);             // map the left motor to scale between 0 and right motor speed depending on how far leftright potentiometer is turned
          dir[0]=1;                                                          // because one motor is upsidedown compared to the other, motor [0] must be 1 and [1] must be -1 for it to drive forward
          dir[1]=-1;  
        }
        else if (inData.speed=2047) {                                        // if speed dial is right in the middle, dont move at all.
          pwm[0] = 0;
          pwm[1] = 0;
          dir[0] = 0;
          dir[1] = 0;
        }
      }

      else if (inData.leftright>=2048){                                            // if leftright dial is greater than 2048, we want to turn right 
          if (inData.speed>=2048){                                                 // speed is greater than 2048, so we move forward
             pwm[0] = map(inData.speed,2048,4095,cMinPWM,cMaxPWM);                 // map the speed from min to max pwm depending on how far right the speed dial is 
             pwm[1] = map(inData.leftright,2048,4095,pwm[0], 0);                   // map the right motor to how far the leftright dial is turned right. Match it with the left motor as an extreme 
             dir[0]=1;                                                             // these directions correspond with moving forward
             dir[1]=-1; 
          }
          else if (inData.speed<=2046){                                            // moving backwards
             pwm[0] = map(inData.speed,2046,0,cMinPWM,cMaxPWM);                    // map the speed of left motor 
             pwm[1] = map(inData.leftright,2048,4095,pwm[0], 0);                   // map the right motor to range between 0 and the left motor's speed
             dir[1]=1;                                                             // these directions correspond with reverse
             dir[0]=-1; 
          }
          else if (inData.speed=2047){                                             // if speed dial is set to 2047, set all pwm and dir to 0
            pwm[0]=0;
            pwm[1]=0;
            dir[0]=0;
            dir[1]=0;
          }
        }

      else if (inData.leftright = 2047) {                                          // if leftright dial is in the middle, make the two motors equivelent
      pwm[1] = pwm[0];             // if no potentiometer value, use pwm as usual
      }

      // update target for set direction
      posChange[k] = (float) (dir[0] * cMaxChange); // update with maximum speed
      targetF[k] = targetF[k] + posChange[k];         // set new target position
      if (k == 0) {                                   // assume differential drive
        target[k] = (int32_t) targetF[k];             // motor 1 spins one way
      }
      else {
        target[k] = (int32_t) -targetF[k];            // motor 2 spins in opposite direction
      }

      // use PID to calculate control signal to motor
      e[k] = target[k] - pos[k];                      // position error
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    // derivative of error
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    // integral of error (finite difference)
      u[k] = cKp * e[k] + cKd * dedt[k] + cKi * eIntegral[k]; // compute PID-based control signal
      ePrev[k] = e[k];                                // store error for next control cycle
  
      // set speed based on computed control signal
      u[k] = fabs(u[k]);                              // get magnitude of control signal
      if (u[k] > cMaxSpeedInCounts) {                 // if control signal will saturate motor
        u[k] = cMaxSpeedInCounts;                     // impose upper limit
      }

      if (commsLossCount < cMaxDroppedPackets / 4) {
        setMotor(dir[k], pwm[k], cIN1Pin[k], cIN2Pin[k]); // update motor speed and direction
          Serial.printf("PWM[0] = %d and PWM[1] = %d \n", pwm[0], pwm[1]);          // for troubleshooting
          Serial.printf("Dir[0] = %d and Dir[1] = %d \n", dir[0], dir[1]);          // for troubleshooting
        }
      else {
        setMotor(0, 0, cIN1Pin[k], cIN2Pin[k]);       // stop motor
      }

#ifdef SERIAL_STUDIO
      if (k == 0) {
        printf("/*");                                 // start of sequence for Serial Studio parsing
      }
      printf("%d,%d,%d,%0.4f", target[k], pos[k], e[k], velMotor[k]);  // target, actual, error, velocity
      if (k < cNumMotors - 1) {
        printf(",");                                  // data separator for Serial Studio parsing
      }
      if (k == cNumMotors -1) {
        printf("*/\r\n");                             // end of sequence for Serial Studio parsing
      }
#endif
  }
  }
  doHeartbeat();                                      // update heartbeat LED
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

// function to reboot the device
void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}


long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle
#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0015259;              // dutyCycle / 65535 * 100
  
  //Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif
  return dutyCycle;

}


// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
// triggered by rising edge of channel A
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);            // cast pointer to static structure
  
  int b = digitalRead(s->chanB);                      // read state of channel B
  if (b > 0) {                                        // B high indicates that it is leading channel A
    s->pos++;                                         // increase position
  }
  else {                                              // B low indicates that it is lagging channel A
    s->pos--;                                         // decrease position
  }
}

