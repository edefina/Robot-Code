// 
// MME 4487 Lab 4 Controller
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Michael Naish
//  Date:     2024 10 07
//

// #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

// Structs

// Control data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
  double speed;                                       // sends the reading of the speed potentiometer
  double leftright;                                   // sends the reading of the left-right 
  int gate;                                           // Reads the value for the gate to be open or closed
} __attribute__((packed)) esp_now_control_data_t;

// Drive data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
} __attribute__((packed)) esp_now_drive_data_t;

// Button structure
struct Button {
  const int pin;                                      // GPIO pin for button
  unsigned int numberPresses;                         // counter for number of button presses
  unsigned int lastPressTime;                         // time of last button press in ms
  bool pressed;                                       // flag for button press event
  bool state;                                         // current state of button; 0 = pressed; 1 = unpressed
  bool lastState;                                     // last state of button
};


// Function declarations
void doHeartbeat();
void failReboot();
void ARDUINO_ISR_ATTR buttonISR(void* arg);

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cStatusLED = 26;                            // GPIO pin of communication status LED
const int cDebounceDelay = 20;                        // button debounce delay in milliseconds
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop

// Variables
uint32_t lastHeartbeat = 0;                           // time of last heartbeat state change
uint32_t lastTime = 0;                                // last time of motor control was updated
uint32_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Button buttonPower = {14, 0, 0, false, true, true};   // On/Off button
Button buttonGate = {13, 0, 0, false, true, true};    // gate servo button
int Switch = 0;                                       // makes the button act as a switch, keeps track of presses
int Switch2 = 0;                                      // makes gate button act as a switch, keeps track of presses
int speed2 = 0;                                       // for reading the value of the speed potentiometer for trouble shooting
int leftright2 = 0;                                   // for reading the value of the left-right potentiometer for trouble shooting

// REPLACE WITH MAC ADDRESS OF YOUR DRIVE ESP32
uint8_t receiverMacAddress[] = {0x88,0x13,0xBF,0x25,0x46,0x00};  // MAC address of drive 00:01:02:03:04:05 
esp_now_control_data_t controlData;                   // data packet to send to drive system
esp_now_drive_data_t inData;                          // data packet from drive system

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
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller
#ifdef PRINT_INCOMING
    Serial.printf("%d\n", inData.time);
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
  Serial.print("MAC address for controller "); 
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  
  // Configure GPIO
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat as output
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output
  pinMode(buttonPower.pin, INPUT_PULLUP);             // configure GPIO for power button pin as an input with pullup resistor
  pinMode(buttonGate.pin, INPUT_PULLUP);              // configure GPIO for gate button pin as an input with pullup resistor =
  attachInterruptArg(buttonPower.pin, buttonISR, &buttonPower, CHANGE); // Configure power pushbutton ISR to trigger on change
  attachInterruptArg(buttonGate.pin, buttonISR, &buttonGate, CHANGE);   // Configure gate pushbutton ISR to trigger on change


  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
  }

  // add drive as peer
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the drive peer\n");
    failReboot();
  }
  else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                 receiverMacAddress[2], receiverMacAddress[3], 
                                                                 receiverMacAddress[4], receiverMacAddress[5]);
  }
  memset(&inData, 0, sizeof(inData));                 // clear drive data
  memset(&controlData, 0, sizeof(controlData));       // clear controller data
}

void loop() {

  uint32_t curTime = micros();                        // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    lastTime = curTime;
    controlData.time = curTime;                       // update transmission time
  
  if (!buttonPower.state) {                           // checks if the power button has been pressed
      Switch = !Switch;                               // keeps track of the last state of the button
  }
      if (Switch == 1) {                              // if the power button is "on"
      controlData.speed = analogRead(34);             // read the speed potentiometer and store it in controlData to be sent
      speed2 = analogRead(34);                        // for troubleshooting
      controlData.leftright = analogRead (32);        // read the left-right potentiometer and store it in controlData to be sent
      leftright2 = analogRead (32);                   // for troubleshooting
      digitalWrite(cStatusLED, 1);                    // turn on LED to show that the power button is "on"
      Serial.printf("Speed: %d\n", speed2);           // print the speed reading for troubleshooting
      Serial.printf("leftright: %d\n\n", leftright2); // print the left-right reading for troubleshooting
      
      if (!buttonGate.state){                         // if the gate button has been pressed
        Switch2 = !Switch2;
      }

        if (Switch2 == 1){                            // and if switch2 is set to 1, send a value of 1 to open the gate
          controlData.gate = 1;
          Serial.printf("Gate: Open");
        }
        else {                                        // for any other value of switch2, close the gate
          controlData.gate = 0;
          Serial.printf("Gate: Closed");
        }
      }

      else if (Switch == 0) {                         // if power is off
        digitalWrite(cStatusLED, 0);                  // turn off the led
        controlData.speed = 2047;                     // set speed to 2047 (which means the motors are not moving, as anything under 2047 is reverse)
        controlData.leftright = 2047;                 // set left right to 2047 which sets the motors to equal
        speed2 = 0;                                   // for troubleshooting
        leftright2 = 0;                               // for troubleshooting
        Serial.printf("Speed: %d\n", speed2);         // for troubleshooting
        Serial.printf("leftright: %d\n\n", leftright2); // for troubleshooting
        controlData.gate = 0;
      }

    if (commsLossCount > cMaxDroppedPackets) { // if drive appears disconnected, update control signal to stop before sending
      controlData.speed = 2047;            // if there is a disconnect, the last sent signal should be the no speed and equal motor values
      controlData.leftright = 2047;           
    } 
    //send data to driver
    if (peer->send_message((const uint8_t *) &controlData, sizeof(controlData))) {
      Serial.printf("Sent");                                 // for troubleshooting
    }
    else {
      Serial.printf("Issue sending");                        // for troubleshooting
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

// function to reboot the device
void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, allowing multiple
// instances of the buttonISR to be created (1 per button)
// implements software debounce and tracks button state
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              // cast pointer to static structure

  uint32_t pressTime = millis();                      // capture current time
  s->state = digitalRead(s->pin);                     // capture state of button
  // if button has been pressed and sufficient time has elapsed
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            // increment button press counter
    s->pressed = true;                                // set flag for "valid" button press
  }
  s->lastPressTime = pressTime;                       // update time of last state change
  s->lastState = s->state;                            // save last state
}

