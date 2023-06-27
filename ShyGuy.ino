/**
 * "Shy Guy", a character for dynamic escape room player interaction
 * Copyright (c) 2023 Playful Technology
 */

// DEFINES
// Helper function to get length of an arbitrary array
#define ArrayLength(x) (sizeof(x) / sizeof((x)[0]))

// STEPS_PER_MM is the number of steps required to move one mm
// Calculated as (Steps per turn * Microstepping) / (Belt Tooth Pitch * Pulley Teeth)
// For a NEMA17 with GT2 timing belt, 80 (with 1/16th microstepping) = 5 full steps
// More info at https://www.circuitist.com/how-to-calculate-steps-per-mm-lead-screw-and-gt2-timing-belt/
#define STEPS_PER_MM 5

// What sort of shield is being used to wire the components to the Arduino? 
// #define USE_CNCSHIELD (UNO) or #define USE_RAMPS (Mega)
#define USE_CNCSHIELD

// Pin definitions for Arduino MEGA with RAMPS v1.4 shield
// See https://github.com/MarlinFirmware/Marlin/blob/2.0.x/Marlin/src/pins/ramps/pins_RAMPS.h
#ifdef USE_RAMPS
  #define X_STEP_PIN         54
  #define X_DIR_PIN          55
  #define X_ENABLE_PIN       38
  #define Z_STEP_PIN         46
  #define Z_MAX_PIN          19
  #define Y_MAX_PIN          15
  #define X_MAX_PIN           2
#endif
// Pin definitions for Arduino UNO with CNC Shield v3
// see https://wiki.keyestudio.com/Ks0160_keyestudio_CNC_Shield_V3
// see https://blog.protoneer.co.nz/arduino-cnc-shield-v3-00-assembly-guide/
#ifdef USE_CNCSHIELD
  #define X_STEP_PIN          2
  #define X_DIR_PIN           5
  #define X_ENABLE_PIN        8
  #define Z_STEP_PIN          4
  #define Z_MAX_PIN          11 
  #define Y_MAX_PIN          10
  #define X_MAX_PIN           9
#endif

// INCLUDES
// See: http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>

// CONSTANTS
// Duration (in ms) for which the hatch stays open before automatically closing again
const int openDuration = 2000;
const byte knockSensorPin = Z_MAX_PIN;
const byte ledPins[] = {X_MAX_PIN, Y_MAX_PIN};
// If time between successive knocks is shorter than minKnockDelay, it will be assumed to be a continuing "bounce" of the preivous knock
// and not counted  
const uint8_t minKnockDelay = 50; //ms
// If time between successive knocks exceeds maxKnockDelay, they will not be counted as part of the same knock pattern
const uint8_t maxKnockDelay = 1000; //ms

// GLOBALS
// Define each stepper and the pins it will use
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
// The timestamp at which the hatch was last opened
unsigned long lastMoveTime;
// States in which the controller can be
typedef enum { Closed, Opening, Open, Closing } State;
// Assume that the hatch starts closed
State currentState = State::Closed;
unsigned long lastKnockTime;
int numKnocks = 0;

// Initial setup
void setup() {
  // Start a serial connection
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  Serial.println(F("Configuring input"));
  pinMode(knockSensorPin, INPUT_PULLUP);

  Serial.println("F(Configuring outputs"));
  for(int i=0; i<2; i++){
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  // Note: if using stepper drivers (A4988 etc.), these need to be enabled by pulling the EN pin low. 
  // On some shields, the pin is automatically pulled to GND via a jumper. If not, write a LOW signal to 
  // whatever pin is connected to the enable pin of the chip. 
  pinMode(X_ENABLE_PIN, OUTPUT);  digitalWrite(X_ENABLE_PIN, LOW);
  // Stepper motor speed (steps/sec) and acceleration (steps per second^2)
  stepperX.setMaxSpeed(3000);
  stepperX.setAcceleration(8000); 
  stepperX.setMinPulseWidth(25);
}

// move() specifies a relative movement for the hatch, in mm, from current position
void move(int distance){
  stepperX.move(distance * STEPS_PER_MM);
}

// moveTo() specifies an absolute position (mm) (with 0 being the position of the motor when first powered up)
void openHatch(){
 stepperX.moveTo(-120 * STEPS_PER_MM);
}

// moveTo() specifies an absolute position (mm) (with 0 being the position of the motor when first powered up)
void closeHatch(){
 stepperX.moveTo(0);
}

void loop() {
  // Record the current timestamp
  unsigned long now = millis();
  
  // Action depends on current state
  switch(currentState) {
    case State::Closed:
      // Check if anyone's knocked on the door
      if(!digitalRead(knockSensorPin)){
          // How long has it been since the last input?
          unsigned long delta = now - lastKnockTime;
          // if this knock is too close to the last one, it's just a bounce. Ignore it.
          if(delta < minKnockDelay) {;}
          // If it's outside the debounce filter, and still within the allowed time fo successive knocks, it's a valid knock
          else if(delta > minKnockDelay && delta < maxKnockDelay) { numKnocks++; }
          // We're outside the time for successive knocks, so start a new counter
          else if(delta > maxKnockDelay) { numKnocks = 1; }
          lastKnockTime = now;
      }
      // If no new knocks have been received within the allowed time, reset the counter
      if(numKnocks > 0 && (now - lastKnockTime > maxKnockDelay)) { numKnocks = 0; }
      // If at least 3 separate knocks have been received within the allotted time
      if(numKnocks >= 3) {
        Serial.println("Opening Hatch");
        numKnocks = 0;
        // Enable the stepper motor
        digitalWrite(X_ENABLE_PIN, LOW);
        // Open the hatch!
        openHatch();
        currentState = State::Opening;  
      }
      break;

    case State::Opening:
      // If the carriage has arrived at its destination
      if(stepperX.distanceToGo() == 0){
        Serial.println("Hatch Open");
        // Disable the motor
        digitalWrite(X_ENABLE_PIN, HIGH);
        currentState = State::Open;
        // Record the timestamp at which the shutter opened
        lastMoveTime = millis();
        // Light up the eyes
        for(int i=0; i<2; i++){
          analogWrite(ledPins[i], 50);
        }
      } 
      break;

    case State::Open:
      // If the hatch has been open for the specified duration
      if((millis() - lastMoveTime) > openDuration) {
        Serial.println("Closing Hatch");
        // Enable the motor
        digitalWrite(X_ENABLE_PIN, LOW);
        // Move back to the starting position
        closeHatch();
        currentState = State::Closing;
      }
      break;

    case State::Closing:
      // If the carriage arrives back at its origin
      if(stepperX.distanceToGo() == 0){
        Serial.println("Hatch Closed");
        // Disable the motor
        digitalWrite(X_ENABLE_PIN, HIGH);
        // Turn off the eye LEDs
        for(int i=0; i<2; i++) { digitalWrite(ledPins[i], LOW); }
        currentState = State::Closed;
        delay(100);
      } 
      break;
  }
  // This gets called every frame, and processes any movement of the steppers as necessary
  stepperX.run();
}