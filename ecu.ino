/**
 * The software behind an electronic engine control system for the
 * Predator 79cc small engine.
 *
 * Bill of materials:
 *   - 1x 2 inch by 3 inch PCB
 *   - 1x 5v buck converter
 *   - 1x 28 pin chip socket
 *   - 1x atmega328 microcontroller
 *   - 1x 16mhz oscillating crystal
 *   - 2x 10k pull-up resistors
 *   - 1x 3144 hall effect sensor
 *   - 1x 10 micro-farad capacitor
 *   - 2x 22 picofarad capacitors
 *   - 2x potentiometers
 *   - 2x servos
 */
#include <Servo.h>

/**
 * A debug flag, useful for bench testing circuitry and components.
 */
#define DEBUG 1

/**
 * The number of microseconds per minute.
 */
#define MICROSECONDS_PER_MINUTE 60000000.0

/**
 * The maximum rotations per minute before rev limiting kicks in.
 *
 * This value can be increased for engines that have had the appropriate
 * upgrades applied to them (governor removed internally, upgraded flywheel,
 * heavier valve springs, etc...).
 */
#define MAX_ENGINE_SPEED 3600

/**
 * Analog input pin number for the throttle potentiometer.
 *
 * Note - this pin must be one of A0, A1, A2, A3, A4, or A5 because those are
 * the only analog pins available on the atmega328 chip.
 */
#define THROTTLE_ANALOG_IN_PIN A0

/**
 * Analog input pin number for the choke potentiometer.
 *
 * Note - this pin must be one of A0, A1, A2, A3, A4, or A5 because those are
 * the only analog pins available on the atmega328 chip.
 */
#define CHOKE_ANALOG_IN_PIN A2

/**
 * Digital input pin number for the tachometer.
 *
 * Note - this pin must be interrupt compatible. That corresponds to digital
 * pins 2 or 3 because those are the only interrupt-compatible pins that map
 * to the atmega328 chip.
 */
#define TACHOMETER_DIGITAL_IN_PIN 3

/**
 * Pulse width modulated output pin for the throttle servo.
 *
 * Note - this pin must be PWM compatible. That corresponds to digital pins 3,
 * 5, 6, 9, 10, or 11 because those are the only PWM-compatible pins that map
 * to the atmega328 chip.
 */
#define THROTTLE_PWM_OUT_PIN 9

/**
 * Pulse width modulated output pin for the choke servo.
 *
 * Note - this pin must be PWM compatible. That corresponds to digital pins 3,
 * 5, 6, 9, 10, or 11 because those are the only PWM-compatible pins that map
 * to the atmega328 chip.
 */
 #define CHOKE_PWM_OUT_PIN 11

/**
 * Digital output pin number for the rev limiter relay.
 *
 * Note - this pin must be one of 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, or 13
 * because those are the only digital pins available on the atmega328 chip
 * that are not used for other purposes.
 */
#define REV_LIMITER_DIGITAL_OUT_PIN 7

/**
 * The minimum position of the throttle servo.
 *
 * Note - if an upgraded carburator is installed, this value may have to be
 * tweaked.
 */
#define THROTTLE_SERVO_MIN_POSITION 0

/**
 * The maximum position of the throttle servo.
 *
 * Note - if an upgraded carburator is installed, this value may have to be
 * tweaked.
 */
#define THROTTLE_SERVO_MAX_POSITION 90

/**
 * The minimum position of the choke servo.
 *
 * Note - if an upgraded carburator is installed, this value may have to be
 * tweaked.
 */
#define CHOKE_SERVO_MIN_POSITION 0

/**
 * The maximum position of the choke servo.
 *
 * Note - if an upgraded carburator is installed, this value may have to be
 * tweaked.
 */
#define CHOKE_SERVO_MAX_POSITION 90

/**
 * The servo controlling the engine throttle position.
 */
Servo throttle;

/**
 * The servo controlling the engine choke position.
 */
 Servo choke;

/**
 * The current throttle position command.
 *
 * Note - this value is mapped to the range defined by
 * THROTTLE_SERVO_MIN_POSITION to THROTTLE_SERVO_MAX_POSITION.
 */
uint8_t throttle_command = 0;

/**
 * The current choke position command.
 *
 * Note - this value is mapped to the range defined by
 * CHOKE_SERVO_MIN_POSITION to CHOKE_SERVO_MAX_POSITION.
 */
 uint8_t choke_command = 0;

/**
 * The current engine speed.
 */
uint32_t engine_speed = 0;

/**
 * Is the engine being rev limited presently?
 */
bool is_rev_limiting = false;

/**
 * The time (in microseconds) that the last engine speed reading was taken.
 */
volatile unsigned long last_reading = 0;

/**
 * The microseconds between the current reading and last reading.
 */
volatile unsigned long last_duration = 0;

/**
 * A flag indicating that a new engine speed sample is ready.
 */
volatile bool resampled = false;

/**
 * Interrupt routine for when the magnet trips the hall effect sensor.
 */
void onHallEffectSensorTrip() {
  unsigned long now = micros();
  if (now - last_reading > 10000) {
    last_duration = now - last_reading;
    last_reading = now;
    resampled = true;
  }
}

/**
 * Reads the analog throttle input and adjusts the throttle servo accordingly.
 */
void resample_throttle_command() {
  // The q3atb54usn potentiometer has a range of 0 to 1024, but the stepper
  // motor that it's controlling only has 90 steps, so we re-map the value
  // to make the scale match.
  uint8_t new_throttle_command = map(
    ((analogRead(THROTTLE_ANALOG_IN_PIN)) + 1), 
    0, 
    1024, 
    0, 
    90
  );

  // In practice the potentiometer is too sensitive and causes the value to
  // jump around even at rest, so to prevent unnecessary wear and tear on the
  // stepper motor, we make sure that it takes a larger change to issue a new
  // command to the stepper.
  if(abs(new_throttle_command - throttle_command) > 1) {
    throttle_command = new_throttle_command;
    throttle.write(throttle_command);
    #ifdef DEBUG
      Serial.print("New throttle command: ");
      Serial.println(new_throttle_command);
    #endif
  }
}

/**
 * Reads the analog choke input and adjusts the choke servo accordingly.
 */
void resample_choke_command() {

  // The q3atb54usn potentiometer has a range of 0 to 1024, but the stepper
  // motor that it's controlling only has 255 steps, so we re-map the value
  // to make the scale match.
  uint8_t new_choke_command = map(
    ((analogRead(CHOKE_ANALOG_IN_PIN)) + 1), 
    0, 
    1024, 
    0, 
    90
  );

  // In practice the potentiometer is too sensitive and causes the value to
  // jump around even at rest, so to prevent unnecessary wear and tear on the
  // stepper motor, we make sure that it takes a larger change to issue a new
  // command to the stepper.
  if(abs(new_choke_command - choke_command) > 1) {
    choke_command = new_choke_command;
    choke.write(choke_command);
    #ifdef DEBUG
      Serial.print("New choke command: ");
      Serial.println(choke_command);
    #endif
  }
}

/**
 * Reads the tachometer input and adjusts the engine speed variable.
 */
void resample_engine_speed() {

  if (resampled) {
    detachInterrupt(digitalPinToInterrupt(TACHOMETER_DIGITAL_IN_PIN));
    unsigned long duration = last_duration;
    resampled = false;
    attachInterrupt(digitalPinToInterrupt(TACHOMETER_DIGITAL_IN_PIN), onHallEffectSensorTrip, FALLING);

    if (duration) {
      engine_speed = (MICROSECONDS_PER_MINUTE / duration);
      #ifdef DEBUG
        Serial.print("Engine speed: ");
        Serial.println(engine_speed);
      #endif
    }
  }
  // Ground the spark if the engine is going too fast.
  if (engine_speed > MAX_ENGINE_SPEED && !is_rev_limiting) {
    digitalWrite(REV_LIMITER_DIGITAL_OUT_PIN, HIGH);
    is_rev_limiting = true;
    #ifdef DEBUG
      Serial.print("Engine exceeded ");
      Serial.print(MAX_ENGINE_SPEED);
      Serial.print(" (");
      Serial.print(engine_speed);
      Serial.println(") - rev limiting applied");
    #endif
  }
  // Unground the spark if the engine isn't redlining.
  else if (engine_speed <= MAX_ENGINE_SPEED && is_rev_limiting) {
    digitalWrite(REV_LIMITER_DIGITAL_OUT_PIN, LOW);
    is_rev_limiting = false;
    #ifdef DEBUG
      Serial.print("Engine dropped below ");
      Serial.print(MAX_ENGINE_SPEED);
      Serial.print(" (");
      Serial.print(engine_speed);
      Serial.println(") - rev limiting disabled");
    #endif
  }
}

/**
 * Initializes pin mappings, interrupts, I/O, and such.
 */
 void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  // Control for the choke servo.
  choke.attach(CHOKE_PWM_OUT_PIN);

  // Control for the throttle servo.  
  throttle.attach(THROTTLE_PWM_OUT_PIN);

  // Input for the tachometer.
  pinMode(TACHOMETER_DIGITAL_IN_PIN, INPUT_PULLUP);

  // Control for the governor.
  pinMode(REV_LIMITER_DIGITAL_OUT_PIN, OUTPUT);

  // Interrupt routine for measuring engine speed.
  attachInterrupt(digitalPinToInterrupt(TACHOMETER_DIGITAL_IN_PIN), onHallEffectSensorTrip, FALLING);
}

/**
 * Engine control loop.
 */
void loop() {
  resample_throttle_command();
  resample_choke_command();
  resample_engine_speed();
}
