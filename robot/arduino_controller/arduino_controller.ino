/*
 * RC Car Arduino Servo/ESC Controller
 * 
 * This firmware runs on an Arduino Uno and controls a servo and ESC
 * based on serial commands from the Raspberry Pi.
 * 
 * Serial Protocol:
 *   STEER:<pulse_us>    - Set steering servo to pulse width (µs)
 *   THROTTLE:<pulse_us> - Set throttle/ESC to pulse width (µs)
 *   STOP                - Set both to idle/neutral (1500 µs)
 * 
 * Note: PWM configuration (min/max values, steering center/range, etc.)
 * is now managed by the Python dashboard on the Raspberry Pi.
 * The Arduino simply receives pre-computed PWM values and applies them.
 * 
 * Baud Rate: 9600
 */

#include <Servo.h>

// Pin Configuration
#define STEERING_PIN 9
#define THROTTLE_PIN 10

// PWM Ranges (in microseconds)
#define STEERING_MIN 1000
#define STEERING_MAX 2000
#define PWM_NEUTRAL 1500
#define THROTTLE_IDLE 1500
#define THROTTLE_MAX 2000

// Servo objects
Servo steering_servo;
Servo throttle_esc;

// Current PWM values
uint16_t steering_pwm = PWM_NEUTRAL;
uint16_t throttle_pwm = PWM_NEUTRAL;

// Command buffer
const int BUFFER_SIZE = 32;
char command_buffer[BUFFER_SIZE];
int buffer_index = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Attach servos to PWM pins
  steering_servo.attach(STEERING_PIN);
  throttle_esc.attach(THROTTLE_PIN);
  
  // Initialize to neutral
  steering_servo.writeMicroseconds(PWM_NEUTRAL);
  throttle_esc.writeMicroseconds(PWM_NEUTRAL);
  
  Serial.println("Arduino ready. Awaiting commands...");
}

void loop() {
  // Check for incoming serial data
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Look for newline to signal end of command
    if (c == '\n' || c == '\r') {
      if (buffer_index > 0) {
        command_buffer[buffer_index] = '\0';  // Null terminate
        process_command(command_buffer);
        buffer_index = 0;
      }
    } else if (buffer_index < BUFFER_SIZE - 1) {
      // Add character to buffer
      command_buffer[buffer_index++] = c;
    }
  }
}

void process_command(const char* command) {
  // Parse command string
  
  if (strncmp(command, "STEER:", 6) == 0) {
    // Extract PWM value
    uint16_t pwm = parse_pwm_value(&command[6]);
    set_steering(pwm);
    
  } else if (strncmp(command, "THROTTLE:", 9) == 0) {
    // Extract PWM value
    uint16_t pwm = parse_pwm_value(&command[9]);
    set_throttle(pwm);
    
  } else if (strcmp(command, "STOP") == 0) {
    // Set both to idle/neutral
    set_steering(PWM_NEUTRAL);
    set_throttle(THROTTLE_IDLE);
    
  } else {
    // Unknown command (silent failure, or uncomment to debug)
    // Serial.print("Unknown: ");
    // Serial.println(command);
  }
}

uint16_t parse_pwm_value(const char* str) {
  // Parse integer from string, return as uint16_t
  uint16_t value = 0;
  while (*str >= '0' && *str <= '9') {
    value = value * 10 + (*str - '0');
    str++;
  }
  return value;
}

void set_steering(uint16_t pwm) {
  // Clamp to valid range
  pwm = constrain(pwm, STEERING_MIN, STEERING_MAX);
  
  // Update servo
  steering_servo.writeMicroseconds(pwm);
  steering_pwm = pwm;
  
  // Optional: log the change (comment out for silent operation)
  // Serial.print("STEER: ");
  // Serial.println(pwm);
}

void set_throttle(uint16_t pwm) {
  // Clamp to forward-only range
  pwm = constrain(pwm, THROTTLE_IDLE, THROTTLE_MAX);
  
  // Update ESC
  throttle_esc.writeMicroseconds(pwm);
  throttle_pwm = pwm;
  
  // Optional: log the change (comment out for silent operation)
  // Serial.print("THROTTLE: ");
  // Serial.println(pwm);
}
