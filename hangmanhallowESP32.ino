// Using Adafruit ESP32 Feather V2 board

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

// Define PWM pins used by the wing (change if you use different channels)
#define SERVO_X_PIN 0   // PWM channel 0
#define SERVO_Y_PIN 1   // PWM channel 1

// Audio FX board pins
#define AUDIO_TRIGGER_X 5   // GPIO5
#define AUDIO_TRIGGER_Y 6   // GPIO6

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default address 0x40

// Servo limits (in degrees)
const int SERVO_MIN = 60;   // 60° corresponds to 90° on PWM (≈ -30° spine)
const int SERVO_MAX = 120;  // 120° corresponds to 90° on PWM (+30° spine)
const int SERVO_NEUTRAL = 90; // 90° centre

void setup() {
  Serial.begin(115200);
  pwm.begin();             // init PWM driver
  pwm.setPWMFreq(50);      // 50 Hz for servos

  pinMode(AUDIO_TRIGGER_X, OUTPUT);
  pinMode(AUDIO_TRIGGER_Y, OUTPUT);
  digitalWrite(AUDIO_TRIGGER_X, HIGH);   // pull‑up to avoid accidental trigger
  digitalWrite(AUDIO_TRIGGER_Y, HIGH);
}

void loop() {
  // Pick a random pause (5‑15 s)
  unsigned long pause = random(5000, 15000);
  delay(pause);

  // Random X angle between limits
  int xAngle = random(SERVO_MIN, SERVO_MAX + 1);
  // Random Y angle between limits
  int yAngle = random(SERVO_MIN, SERVO_MAX + 1);

  // Convert degrees to PWM duty cycle
  pwm.setPWM(SERVO_X_PIN, 0, map(xAngle, 0, 180, 0, 4095));
  pwm.setPWM(SERVO_Y_PIN, 0, map(yAngle, 0, 180, 0, 4095));

  // Trigger sound for this movement
  triggerAudio();

  // Wait a short time while spine moves
  delay(500);  // 0.5 s movement time

  // Return spine to neutral (0,0)
  pwm.setPWM(SERVO_X_PIN, 0, map(SERVO_NEUTRAL, 0, 180, 0, 4095));
  pwm.setPWM(SERVO_Y_PIN, 0, map(SERVO_NEUTRAL, 0, 180, 0, 4095));

  // Optional: short pause before next cycle
  delay(2000);
}

void triggerAudio() {
  // Pull trigger pins low for < 100 ms to fire the clip
  digitalWrite(AUDIO_TRIGGER_X, LOW);
  digitalWrite(AUDIO_TRIGGER_Y, LOW);
  delay(50);
  digitalWrite(AUDIO_TRIGGER_X, HIGH);
  digitalWrite(AUDIO_TRIGGER_Y, HIGH);
}
