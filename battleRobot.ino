#include <ESP32Servo.h>
// ტესტი
// Define the GPIO pins connected to each ESC signal wire
// MAKE SURE THESE MATCH YOUR ACTUAL WIRING!
const int ESC1_PIN = 13; // Example: Motor 1 (e.g., Drive Front Left)
const int ESC2_PIN = 14; // Example: Motor 2 (e.g., Drive Front Right)
const int ESC3_PIN = 26; // Example: Motor 3 (e.g., Drive Rear Left or Weapon)
const int ESC4_PIN = 27; // Example: Motor 4 (e.g., Drive Rear Right or Weapon)

// Create Servo objects for each ESC
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Define standard RC PWM pulse widths in microseconds (us)
// You might need to slightly adjust these after initial calibration if motors don't spin smoothly.
const int MIN_PULSE_WIDTH = 1000; // Corresponds to STOP/Minimum throttle
const int MAX_PULSE_WIDTH = 2000; // Corresponds to MAX throttle
const int NEUTRAL_PULSE_WIDTH = 1500; // Corresponds to STOP for bidirectional motors

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial.println("ESP32 Motor Controller - ESC Calibration & Test");

  // Attach each Servo object to its respective pin with min/max pulse width limits
  esc1.attach(ESC1_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  esc2.attach(ESC2_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  esc3.attach(ESC3_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  esc4.attach(ESC4_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  Serial.println("ESCs attached to pins.");
  Serial.println("\n--- STARTING ESC CALIBRATION SEQUENCE (READ CAREFULLY) ---");
  Serial.println("1. Ensure MAIN BATTERY is DISCONNECTED from all ESCs.");
  Serial.println("2. Upload this code to your ESP32.");
  Serial.println("3. The ESP32 will immediately send MAX throttle signal (2000us).");
  Serial.println("4. NOW, QUICKLY & CAREFULLY connect your MAIN BATTERY to the ESCs.");
  Serial.println("5. Listen for the ESCs' beeps. You should hear initial power-on beeps, then a special set of beeps (often 2 short beeps) indicating 'waiting for max throttle'.");
  Serial.println("6. After those beeps (usually within 2-5 seconds), the code will automatically send MIN throttle (1000us).");
  Serial.println("7. Listen for another set of beeps (often 1 long beep or several short ones) indicating 'min throttle received' and ARMING.");
  Serial.println("8. Once you hear the ARMING beeps, calibration is complete!");
  Serial.println("   The motors are now armed and ready to receive commands in loop().");
  Serial.println("DO NOT SKIP CALIBRATION! It's vital for safe and proper operation.");

  // Step 1 of calibration: Send MAX throttle
  esc1.writeMicroseconds(MAX_PULSE_WIDTH);
  esc2.writeMicroseconds(MAX_PULSE_WIDTH);
  esc3.writeMicroseconds(MAX_PULSE_WIDTH);
  esc4.writeMicroseconds(MAX_PULSE_WIDTH);
  Serial.println("\nSending MAX throttle for calibration... (Connect battery now if not already)");
  delay(5000); // Give yourself 5 seconds to connect the main battery after upload

  // Step 2 of calibration: Send MIN throttle
  esc1.writeMicroseconds(MIN_PULSE_WIDTH);
  esc2.writeMicroseconds(MIN_PULSE_WIDTH);
  esc3.writeMicroseconds(MIN_PULSE_WIDTH);
  esc4.writeMicroseconds(MIN_PULSE_WIDTH);
  Serial.println("Sending MIN throttle for calibration... (Listen for arming beeps)");
  delay(2000); // Give ESCs time to register min signal and arm

  Serial.println("Calibration sequence complete. ESCs should now be armed.");
  Serial.println("Starting basic motor test in loop(). Always start with very low throttle!");
}

void loop() {
  // --- Basic Motor Test Routine ---
  // This will spin up motors slowly, hold a speed, then stop them.
  // This is a basic example. You will later replace this with joystick/RC control logic.

  Serial.println("\nRamping up motors...");
  for (int pulse = MIN_PULSE_WIDTH + 50; pulse <= NEUTRAL_PULSE_WIDTH + 200; pulse += 10) { // Start slightly above min, go a bit above neutral
      // Adjust the `+50` and `+200` to find a good low-speed point and a safe testing maximum.
      // For bidirectional drive motors, 1500us is stop. Above 1500us is forward, below is reverse.
      // For weapon motors, 1000us is stop, 2000us is full speed (unidirectional).

      // Apply same speed to all motors for initial test
      esc1.writeMicroseconds(pulse);
      esc2.writeMicroseconds(pulse);
      esc3.writeMicroseconds(pulse);
      esc4.writeMicroseconds(pulse);
      delay(50); // Small delay to make the ramp-up visible
  }

  Serial.println("Motors at a low test speed for 3 seconds.");
  delay(3000); // Hold the speed for 3 seconds

  Serial.println("Ramping down motors to stop...");
  for (int pulse = NEUTRAL_PULSE_WIDTH + 200; pulse >= MIN_PULSE_WIDTH; pulse -= 10) {
      esc1.writeMicroseconds(pulse);
      esc2.writeMicroseconds(pulse);
      esc3.writeMicroseconds(pulse);
      esc4.writeMicroseconds(pulse);
      delay(50);
  }

  Serial.println("Motors stopped. Waiting 5 seconds before repeating.");
  esc1.writeMicroseconds(MIN_PULSE_WIDTH); // Ensure all motors are truly stopped/disarmed
  esc2.writeMicroseconds(MIN_PULSE_WIDTH);
  esc3.writeMicroseconds(MIN_PULSE_WIDTH);
  esc4.writeMicroseconds(MIN_PULSE_WIDTH);
  delay(5000); // Wait 5 seconds before the next test cycle
}