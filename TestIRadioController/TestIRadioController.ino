// --- PIN DEFINITIONS FOR PWM INPUT ---
// Connect the Signal pin of each receiver channel to these ESP32 GPIOs.
// (Remember to also connect receiver 5V to ESP32 5V, and receiver GND to ESP32 GND)
const int CH1_AILERON_PIN  = 25; // Connect to FS-iA10B CH1 (Aileron)
const int CH2_ELEVATOR_PIN = 26; // Connect to FS-iA10B CH2 (Elevator)
const int CH3_THROTTLE_PIN = 33; // Connect to FS-iA10B CH3 (Throttle)
const int CH4_RUDDER_PIN   = 32; // Connect to FS-iA10B CH4 (Rudder)

// --- RC Signal Calibration (Adjust if needed for your specific setup) ---
// Standard PWM pulse widths for RC signals in microseconds (us)
const int PWM_MIN_US = 1000; // Minimum pulse width (e.g., stick all the way down/left)
const int PWM_MAX_US = 2000; // Maximum pulse width (e.g., stick all the way up/right)
const int PWM_MID_US = 1500; // Center pulse width

// --- Failsafe Timeout (Optional, for detecting signal loss) ---
const unsigned long SIGNAL_TIMEOUT_MS = 500; // If no valid pulse for this long, consider signal lost
unsigned long lastSignalTime[4]; // Array to store last time each channel received a signal

void setup() {
  Serial.begin(115200);
  Serial.println("--- FS-iA10B PWM Receiver Test ---");
  Serial.println("Configuring ESP32 GPIOs for PWM input...");

  // Set the ESP32 pins as INPUT
  pinMode(CH1_AILERON_PIN, INPUT);
  pinMode(CH2_ELEVATOR_PIN, INPUT);
  pinMode(CH3_THROTTLE_PIN, INPUT);
  pinMode(CH4_RUDDER_PIN, INPUT);

  // Initialize last signal times
  for (int i = 0; i < 4; i++) {
    lastSignalTime[i] = millis();
  }

  Serial.println("Receiver ready. Ensure receiver is powered, bound, and transmitter is ON.");
  Serial.println("Move sticks/switches and observe values (us):");
  Serial.println("--------------------------------------------------");
  Serial.println("AILERON\tELEVATOR\tTHROTTLE\tRUDDER\tSignal Lost?");
}

void loop() {
  // Read raw PWM pulse widths in microseconds
  // pulseIn() waits for a pulse, so it can be blocking.
  int aileron_us  = pulseIn(CH1_AILERON_PIN, HIGH, 25000); // 25ms timeout
  int elevator_us = pulseIn(CH2_ELEVATOR_PIN, HIGH, 25000);
  int throttle_us = pulseIn(CH3_THROTTLE_PIN, HIGH, 25000);
  int rudder_us   = pulseIn(CH4_RUDDER_PIN, HIGH, 25000);

  // Update last signal time if a valid pulse was received (pulseIn returns 0 on timeout)
  if (aileron_us > 0)  lastSignalTime[0] = millis();
  if (elevator_us > 0) lastSignalTime[1] = millis();
  if (throttle_us > 0) lastSignalTime[2] = millis();
  if (rudder_us > 0)   lastSignalTime[3] = millis();

  // Check for signal loss for any channel
  bool anySignalLost = false;
  for (int i = 0; i < 4; i++) {
    if (millis() - lastSignalTime[i] > SIGNAL_TIMEOUT_MS) {
      anySignalLost = true;
      break; // One channel lost is enough to indicate overall signal loss
    }
  }

  // Print values to Serial Monitor
  Serial.print(aileron_us);
  Serial.print("\t");
  Serial.print(elevator_us);
  Serial.print("\t\t"); // Extra tab for alignment
  Serial.print(throttle_us);
  Serial.print("\t\t"); // Extra tab for alignment
  Serial.print(rudder_us);
  Serial.print("\t");
  Serial.println(anySignalLost ? "YES" : "NO");

  // A small delay to make the serial output readable, but be mindful that pulseIn() already causes delays.
  delay(100); 
}