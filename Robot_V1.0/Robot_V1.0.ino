#include <ESP32Servo.h> 
#include <IBusBM.h>     

// --- RC Receiver Pin Definitions ---
// For iBUS, you only need ONE signal pin.
// We'll use Serial2 for iBUS communication on the ESP32.
// ESP32 UART2 default pins are GPIO16 (RX) and GPIO17 (TX).
// We only need the RX pin for the receiver.
const int IBUS_RX_PIN = 16; // Connect your FS-iA10B iBUS signal wire here

// --- RC Signal Mapping Constants ---
// These are typical raw values from FlySky iBUS channels
const int IBUST_MIN_VAL = 1000;
const int IBUST_MAX_VAL = 2000;
const int IBUST_MID_VAL = 1500;

// FlySky channels are commonly mapped as:
// Channel 0: Aileron (Roll)
// Channel 1: Elevator (Pitch)
// Channel 2: Throttle
// Channel 3: Rudder (Yaw)
// Channel 4+: Aux channels (Switches, Pots)
// Verify this on your FS-i6X in the "Aux. channels" or "Functions" menu
const int CHANNEL_THROTTLE = 2; // For your robot's forward/backward movement
const int CHANNEL_STEERING = 3; // For your robot's left/right turning (Yaw/Rudder)
const int CHANNEL_AUX1     = 4; // Example Aux channel for weapon activation
const int CHANNEL_AUX2     = 5; // Example Aux channel for other functions

// --- Failsafe Configuration ---
// iBUS has a built-in failsafe, but it's good to have a software fallback.
// If the receiver stops sending data (e.g., power loss to receiver), this timeout catches it.
const unsigned long RECEIVER_TIMEOUT_MS = 500; // If no data for this long, assume failsafe

class RadioController {
private:
    IBusBM ibus; 
    unsigned long lastDataReceivedTime; 

public:
    RadioController() {
        lastDataReceivedTime = millis(); 
    }

    void begin() {
        ibus.begin(Serial2, IBUS_RX_PIN);
        Serial.println("RadioController (iBUS) initialized on Serial2.");
    }

    void readChannels() {
        if (ibus.read()) { 
            lastDataReceivedTime = millis();
        }
    }


    //TODO
    bool isSignalLost() {
        // Check IBusBM's internal failsafe flag (if configured on TX/RX)
        // AND check our software timeout.
        return ibus.get Failsafe() || (millis() - lastDataReceivedTime > RECEIVER_TIMEOUT_MS);
    }

    // Helper to map raw iBUS channel values to a desired range
    int mapChannel(int channelIndex, int outMin, int outMax) {
        // თუ სიგნალი არ გვაქვს გააჩეროს , აქ შესამოწმებელია ეს საშუალო თუ აჩერებს 
        if (isSignalLost()) return (outMin + outMax) / 2; 
        
        int rawValue = ibus.readChannel(channelIndex);
        rawValue = constrain(rawValue, IBUST_MIN_VAL, IBUST_MAX_VAL);
        return map(rawValue, IBUST_MIN_VAL, IBUST_MAX_VAL, outMin, outMax);
    }

   
    int getThrottle() {
        return mapChannel(CHANNEL_THROTTLE, -100, 100);
    }

    int getSteering() {
        return mapChannel(CHANNEL_STEERING, -100, 100);
    }

    int getAux1() {
        return mapChannel(CHANNEL_AUX1, 0, 100);
    }

    int getAux2() {
        return mapChannel(CHANNEL_AUX2, 0, 100);
    }

    int getRawChannel(int channelIndex) {
        return ibus.readChannel(channelIndex);
    }
};

// --- Global RadioController Object ---
RadioController radio; // No pins in constructor needed for iBUS

// --- Motor Class Definition (from previous example) ---
// Global constants for common motor PWM values
const int MOTOR_PWM_MIN_US = 1000;
const int MOTOR_PWM_MAX_US = 2000;
const int MOTOR_PWM_NEUTRAL_US = 1500;

// Define GPIO pins for your motors
const int PIN_DRIVE_FRONT_LEFT = 13;
const int PIN_DRIVE_FRONT_RIGHT = 14;
const int PIN_WEAPON_MOTOR_1 = 26;
const int PIN_WEAPON_MOTOR_2 = 27;

class Motor {
private:
    Servo esc;
    int pin;
    int minPulse;
    int maxPulse;
    int neutralPulse;
    bool isBidirectional;
    int currentSpeedUs;

public:
    Motor(int motorPin, bool bidirectional, int pMin = MOTOR_PWM_MIN_US, int pMax = MOTOR_PWM_MAX_US, int pNeutral = MOTOR_PWM_NEUTRAL_US) {
        pin = motorPin;
        isBidirectional = bidirectional;
        minPulse = pMin;
        maxPulse = pMax;
        neutralPulse = pNeutral;
        currentSpeedUs = isBidirectional ? neutralPulse : minPulse;
    }

    void begin() {
        esc.attach(pin, minPulse, maxPulse);
        esc.writeMicroseconds(currentSpeedUs);
    }

    void setSpeedUs(int speedUs) {
        speedUs = constrain(speedUs, minPulse, maxPulse);
        esc.writeMicroseconds(speedUs);
        currentSpeedUs = speedUs;
    }

    void setSpeedPercent(int percent) {
        percent = constrain(percent, 0, 100);
        int targetPulse;
        if (isBidirectional) {
            if (percent < 50) {
                targetPulse = map(percent, 0, 50, minPulse, neutralPulse);
            } else {
                targetPulse = map(percent, 50, 100, neutralPulse, maxPulse);
            }
        } else {
            targetPulse = map(percent, 0, 100, minPulse, maxPulse);
        }
        setSpeedUs(targetPulse);
    }

    void stop() {
        setSpeedUs(isBidirectional ? neutralPulse : minPulse);
    }

    void forward(int percent) {
        if (isBidirectional) {
            percent = constrain(percent, 0, 100);
            setSpeedPercent(50 + (percent / 2));
        } else {
            setSpeedPercent(percent);
        }
    }

    void reverse(int percent) {
        if (isBidirectional) {
            percent = constrain(percent, 0, 100);
            setSpeedPercent(50 - (percent / 2));
        } else {
            stop();
        }
    }

    int getCurrentSpeedUs() {
        return currentSpeedUs;
    }

    void sendMaxThrottleForCalibration() {
        setSpeedUs(maxPulse);
    }

    void sendMinThrottleForCalibration() {
        setSpeedUs(minPulse);
    }
};

// --- Global Motor Objects ---
Motor driveFrontLeft(PIN_DRIVE_FRONT_LEFT, true);
Motor driveFrontRight(PIN_DRIVE_FRONT_RIGHT, true);
Motor weaponMotor1(PIN_WEAPON_MOTOR_1, false);
Motor weaponMotor2(PIN_WEAPON_MOTOR_2, false);


void setup() {
    Serial.begin(115200);
    Serial.println("--- ESP32 Robot Control with FlySky FS-iA10B (iBUS) and Motors (OOP) ---");

    radio.begin(); // Initialize the iBUS radio controller

    // Initialize motors (as per previous example, ensure ESC calibration is done separately)
    driveFrontLeft.begin();
    driveFrontRight.begin();
    weaponMotor1.begin();
    weaponMotor2.begin();

    // IMPORTANT: Perform ESC Calibration as described in the previous example!
    // This code assumes ESCs are already calibrated. If not, run the calibration sketch first.
    Serial.println("\n*** IMPORTANT: Ensure your ESCs are calibrated before running this code. ***");
    Serial.println("Connect FS-iA10B iBUS to ESP32 GPIO16 (UART2 RX) and power receiver.");
    Serial.println("Turn on FS-i6X transmitter.");
    Serial.println("Check Serial Monitor for channel values and robot response.");
    Serial.println("--- Use the 'Aux. channels' menu on your FS-i6X to verify channel assignments! ---");
}

void loop() {
    radio.readChannels(); // Constantly read the latest data from the iBUS receiver

    if (radio.isSignalLost()) {
        Serial.println("RC Signal Lost! Engaging Failsafe (stopping motors).");
        driveFrontLeft.stop();
        driveFrontRight.stop();
        weaponMotor1.stop();
        weaponMotor2.stop();
    } else {
        // --- Drive Motor Control based on RC ---
        int throttleValue = radio.getThrottle();  // -100 to 100
        int steeringValue = radio.getSteering();  // -100 to 100

        // Simple tank drive mixing example:
        // Adjust these mixing values to suit your robot's desired movement
        int leftMotorSpeed = throttleValue + steeringValue;
        int rightMotorSpeed = throttleValue - steeringValue;

        // Constrain speeds to -100 to 100
        leftMotorSpeed = constrain(leftMotorSpeed, -100, 100);
        rightMotorSpeed = constrain(rightMotorSpeed, -100, 100);

        // Apply to motors using forward/reverse/stop methods
        if (leftMotorSpeed > 0) {
            driveFrontLeft.forward(leftMotorSpeed);
        } else if (leftMotorSpeed < 0) {
            driveFrontLeft.reverse(abs(leftMotorSpeed));
        } else {
            driveFrontLeft.stop();
        }

        if (rightMotorSpeed > 0) {
            driveFrontRight.forward(rightMotorSpeed);
        } else if (rightMotorSpeed < 0) {
            driveFrontRight.reverse(abs(rightMotorSpeed));
        } else {
            driveFrontRight.stop();
        }

        // --- Weapon Motor Control based on RC Aux Channel ---
        int aux1Value = radio.getAux1(); // 0-100 (e.g., from a switch or knob)

        // Example: If Aux1 is high (e.g., switch in ON position), spin weapon motors at full speed.
        // You'll need to adapt this logic for your specific weapon(s) and how your Aux channels are used.
        if (aux1Value > 50) { // Assuming aux channel is a 2-position switch (0 or 100)
            weaponMotor1.setSpeedPercent(100); // Full speed
            weaponMotor2.setSpeedPercent(100); // Full speed
        } else {
            weaponMotor1.stop();
            weaponMotor2.stop();
        }
    }

    // --- Debugging Output (uncomment if needed) ---
    // Serial.print("Thr: "); Serial.print(radio.getThrottle());
    // Serial.print("\tSte: "); Serial.print(radio.getSteering());
    // Serial.print("\tAux1: "); Serial.print(radio.getAux1());
    // Serial.print("\tRaw Ch2: "); Serial.print(radio.getRawChannel(2)); // Raw throttle
    // Serial.print("\tRaw Ch3: "); Serial.print(radio.getRawChannel(3)); // Raw steering
    // Serial.print("\tRaw Ch4: "); Serial.print(radio.getRawChannel(4)); // Raw Aux1
    // Serial.print("\tSignal Lost: "); Serial.println(radio.isSignalLost() ? "YES" : "NO");

    delay(20); // Small delay to give ESP32 time for other tasks and not flood serial
}