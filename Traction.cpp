#include "Traction.h"

volatile uint32_t lastPulseTime = 0;
volatile uint32_t pulseInterval = 0;
volatile bool newDataAvailable = false;

float currentRearSpeedKmh = 0.0;
float lastValidSpeed = 0.0;

// ==========================================
// 1. INTERRUPT (Hall Sensor Pulse)
// ==========================================
void IRAM_ATTR rearSpeedISR() {
    uint32_t now = micros();
    uint32_t dt = now - lastPulseTime;

    // Stage 1: Reject impossible pulses (<150 µs) → electrical noise
    // (A 150 µs pulse on a 24-tooth wheel = ~1747 km/h)
    if (dt < 150) return;

    // Stage 2: Debounce → accept only pulses >500 µs
    // (500 µs safely handles speeds up to ~524 km/h on a 24-tooth wheel)
    if (dt > 500) {
        pulseInterval = dt;
        lastPulseTime = now;
        newDataAvailable = true;
    }
}

// ==========================================
// 2. SETUP
// ==========================================
void setupTractionControl() {
    // Set to INPUT because you have a physical 10k pull-up resistor to 3.3V installed
    pinMode(REAR_SPEED_PIN, INPUT);  
    attachInterrupt(digitalPinToInterrupt(REAR_SPEED_PIN), rearSpeedISR, FALLING);
}

// ==========================================
// 3. CALCULATE & SEND
// ==========================================
void calculateAndSendSpeed() {

    uint32_t now = micros();

    // A. TIMEOUT → No pulses for 0.5s = wheel stopped
    if (now - lastPulseTime > 500000) {
        currentRearSpeedKmh = 0.0;
        lastValidSpeed = 0.0;
        pulseInterval = 0;
    }

    // B. NEW DATA AVAILABLE → Calculate speed
    else if (newDataAvailable) {
        newDataAvailable = false;

        uint32_t dt = pulseInterval;
        if (dt > 0) {
            float freq = 1000000.0f / dt;  // Hz
            float wheelRPM = (freq / NUM_TEETH) * 60.0f;
            float speed = wheelRPM * TIRE_CIRCUMFERENCE_MM * 0.00006f;

            // Clamp unrealistic spikes (sensor noise > 300 km/h)
            if (speed < 300.0f) {
                // Smooth the data for hand-cut teeth (Exponential Moving Average)
                currentRearSpeedKmh = (lastValidSpeed * 0.6f) + (speed * 0.4f);
                lastValidSpeed = currentRearSpeedKmh;
            }
        }
    }

    // C. SEND TO MAXXECU
    twai_message_t msg;
    msg.identifier = CAN_ID_REAR_SPEED;
    msg.extd = 0;
    msg.data_length_code = 2;

    // Send as Integer * 10 (e.g. 100.5 km/h -> 1005)
    uint16_t sendValue = (uint16_t)(currentRearSpeedKmh * 10.0f);
    
    msg.data[0] = (sendValue >> 8) & 0xFF;
    msg.data[1] = sendValue & 0xFF;

    twai_transmit(&msg, pdMS_TO_TICKS(1));
}