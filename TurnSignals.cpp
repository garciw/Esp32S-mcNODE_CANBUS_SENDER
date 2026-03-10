#include "TurnSignals.h"
#include "GlobalVariables.h"

// We use a timeout to keep the signal active even during the "off" phase of the physical blinker
static unsigned long lastLeftActive = 0;
static unsigned long lastRightActive = 0;
const unsigned long SIGNAL_HOLD_TIME = 600; // Keep signal true for 600ms after last pulse

void update_turn_signals() {
    bool leftInput = (digitalRead(TURN_LEFT_PIN) == LOW);   // Assuming Active LOW
    bool rightInput = (digitalRead(TURN_RIGHT_PIN) == LOW); // Assuming Active LOW
    unsigned long now = millis();

    // 1. Pulse Stretcher (Debounce)
    // If the wire is active, reset the timer
    if (leftInput) lastLeftActive = now;
    if (rightInput) lastRightActive = now;

    // 2. Logic: Are we currently signaling?
    // We are active if the input is LIVE OR if we saw it less than 600ms ago
    bool sendLeft = (now - lastLeftActive < SIGNAL_HOLD_TIME);
    bool sendRight = (now - lastRightActive < SIGNAL_HOLD_TIME);

    // 3. Send SOLID status (1 or 0), do NOT blink here
    canData.leftTurnSignal = sendLeft ? 1 : 0;
    canData.rightTurnSignal = sendRight ? 1 : 0;
}