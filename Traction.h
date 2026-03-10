#ifndef TRACTION_H
#define TRACTION_H

#include <Arduino.h>
#include <driver/twai.h>
#include "GlobalVariables.h"

// ================= CONFIGURATION =================
// 🔧 INPUT PIN
#define REAR_SPEED_PIN 35

// 🔧 TIRE SPECS
#define TIRE_CIRCUMFERENCE_MM 1747.0   // 195/45R15
#define NUM_TEETH            24        // 👈 Updated to 24-tooth trigger wheel

// 🔧 CAN BUS SETTINGS
#define CAN_ID_REAR_SPEED 0x650
// =================================================

extern float currentRearSpeedKmh;

void setupTractionControl();
void calculateAndSendSpeed();

#endif