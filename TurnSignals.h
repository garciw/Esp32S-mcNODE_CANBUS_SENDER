#ifndef TURNSIGNALS_H
#define TURNSIGNALS_H

#include <Arduino.h>
#include "GlobalVariables.h"

// Pin Definitions
#define TURN_LEFT_PIN   19   // GPIO19 NodeMCU
#define TURN_RIGHT_PIN  18   // GPIO18 NodeMCU

// Function Prototype
void update_turn_signals(); 

#endif // TURNSIGNALS_H