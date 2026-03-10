#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H

#include <Arduino.h>
#include <stdint.h>

// ✅ OPTIMIZED MAXXECU STRUCT (Packed by Size)
// Total Size: ~344 bytes (Dependent on alignment)
// This MUST match the Receiver struct EXACTLY.

typedef struct {
    // ==========================================
    // 1. 4-BYTE VARIABLES (Floats & Int32)
    // ==========================================
    float throttlePos;
    float MAP;
    float Afr;
    float lambda;
    float lambdaA;
    float lambdaB;
    float ignitionAngle;
    float fuelPulseWidth;
    float fuelDuty;
    float undrivenSpeed;
    float drivenSpeed;
    float wheelSlip;
    float targetSlip;
    float tractionCtrl;
    float lambdaCorrA;
    float lambdaCorrB;
    float firmwareVersion;
    float batteryVoltage;
    float baroPressure;
    float intakeAirTemp;
    float coolantTemp;
    float totalFuelTrim;
    float ethanolConcentration;
    float totalIgnitionComp;
    float userAnalogInput1;
    float userAnalogInput2;
    float userAnalogInput3;
    float userAnalogInput4;
    float userChannel1;
    float userChannel2;
    float userChannel3;
    float userChannel4;
    float userChannel5;
    float userChannel6;
    float userChannel7;
    float userChannel8;
    float userChannel9;
    float userChannel10;
    float userChannel11;
    float userChannel12;
    float boostSolenoidDuty;
    float oilPressure;
    float oilTemp;
    float wastegatePressure;
    float coolantPressure;
    float boostTarget;
    float virtualFuelTank;
    float transmissionTemp;
    float differentialTemp;
    float accelerationForward;
    float accelerationRight;
    float accelerationUp;
    float lambdaTarget;
    float knockLevel;
    float warningCH;
    float tripmeter;
    float fuelLevelPercent;
    
    // GPS Floats
    float gpsLat;
    float gpsLon;

    int rpm; // Standard int is 4 bytes on ESP32

    // ==========================================
    // 2. 2-BYTE VARIABLES (Int16)
    // ==========================================
    int16_t id;
    int16_t ignitionCut;
    int16_t fuelCut;
    int16_t vehicleSpeed;
    int16_t egt1;
    int16_t egtDifference;
    int16_t cpuTemp;
    int16_t errorCodeCount;
    int16_t syncLostCount;
    int16_t gear;
    int16_t fuelPressure;
    int16_t revLimitRPM;
    int16_t SPARE3;
    int16_t ecuErrorCodes;
    int16_t adjustedVE1;
    int16_t Torque_Nm;
    int16_t HorsePowerSC10;
    int16_t tachoRPM;
    int16_t radFan;

    // ==========================================
    // 3. 1-BYTE VARIABLES (Uint8 & Bool)
    // ==========================================
    uint8_t isHeartbeat;
    uint8_t activeBoostTable;
    uint8_t activeTuneSelector;
    uint8_t shiftcutActive;
    uint8_t revLimitActive;
    uint8_t antiLagActive;
    uint8_t launchControlActive;
    uint8_t tractionPowerLimiterActive;
    uint8_t throttleBlipActive;
    uint8_t acIdleUpActive;
    uint8_t knockDetected;
    uint8_t brakePedalActive;
    uint8_t clutchPedalActive;
    uint8_t speedLimitActive;
    uint8_t gpLimiterActive;
    uint8_t userCutActive;
    uint8_t ecuLogging;
    uint8_t nitrousActive;
    uint8_t leftTurnSignal;
    uint8_t rightTurnSignal;
    int8_t userPWMTable1;

    // GPS / Bridge Flags
    uint8_t gpsSpeedLimit;
    bool wifiConnected;
    bool gpsLocked;
    bool tomtomOnline;
    
    // Custom Flags
    uint8_t parkingBrakeActive;
    uint8_t oilPressureWarning;
    uint8_t fuelLowWarning;

    // Weather
    int8_t weatherTemp;
    uint8_t weatherHumidity;

    // ==========================================
    // 4. ARRAYS (Strings)
    // ==========================================
    char weatherCond[8]; // Bumped to 8 for alignment
    char displayName[32];

} Maxxecu;

// --- CONTROL PACKET (Keep as is) ---
typedef struct {
    int boostMode;       // 0=Street, 1=Sport, 2=Race
    int tractionVal;     // 0-100%
    bool scrambleActive; // true/false
    bool valetActive;    // true/false
} ControlPacket;

// Static assert: Size CHANGED! 
// Use Serial.println(sizeof(Maxxecu)) to find the new size and update this number.
 static_assert(sizeof(Maxxecu) == 348, "Maxxecu size is incorrect!"); 

// Declare Global Instance
extern Maxxecu canData;
extern uint8_t buffer[32];
extern uint8_t bufferIndex;

#endif // GLOBALVARIABLE_H



/*
#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H

#include <Arduino.h>
#include <stdint.h>
// ✅ Declare Maxxecu Struct Globally

typedef struct {
// ECU data 
int16_t id;
uint8_t isHeartbeat;
   // uint32_t msg_count = 0;
int rpm;
float throttlePos;
float MAP;
float Afr;
float lambda;
float lambdaA;
float lambdaB;
float ignitionAngle;
int16_t ignitionCut;
float fuelPulseWidth;
float fuelDuty;
int16_t fuelCut;
int16_t vehicleSpeed;
float undrivenSpeed;
float drivenSpeed;
float wheelSlip;
float targetSlip;
float tractionCtrl;
float lambdaCorrA;
float lambdaCorrB;
float firmwareVersion;
float batteryVoltage;
float baroPressure;
float intakeAirTemp;
float coolantTemp;
float totalFuelTrim;
float ethanolConcentration;
float totalIgnitionComp;
int16_t egt1;
int16_t egtDifference;
int16_t cpuTemp;
int16_t errorCodeCount;
int16_t syncLostCount;
float userAnalogInput1;
float userAnalogInput2;
float userAnalogInput3;
float userAnalogInput4;
float userChannel1;
float userChannel2;
float userChannel3;
float userChannel4;
float userChannel5;
float userChannel6;
float userChannel7;
float userChannel8;
float userChannel9;
float userChannel10;
float userChannel11;
float userChannel12;
int16_t gear;
float boostSolenoidDuty;
float oilPressure;
float oilTemp;
int16_t fuelPressure;
float wastegatePressure;
float coolantPressure;
float boostTarget;
uint8_t activeBoostTable;
uint8_t activeTuneSelector;
float virtualFuelTank;
float transmissionTemp;
float differentialTemp;

uint8_t shiftcutActive;
uint8_t revLimitActive;
uint8_t antiLagActive;
uint8_t launchControlActive;
uint8_t tractionPowerLimiterActive;
uint8_t throttleBlipActive;
uint8_t acIdleUpActive;
uint8_t knockDetected;

uint8_t brakePedalActive;
uint8_t clutchPedalActive;
uint8_t speedLimitActive;
uint8_t gpLimiterActive;
uint8_t userCutActive;
uint8_t ecuLogging;
uint8_t nitrousActive;

uint8_t leftTurnSignal;           // CAN bus Default SPARE2
uint8_t rightTurnSignal;         // CAN bus Default SPARE3
int16_t revLimitRPM;
int16_t SPARE3;
float accelerationForward;
float accelerationRight;
float accelerationUp;
float lambdaTarget;
int16_t ecuErrorCodes;
float knockLevel;
int16_t adjustedVE1;
int16_t Torque_Nm;
int16_t HorsePowerSC10;
int16_t tachoRPM;
float warningCH;
float tripmeter;
int16_t radFan;
uint8_t userPWMTable1;    //ACIS solenoid
float fuelLevelPercent;

// GPS data BridgeNODE
  uint8_t gpsSpeedLimit;
  float gpsLat;
  float gpsLon;
  bool wifiConnected;
  bool gpsLocked;
  bool tomtomOnline;
  char displayName[32];

//weather info from BridgeNODE
  int8_t weatherTemp;
  uint8_t weatherHumidity;
  char weatherCond[7];  // 6 chars + null

// Custom warning flags from BridgeNode
uint8_t parkingBrakeActive;
uint8_t oilPressureWarning;
uint8_t fuelLowWarning;
} Maxxecu;

// --- ADD THIS SECTION ---
typedef struct {
    int boostMode;       // 0=Street, 1=Sport, 2=Race
    int tractionVal;     // 0-100%
    bool scrambleActive; // true/false
    bool valetActive;    // true/false
} ControlPacket;
// ------------------------

// Static assert to ensure the size of struct_Maxxecu is 288 bytes (or whatever your expected size is)
static_assert(sizeof(Maxxecu) == 368, "Maxxecu size is incorrect!");  //352

// Declare Global Instance

extern Maxxecu canData;
extern uint8_t buffer[32];
extern uint8_t bufferIndex;
#endif // GLOBALVARIABLE_H
*/