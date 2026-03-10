#include <Wire.h>
#include <driver/twai.h>
#include <Adafruit_ADS1X15.h>
#include "TurnSignals.h"
#include "GlobalVariables.h"
#include "Traction.h"

// ============================================================
// CONFIGURATION
// ============================================================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Screen Serial Pins
#define SCREEN_RX_PIN 16
#define SCREEN_TX_PIN 17

// Pins
#define LED_PIN 2
#define BRAKE_SWITCH_PIN 27
#define oilLight_SWITCH_PIN 34
#define ACIS_CONTROL_PIN 32

// ADS1115 Pin Map
const int oilPressurePin = 0;
const int fuelPressurePin = 1;
const int fuelLevelPin1 = 3;
const int knockSensorPin = 2;

// Bits for MaxxECU Status
#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

// Hardware Serial for the Screen
HardwareSerial ScreenSerial(2); 

// Objects
Adafruit_ADS1115 ads;
Maxxecu canData;
ControlPacket incomingControl;

// Timers
unsigned long msg_count = 0;
unsigned long lastTractionUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSerialSend = 0; 
unsigned long lastECUSend = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
void sendKnockToECU();
void sendStatusToECU();
void sendControlToECU();
void read_analog_inputs();
void parse_canData();
void TaskFunction(void* pvParameters);
void send_data();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200); 
  esp_log_level_set("*", ESP_LOG_INFO);

  // Initialize Hardware Serial for the display
  ScreenSerial.begin(115200, SERIAL_8N1, SCREEN_RX_PIN, SCREEN_TX_PIN);
  ScreenSerial.setTimeout(10); 

  // 1. CAN BUS
  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general_config.rx_queue_len = 65;
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) {
    while (1); 
  }
  twai_start();

  // 2. ADS1115
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!ads.begin(0x48)) {
    while (1); 
  }
  ads.setGain(GAIN_ONE);

  // 4. PINS
  pinMode(TURN_LEFT_PIN, INPUT);
  pinMode(TURN_RIGHT_PIN, INPUT);
  pinMode(BRAKE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(oilLight_SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACIS_CONTROL_PIN, OUTPUT);
  digitalWrite(ACIS_CONTROL_PIN, LOW);

  // 5. TRACTION SETUP
  setupTractionControl();

  // 6. BACKGROUND TASK (CAN Receive) 
  xTaskCreatePinnedToCore(TaskFunction, "CAN_Task", 4096, NULL, 1, NULL, 1);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ============================================================
  // ENGINE / CAN LOGIC (ALWAYS RUNS)
  // ============================================================

  // Traction Control (20ms)
  if (now - lastTractionUpdate > 20) {
    calculateAndSendSpeed();
    canData.undrivenSpeed = currentRearSpeedKmh;
    lastTractionUpdate = now;
  }

  // ECU Output (50ms)
  if (now - lastECUSend > 50) {
    sendKnockToECU();
    sendStatusToECU();
    lastECUSend = now;
  }

  // Analog Sensors (100ms)
  if (now - lastSensorRead > 100) {
    read_analog_inputs();
    lastSensorRead = now;
  }

  // ============================================================
  // SEND DATA TO DISPLAY (35ms pacing)
  // ============================================================
  if (now - lastSerialSend > 35) {
    send_data();

    // RECEIVE CONTROL PACKET FROM SCREEN
    if (ScreenSerial.available() >= sizeof(ControlPacket)) {
      ScreenSerial.readBytes((uint8_t*)&incomingControl, sizeof(ControlPacket));
      sendControlToECU();
      
      // Flush extra garbage bytes to keep stream in sync
      while (ScreenSerial.available()) ScreenSerial.read();
    }

    lastSerialSend = now;
  }
}

// ============================================================
// BACKGROUND TASK (CAN Receive)
// ============================================================
void TaskFunction(void* pvParameters) {
  while (true) {
    parse_canData();
    update_turn_signals();
    vTaskDelay(1);
  }
}

// ============================================================
// SENSOR READING
// ============================================================
void read_analog_inputs() {
  // Raw Readings
  int16_t oilPressureRaw    = ads.readADC_SingleEnded(oilPressurePin);
  int16_t fuelPressureRaw   = ads.readADC_SingleEnded(fuelPressurePin);
  int16_t fuelLevelRaw      = ads.readADC_SingleEnded(fuelLevelPin1); 
  int16_t knockRaw          = ads.readADC_SingleEnded(knockSensorPin); 

  float oilPressureVoltage  = oilPressureRaw * 0.000125;
  float fuelPressureVoltage = fuelPressureRaw * 0.000125;
  float fuelSenderVoltage   = fuelLevelRaw * 0.000125;
  float knockVoltage        = knockRaw * 0.000125;

  // --- KNOCK ---
  canData.knockLevel = (int16_t)((knockVoltage / 5.0) * 100.0);   

  // --- FUEL LEVEL ---
  float V_source = 3.3;  
  float R_pullup = 10000.0; 
  if (fuelSenderVoltage > V_source - 0.01) fuelSenderVoltage = V_source - 0.01;
  if (fuelSenderVoltage < 0.0) fuelSenderVoltage = 0.0;
  float fuelSenderResistance = (fuelSenderVoltage * R_pullup) / (V_source - fuelSenderVoltage);
  float fuelPercent = interpolateFuelPercent(fuelSenderResistance);
  static float smoothedFuel = 0;
  smoothedFuel = 0.02 * fuelPercent + 0.98 * smoothedFuel;
  canData.fuelLevelPercent = smoothedFuel;
  canData.fuelLowWarning = (smoothedFuel < 15.0) ? 1 : 0;

  // --- OIL PRESSURE ---
  bool oilSensorConnected = oilPressureVoltage > 0.45 && oilPressureVoltage < 4.6;
  float rawOilPressure = oilSensorConnected ? mapToPressure(oilPressureVoltage) : 0.0;
  if (canData.rpm < 300 && rawOilPressure < 3.0) canData.oilPressure = 0.0;
  else canData.oilPressure = rawOilPressure;
  bool warningLowIdle = (canData.rpm < 450 && rawOilPressure < 5.0);
  bool warningWhileRunning = (canData.rpm >= 1500 && rawOilPressure < 8.0);
  canData.oilPressureWarning = warningLowIdle || warningWhileRunning;

  // --- FUEL PRESSURE ---
  float calculatedFuelPress = 0.0;
  if (fuelPressureVoltage > 0.45 && fuelPressureVoltage < 4.6) {
      calculatedFuelPress = (fuelPressureVoltage - 0.5) * (1000.0 / 4.0); 
  }
  canData.fuelPressure = (int16_t)calculatedFuelPress; 

  // --- SWITCHES ---
  canData.parkingBrakeActive = digitalRead(BRAKE_SWITCH_PIN) == LOW;
}

// ============================================================
// CAN SENDERS
// ============================================================
void sendKnockToECU() {
  int16_t dataToSend = (int16_t)(canData.knockLevel * 10.0);
  twai_message_t tx_message;
  tx_message.identifier = 0x703;
  tx_message.data_length_code = 2;
  tx_message.flags = 0;
  tx_message.data[0] = (uint8_t)(dataToSend & 0xFF);
  tx_message.data[1] = (uint8_t)((dataToSend >> 8) & 0xFF);
  twai_transmit(&tx_message, pdMS_TO_TICKS(1));
}

void sendControlToECU() {
  twai_message_t message;
  message.identifier = 0x666;
  message.extd = 0;
  message.flags = 0;
  message.data_length_code = 3;
  message.data[0] = incomingControl.boostMode;
  message.data[1] = incomingControl.tractionVal;
  message.data[2] = (incomingControl.scrambleActive ? 0x01 : 0) |
                    (incomingControl.valetActive ? 0x02 : 0);
  twai_transmit(&message, pdMS_TO_TICKS(1));
}

void sendStatusToECU() {
  twai_message_t tx_msg1;
  tx_msg1.identifier = 0x750;
  tx_msg1.data_length_code = 5;
  tx_msg1.flags = 0;
  tx_msg1.data[0] = canData.parkingBrakeActive;
  tx_msg1.data[1] = canData.oilPressureWarning;
  tx_msg1.data[2] = canData.fuelLowWarning;
  uint16_t fuelSend = (uint16_t)(canData.fuelLevelPercent * 10);
  tx_msg1.data[3] = fuelSend & 0xFF;
  tx_msg1.data[4] = (fuelSend >> 8) & 0xFF;
  twai_transmit(&tx_msg1, pdMS_TO_TICKS(1));

  twai_message_t tx_msg2;
  tx_msg2.identifier = 0x751;
  tx_msg2.data_length_code = 6;
  tx_msg2.flags = 0;
  int16_t oilP_Send = (int16_t)canData.oilPressure;
  tx_msg2.data[0] = oilP_Send & 0xFF;
  tx_msg2.data[1] = (oilP_Send >> 8) & 0xFF;
  int16_t fuelP_Send = canData.fuelPressure;
  tx_msg2.data[2] = fuelP_Send & 0xFF;
  tx_msg2.data[3] = (fuelP_Send >> 8) & 0xFF;
  uint16_t speedSend = (uint16_t)(canData.undrivenSpeed * 10);
  tx_msg2.data[4] = speedSend & 0xFF;
  tx_msg2.data[5] = (speedSend >> 8) & 0xFF;
  twai_transmit(&tx_msg2, pdMS_TO_TICKS(1));
}

// ============================================================
// SERIAL SEND (WITH MAGIC HEADER)
// ============================================================
void send_data() {
  canData.id = msg_count++;
  
  // 🛑 NEW: The "Magic Barcode" Synchronization Header
  const uint8_t header[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  ScreenSerial.write(header, 4);
  
  // Send the actual engine data
  ScreenSerial.write((uint8_t*)&canData, sizeof(canData));
}
// ============================================================
// CAN PARSER (Standard)
// ============================================================
void parse_canData() {
  twai_message_t ReadCAN;
  while (twai_receive(&ReadCAN, 0) == ESP_OK) { 
    switch (ReadCAN.identifier) {
        
    case 0x112: memcpy(canData.displayName + 0,  ReadCAN.data, 8); break;
    case 0x113: memcpy(canData.displayName + 8,  ReadCAN.data, 8); break;
    case 0x114: memcpy(canData.displayName + 16, ReadCAN.data, 8); break;
    case 0x115: memcpy(canData.displayName + 24, ReadCAN.data, 8); break;
    case 0x116: canData.displayName[32 - 0] = '\0'; break;

    case 0x120: {
      canData.weatherTemp = (int8_t)ReadCAN.data[0];
      canData.weatherHumidity = ReadCAN.data[1];
      memcpy(canData.weatherCond, &ReadCAN.data[2], 6);
      canData.weatherCond[6] = '\0';
    } break;

    case 0x520:
      canData.rpm = (ReadCAN.data[1] << 8) | ReadCAN.data[0];
      canData.throttlePos = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.MAP = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.lambda = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.001;
    break;

    case 0x521:
      canData.lambdaA = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.001;
      canData.Afr = canData.lambdaA * 14.7;
      canData.lambdaB = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.001;
      canData.ignitionAngle = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.ignitionCut = (ReadCAN.data[7] << 8) | ReadCAN.data[6];
    break;

    case 0x522:
      canData.fuelPulseWidth = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.001;
      canData.fuelDuty = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.fuelCut = (ReadCAN.data[5] << 8) | ReadCAN.data[4];
      canData.vehicleSpeed = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) / 10;
    break;

    case 0x523:
    {
       // Undriven Speed Blocked (We calculate it locally)
       canData.drivenSpeed = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
       canData.wheelSlip = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
       canData.targetSlip = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    }
    break;

    case 0x524:
      canData.tractionCtrl = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.1;
      canData.lambdaCorrA = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.lambdaCorrB = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.firmwareVersion = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.001;
    break;

    case 0x525:
      canData.userChannel9 = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.1;
      canData.userChannel10 = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.userChannel11 = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.userChannel12 = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x526: 
      canData.shiftcutActive = (ReadCAN.data[0] & BIT_0) ? 1 : 0;
      canData.revLimitActive = (ReadCAN.data[0] & BIT_1) ? 1 : 0;
      canData.antiLagActive  = (ReadCAN.data[0] & BIT_2) ? 1 : 0;
      canData.launchControlActive = (ReadCAN.data[0] & BIT_3) ? 1 : 0;
      canData.tractionPowerLimiterActive = (ReadCAN.data[0] & BIT_4) ? 1 : 0;
      canData.throttleBlipActive = (ReadCAN.data[0] & BIT_5) ? 1 : 0;
      canData.acIdleUpActive     = (ReadCAN.data[0] & BIT_6) ? 1 : 0;
      canData.knockDetected      = (ReadCAN.data[0] & BIT_7) ? 1 : 0;

      canData.brakePedalActive  = (ReadCAN.data[1] & BIT_0) ? 1 : 0;
      canData.clutchPedalActive = (ReadCAN.data[1] & BIT_1) ? 1 : 0;
      canData.speedLimitActive  = (ReadCAN.data[1] & BIT_2) ? 1 : 0;
      canData.gpLimiterActive   = (ReadCAN.data[1] & BIT_3) ? 1 : 0;
      canData.userCutActive     = (ReadCAN.data[1] & BIT_4) ? 1 : 0;
      canData.ecuLogging        = (ReadCAN.data[1] & BIT_5) ? 1 : 0;
      canData.nitrousActive     = (ReadCAN.data[1] & BIT_6) ? 1 : 0;
      // Bytes 4 & 5 are Rev Limit (Target RPM)
      canData.revLimitRPM = (ReadCAN.data[5] << 8) | ReadCAN.data[4];
    break;

    case 0x527:
      canData.accelerationForward = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.1;
      canData.accelerationRight = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.accelerationUp = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.lambdaTarget = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.001 * 14.7;
    break;

    case 0x530:
      canData.batteryVoltage = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.01;
      canData.baroPressure = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.intakeAirTemp = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.coolantTemp = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x531:
      canData.totalFuelTrim = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]); 
      canData.ethanolConcentration = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.totalIgnitionComp = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.egt1 = (ReadCAN.data[7] << 8) | ReadCAN.data[6];
    break;

    case 0x534:
      canData.egtDifference = (ReadCAN.data[1] << 8) | ReadCAN.data[0];
      canData.cpuTemp = (ReadCAN.data[3] << 8) | ReadCAN.data[2];
      canData.errorCodeCount = (ReadCAN.data[5] << 8) | ReadCAN.data[4];
      canData.syncLostCount = (ReadCAN.data[7] << 8) | ReadCAN.data[6];
    break;

    case 0x535:
      canData.userAnalogInput1 = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.1;
      canData.userAnalogInput2 = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.userAnalogInput3 = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.userAnalogInput4 = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x536:
      canData.gear = (ReadCAN.data[1] << 8) | ReadCAN.data[0];
      canData.boostSolenoidDuty = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.oilTemp = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x537:
      canData.wastegatePressure = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.coolantPressure = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.boostTarget = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x538:
      canData.userChannel1 = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.1; 
      canData.userChannel2 = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1; 
      canData.userChannel3 = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1; 
      canData.userChannel4 = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1; 
    break;

    case 0x539:
      canData.userChannel5 = ((ReadCAN.data[1] << 8) | ReadCAN.data[0]) * 0.01; 
      canData.userChannel6 = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.userChannel7 = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.userChannel8 = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x540:
      canData.activeBoostTable = ReadCAN.data[0];
      canData.activeTuneSelector = ReadCAN.data[1];
      canData.virtualFuelTank = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.01;
      canData.transmissionTemp = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.differentialTemp = ((ReadCAN.data[7] << 8) | ReadCAN.data[6]) * 0.1;
    break;

    case 0x542:
      canData.ecuErrorCodes = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]);
    break;

    case 0x543:
      canData.tachoRPM = (ReadCAN.data[1] << 8) | ReadCAN.data[0];
      canData.warningCH = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]); 
      canData.tripmeter = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
      canData.radFan = (ReadCAN.data[7] << 8) | ReadCAN.data[6];
    break;

    case 0x544:
      canData.adjustedVE1 = (ReadCAN.data[1] << 8) | ReadCAN.data[0];
      canData.Torque_Nm = ((ReadCAN.data[3] << 8) | ReadCAN.data[2]) * 0.1;
      canData.HorsePowerSC10 = ((ReadCAN.data[5] << 8) | ReadCAN.data[4]) * 0.1;
    break;

    case 0x700:
    {
      canData.userPWMTable1 = (ReadCAN.data[0]); 
      if (canData.userPWMTable1 > 0) digitalWrite(ACIS_CONTROL_PIN, HIGH);
      else digitalWrite(ACIS_CONTROL_PIN, LOW);
    }
    break;
  }
  }
}

// ============================================================
// HELPERS
// ============================================================
float mapToPressure(float voltage) {
  return (voltage - 0.5) * 100.0 / 4.0;  
}

// ============================================================
// TOYOTA CAMRY SENSOR (10-110 Ohms Safe Zone)
// ============================================================
const int NUM_POINTS = 6; 
const float fuelResistancePoints[NUM_POINTS] = { 10.0,  30.0, 50.0, 75.0, 95.0, 110.0 };
const float fuelPercentPoints[NUM_POINTS]    = { 100.0, 80.0, 50.0, 25.0, 10.0, 0.0 };

float interpolateFuelPercent(float resistance) {
  static float lastValidPercent = 50.0; 
  bool wiperSlidOff = (resistance > 115.0 || resistance < 8.0);

  if (wiperSlidOff) {
      if (lastValidPercent >= 80.0) return 100.0; 
      else if (lastValidPercent <= 20.0) return 0.0;   
      else return lastValidPercent; 
  }

  float currentPercent = 0.0;
  if (resistance <= fuelResistancePoints[0]) {
      currentPercent = fuelPercentPoints[0];
  }
  else if (resistance >= fuelResistancePoints[NUM_POINTS - 1]) {
      currentPercent = fuelPercentPoints[NUM_POINTS - 1];
  }
  else {
      for (int i = 0; i < NUM_POINTS - 1; i++) {
        if (resistance >= fuelResistancePoints[i] && resistance <= fuelResistancePoints[i + 1]) {
          float r1 = fuelResistancePoints[i];
          float r2 = fuelResistancePoints[i + 1];
          float p1 = fuelPercentPoints[i];
          float p2 = fuelPercentPoints[i + 1];
          currentPercent = p1 + (resistance - r1) * (p2 - p1) / (r2 - r1);
          break;
        }
      }
  }
  lastValidPercent = currentPercent; 
  return currentPercent;
}
// win✅ 
