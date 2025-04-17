/*
  source for power management: https://embeddedcentric.com/lesson-14-nrf5x-power-management-tutorial/

  TODO:
    - use BLE event handle see blecallbackled/callbackled for sample
    - use BLE enhanced advertise

*/

#include <mic.h>
#if defined(WIO_TERMINAL)
#include "processing/filters.h"
#endif
#include <NanoBLEFlashPrefs.h>

#include "LSM6DS3.h"
#include "Wire.h"

#include <ArduinoBLE.h>

#define VERSION     102      // upto 2 digit major, 2 digit minor


#define SHOWPLOT    0         // for sound debuging only, if 1 just plot sine wave to serial port, no any other processing in the debug
#define SHOWACCEL   0         // for IMU debuging only
#define SWHOBLE     1
#define SHOWACTIVITY  1
#define SHOWBATTERY 1

#define DIFFTARGET 31000      // glock & gas
#define LATCHSHOW  50
#define OUTPUTPIN1  LEDR
#define OUTPUTPIN2  D1        // I/O of the laser control pin
#define OUTPUTSTANDBY LEDB
#define OUTPUTIDLE  LEDG 

#define INERTTIMEOUT    10    // inert check time in sec
#define STANDBYTIMEOUT  30    // stand by time after inert, working in low power mode
#define SLEEPTIMEOUT    60    // sleep timeout after standby, this will shutdown the device

#define SOUND_THRESHOLD_UUID      "D4E5F678-9012-3456-789A-BCDEF0123456"
#define START_CHARACTERISTIC_UUID "E5F6A1B2-C3D4-5678-9012-3456789ABCDE"
typedef enum State 
{
  Error = -1,
  Starting = 0,
  Aims = 1,
  Standby = 2,
  Idle = 3,
  Sleep = 4
};

//BLE
BLEService tslaserService("87E01439-99BE-45AA-9410-DB4D3F23EA99"); // Bluetooth® Low Energy Service


// Note for add more characteristic
//1. add below characteristic
//2. add in setup() 
//3. add in handle DoBLE() if writable
//4. add .setValue if notify


// Bluetooth® Low Energy Laser Characteristic - custom 128-bit UUID, read/writable/notify by central
// on/off laser for testing/borsight 1 = laser alway on, 0 = laser off
BLEShortCharacteristic debugLaserCharacteristic("1FB91B09-88A0-4AD3-9BB9-28A2E4E85F8E", BLERead | BLEWrite);

// debug mic sensitive; 1 = start debug, 0 = stop, must subscribe for the notify
BLEShortCharacteristic debugMicCharacteristic("EA727F6D-1B2F-4613-BADF-DDF2C01659EC", BLERead | BLEWrite | BLENotify);

// deviceStatus -1 = unknow error, 0 = ready, 1 = mic error, 2 = IMU error, 3 = stoarge error, 4 = BLE error
BLEShortCharacteristic deviceStatusCharacteristic("D90A7C02-9B21-4243-8372-3E523FA7978B", BLERead);

// set id of device to send out of the shot
BLEShortCharacteristic deviceIDCharacteristic("26B4FBEA-53D6-4D84-B5BC-21A211F0B317", BLERead | BLEWrite);

// enable/disable the laser; 1 = enable, 0 = disable
BLEShortCharacteristic enableLaserCharacteristic("B45BDB1D-2C6A-4417-B97C-F10C217E2045", BLERead | BLEWrite);

// notify laser shot as a pulse 1 = laser on, 0 = laser off; ___|""|__
BLEShortCharacteristic onShootCharacteristic("536C26D4-1C9A-48F3-A9A2-C3D28A2088FA", BLERead | BLENotify);

// send out 16 bit of [bit 15----0] for [15-8]=device id, [7-0] = magazine count 0-255 (now support only counting up)
BLEWordCharacteristic onShootWithIDAndMagazineCharacteristic("BEAC5C3F-A981-4481-A626-A4F89611F9C8", BLERead | BLENotify);

// diff in sound check for microphone detecion control
BLEShortCharacteristic micLevelDetectCharacteristic("2C4959B0-010A-4F00-B229-F36AD1500CE8", BLERead | BLEWrite);

// delay in msec to on laser  
BLEShortCharacteristic latchLaserCharacteristic("254C722E-794D-4C8E-B0BE-3EB824294AD6", BLERead | BLEWrite);

// counter of the shot since day one
BLEUnsignedLongCharacteristic counterAllCharacteristic("134D9AED-7952-42D0-B8FC-96E59AADA8AD", BLERead | BLENotify);

// counter of the shot since last reset or last charged, 0 = reset
BLEUnsignedLongCharacteristic counterAccCharacteristic("2649093C-943C-4F6E-9585-5A445C70DB8F", BLERead | BLEWrite | BLENotify);

// profile control, write any to load default settings, not yet implement 2nd profile
BLEByteCharacteristic profileCharacteristic("D46AAB9F-0082-4398-969A-17645952A9B2", BLERead | BLEWrite);

// battery status, -1 = unknow, 0 = low, 1 = fully charge or not charged
BLECharCharacteristic batteryCharacteristic("DA313D4A-BCAD-4F4E-99A1-1004A5412461", BLERead | BLENotify);

BLECharacteristic soundThresholdCharacteristic(SOUND_THRESHOLD_UUID, BLERead | BLEWrite, sizeof(int));
BLECharacteristic startCharacteristic(START_CHARACTERISTIC_UUID, BLERead | BLEWrite, 1);

//Mic
// Settings
#if defined(WIO_TERMINAL)
#define DEBUG 1                 // Enable pin pulse during ISR  
#define SAMPLES 16000*3
#elif defined(ARDUINO_ARCH_NRF52840)
#define DEBUG 1                 // Enable pin pulse during ISR  
#define SAMPLES 800
#endif

mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
#if defined(WIO_TERMINAL)
  .debug_pin = 1                // Toggles each DAC ISR (if DEBUG is set to 1)
#elif defined(ARDUINO_ARCH_NRF52840)
  .debug_pin = 0                // Toggles each DAC ISR (if DEBUG is set to 1)
#endif
};

int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

int16_t maxval = 1000;
int16_t minval = -1000;

int16_t lastdiff = 0;
int16_t shotcounter = 0;
int16_t allcounter = 0;
unsigned long startMillis = 0;

uint32_t lastsavecounter = 0;

#if defined(WIO_TERMINAL)
DMA_ADC_Class Mic(&mic_config);
#elif defined(ARDUINO_ARCH_NRF52840)
NRF52840_ADC_Class Mic(&mic_config);
#endif

//eeprom
// Access to the library
NanoBLEFlashPrefs myFlashPrefs;
bool settingsChanged = false;  // set this to true for auto save the settings when the device on idle

//IMU params
bool onaims = false;
bool oninert = false;
unsigned long inertstartMillis = 0;

float accelx_last = 0;
float accely_last = 0;
float accelz_last = 0;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// to be settings/reading remotely from central
bool debugLaser = false;
bool debugMic = false;            // return max range of the sound
int16_t deviceStatus = -1;   //  -1 = unknow error, 0 = ready, 1 = mic error, 2 = IMU error, 3 = stoarge error, 4 = BLE error
//id                              // in the persistance data, to send out with the magazine count
byte magazineCount = 0;
bool enableLaser = true;          // shall en/dis from software
//difftarget
//latchshow
//counter_all                     // in the persistance data structure
//counter_acc                     // in the persistance data structure
//--below are not yet implement
//serialNo
//battery level (later version)


bool laserison = false;  // internall work for latching the laser

// to save in eeprom
typedef struct tagPersistanceData 
{
  uint16_t size; // version by size

  // unique data and statistic
  uint32_t serialNo;      // readable
  uint32_t counter_all;   // no reset, for endurance monitoring
  uint32_t counter_acc;   // resetable, for batterly life monitoring

  unsigned char id;            // read/write
  unsigned char profileID;     // not yet implement

  // parameter settings for profile
  int16_t difftarget; // = DIFFTARGET;  // sensitive of the microsoft default is 3000, less value is more sensitive
  int16_t latchshow;  //  = 50;         // latch time to turn on laser after silent
  uint16_t inertTimeout;
  uint16_t standbyTimeout;
  uint16_t sleepTimeout;

} PersistanceData;

PersistanceData persistanceData;


#if defined(WIO_TERMINAL)
FilterBuHp filter;
#endif

State currentState = State::Starting;
State previousState = State::Starting;
unsigned long stateStartMillis = 0;

int SOUND_THRESHOLD = 100;
bool started = false;
bool thresholdSet = false;
bool connectedToMaster = false;

void onThresholdWrite(BLEDevice central, BLECharacteristic characteristic) {
  characteristic.readValue(&SOUND_THRESHOLD, sizeof(SOUND_THRESHOLD));
  thresholdSet = true;
  Serial.print("🔧 SOUND_THRESHOLD updated to: ");
  Serial.println(SOUND_THRESHOLD);
}

void setup() {

  deviceStatus = 0; // ready

  Serial.begin(115200);
  delay(1000);

  InitLoadPersistanceData(&persistanceData);

#if defined(WIO_TERMINAL)  
  pinMode(WIO_KEY_A, INPUT_PULLUP);
#endif

  pinMode(OUTPUTPIN1, OUTPUT);
  pinMode(OUTPUTPIN2, OUTPUT);
  pinMode(OUTPUTSTANDBY, OUTPUT);
  pinMode(OUTPUTIDLE, OUTPUT);

  Mic.set_callback(audio_rec_callback);

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed");
    deviceStatus = 1;
  } else {
    Serial.println("Mic initialization done.");
  }

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
    deviceStatus = 3;
  } else {
    Serial.println("Device OK!");
  }

  LEDLaser(false);

  // BLE Initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    deviceStatus = 4;
  } else {
    // Optional: Set names for BLE visibility
    BLE.setLocalName("XIAO_LaserSlave");
    BLE.setDeviceName("XIAO_LaserSlave");

    // Add characteristics to service
    tslaserService.addCharacteristic(debugLaserCharacteristic);
    tslaserService.addCharacteristic(debugMicCharacteristic);
    tslaserService.addCharacteristic(deviceStatusCharacteristic);
    tslaserService.addCharacteristic(deviceIDCharacteristic);
    tslaserService.addCharacteristic(enableLaserCharacteristic);
    tslaserService.addCharacteristic(onShootCharacteristic);
    tslaserService.addCharacteristic(onShootWithIDAndMagazineCharacteristic);
    tslaserService.addCharacteristic(micLevelDetectCharacteristic);
    tslaserService.addCharacteristic(latchLaserCharacteristic);
    tslaserService.addCharacteristic(counterAllCharacteristic);
    tslaserService.addCharacteristic(counterAccCharacteristic);
    tslaserService.addCharacteristic(profileCharacteristic);
    tslaserService.addCharacteristic(batteryCharacteristic);
    tslaserService.addCharacteristic(soundThresholdCharacteristic);
    tslaserService.addCharacteristic(startCharacteristic);

    // Add service to BLE stack
    BLE.addService(tslaserService);

    // Optional: Add write handler
    soundThresholdCharacteristic.setEventHandler(BLEWritten, onThresholdWrite);

    // Initialize values
    debugLaserCharacteristic.writeValue(debugLaser);
    debugMicCharacteristic.writeValue(debugMic);
    deviceStatusCharacteristic.writeValue(deviceStatus);
    deviceIDCharacteristic.writeValue(persistanceData.id);
    enableLaserCharacteristic.writeValue(enableLaser);
    onShootCharacteristic.writeValue(false);
    onShootWithIDAndMagazineCharacteristic.writeValue(ShootValue());
    micLevelDetectCharacteristic.writeValue(persistanceData.difftarget);
    latchLaserCharacteristic.writeValue(persistanceData.latchshow);
    counterAllCharacteristic.writeValue(persistanceData.counter_all);
    counterAccCharacteristic.writeValue(persistanceData.counter_acc);
    profileCharacteristic.writeValue(persistanceData.profileID);
    batteryCharacteristic.writeValue(-1); // not yet known

    // Set advertised service and begin advertising
    BLE.setAdvertisedService(tslaserService);
    BLE.advertise();
    Serial.println("📡 Advertising...");
  }

  // Battery charging control
  pinMode(P0_13, OUTPUT);
  digitalWrite(P0_13, LOW); // 100mA

  // Battery charging status
  pinMode(P0_17, INPUT); // 0 = charging, 1 = charged
  CheckBattery();
}

void loop() { 

  BLE.poll();

  DoBLE();

  if (currentState != previousState || stateStartMillis == 0)
  {
    stateStartMillis = millis();
    previousState = currentState;
  }
  unsigned long stateDiff = millis() - stateStartMillis;

  if (!connectedToMaster) {
    Serial.println("Not connected to Master yet...");
    delay(1000);
    return;
  }

  if (!started || !thresholdSet) {
    if (currentState != State::Starting) {
      currentState = State::Starting;
      Serial.println("Waiting for SOUND_THRESHOLD and START from master...");
    }

    LEDAims(false);
    LEDIdle(false);
    LEDStandby(true);
    return;
  }

  if (deviceStatus == 0 && started && thresholdSet) 
  {
    CheckIMU();
    
    if (onaims && !oninert) 
    {
      currentState = State::Aims;
      record_ready = true;
    }
    else 
    {
      if (oninert && previousState == State::Aims) 
      {
        currentState = State::Standby;
      }
      else if (oninert && previousState == State::Standby && stateDiff > (persistanceData.standbyTimeout * 1000)) 
      {
        currentState = State::Idle;
      }
      else if (previousState == State::Idle && stateDiff > (persistanceData.sleepTimeout * 1000))
      {
        if (started && thresholdSet) {
          currentState = State::Sleep;
        }
      }
      else if (currentState != State::Sleep)
      {
        
      }
    }

    if (currentState != State::Aims) 
    {
      if (lastsavecounter != persistanceData.counter_all || settingsChanged)
      {
        SavePersistanceData(&persistanceData);
        lastsavecounter = persistanceData.counter_all;
        settingsChanged = false;
      }
    }
  }
  else 
  {
    currentState = State::Error;
  }

#if defined(WIO_TERMINAL)  
  if (digitalRead(WIO_KEY_A) == LOW && !recording) {

      Serial.println("Starting sampling");
      recording = 1;
      record_ready = false;  
  }
  
#endif

#if defined(WIO_TERMINAL)  
    if (!recording && record_ready)
#elif defined(ARDUINO_ARCH_NRF52840)
    if (record_ready)
#endif  
    {
#if SHOWPLOT == 1
    Serial.println("Finished sampling");
#endif
    int16_t thismin = 0;
    int16_t thismax = 0;
    
    for (int i = 0; i < SAMPLES; i++) {
      
      //int16_t sample = filter.step(recording_buf[i]);
      int16_t sample = recording_buf[i];
      if (sample < minval)
        minval = sample;
      if (sample > maxval)
        maxval = sample;

      if (sample < thismin)
        thismin = sample;
      if (sample > thismax)
        thismax = sample;

#if SHOWPLOT == 1

      Serial.print(sample);
      Serial.print(",");
      Serial.print(minval);
      Serial.print(",");
      Serial.print(maxval);
      Serial.println();
#else
      int16_t diff = thismax - thismin;
      int16_t difftarget = persistanceData.difftarget;
      if (diff > difftarget && diff != lastdiff) 
      {
        lastdiff = diff;
        shotcounter++;
        allcounter++;
        startMillis = millis();

        if (debugMic) 
        {
          debugMicCharacteristic.setValue(diff);
        }

        Serial.print(diff);
        Serial.print(",");
        Serial.print(shotcounter);
        Serial.print(",");
        Serial.print(persistanceData.counter_all+1);
        //Serial.print(",");
        //Serial.print(allcounter);
        Serial.println();

        LEDLaser(true);
        laserison = true;
      }
      else 
      {
        shotcounter = 0;
        unsigned long currentMillis = millis();
        int16_t latchshow = persistanceData.latchshow;
        if (laserison && currentMillis > startMillis + latchshow) 
        {
          LEDLaser(false);
          laserison = false;
          persistanceData.counter_all++;
          persistanceData.counter_acc++;
          magazineCount++;
          onShootWithIDAndMagazineCharacteristic.setValue(ShootValue());
          counterAllCharacteristic.setValue(persistanceData.counter_all);
          counterAccCharacteristic.setValue(persistanceData.counter_acc);
        }
      }
#endif
      //

    }
  
    record_ready = false; 
  }

  //other status rathan laser
  static uint16_t lastPrintout = 0;
  uint16_t secToPrint = stateDiff / 2000;
  if (currentState == State::Aims) 
  {
    LEDAims(true);
#if SHOWACTIVITY == 1
    if (lastPrintout != secToPrint) 
    {
      Serial.print("Aims ");
      Serial.print(stateDiff / 1000);
      Serial.println();
      lastPrintout = secToPrint;
      NRF_POWER->TASKS_CONSTLAT = 1;
    }
#endif
  }
  else if (currentState == State::Idle)
  {
    LEDIdle(true);
#if SHOWACTIVITY == 1
    if (lastPrintout != secToPrint) 
    {
      Serial.print("Idle ");
      Serial.print(stateDiff / 1000);
      Serial.println();
      lastPrintout = secToPrint;
      NRF_POWER->TASKS_LOWPWR = 1;
    }
#endif
  }
  else if (currentState == State::Standby)
  {
    LEDStandby(true);
#if SHOWACTIVITY == 1
    if (lastPrintout != secToPrint) 
    {
      Serial.print("Standby ");
      Serial.print(stateDiff / 1000);
      Serial.println();
      lastPrintout = secToPrint;
    }
#endif
  }
  else 
  {
    LEDSleep();
#if SHOWACTIVITY == 1
    if (lastPrintout != secToPrint) 
    {
      Serial.print("Sleep ");
      Serial.print(stateDiff / 1000);
      Serial.println();
      lastPrintout = secToPrint;

      //if (stateDiff / 1000 > 30) 
      {
        // LEDOff();
        // Serial.println("Shutdown");
        // NRF_POWER->SYSTEMOFF = 1;
      }
    }
#endif
  }

  CheckBattery();
  
}

static void audio_rec_callback(uint16_t *buf, uint32_t buf_len) 
{
  
static uint32_t idx = 0;
  // Copy samples from DMA buffer to inference buffer
#if defined(WIO_TERMINAL)
  if (recording) 
#endif
  {
    for (uint32_t i = 0; i < buf_len; i++) {
  
      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
#if defined(WIO_TERMINAL)
      recording_buf[idx++] = filter.step((int16_t)(buf[i] - 1024) * 16);
      //recording_buf[idx++] = (int16_t)(buf[i] - 1024) * 16;  
#elif defined(ARDUINO_ARCH_NRF52840)
      recording_buf[idx++] = buf[i];
#endif

      if (idx >= SAMPLES){ 
        idx = 0;
        recording = 0;
        //record_ready = true;
        break;
     } 
    }
  }

}

// blue lit
void LEDAims(bool on) 
{
  if (on) 
  {
      digitalWrite(OUTPUTSTANDBY, LOW);       // will turn the led off
      digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led on
  }
  else 
  {
      digitalWrite(OUTPUTSTANDBY, HIGH);         // will turn the led off
  }
}

// blue blink
void LEDStandby(bool on)
{
  static unsigned long lastTime = 0;
  static bool islit = false; // first is not lit, we came from aims state
  if (on) 
  {
      if (lastTime == 0) 
      {
        lastTime = millis();
        digitalWrite(OUTPUTSTANDBY, HIGH);         // will turn the led off
      }
      else if (millis() > lastTime + 500)
      {
        lastTime = millis();
        if (islit == false) 
        {
          digitalWrite(OUTPUTSTANDBY, LOW);         // will turn the led on
          islit = true;
        }
        else 
        {
          digitalWrite(OUTPUTSTANDBY, HIGH);         // will turn the led off
          islit = false;
        }
      }
      digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led off
  }
  else 
  {
      digitalWrite(OUTPUTSTANDBY, HIGH);         // will turn the led off
  }
}

// green
void LEDIdle(bool on) 
{
  static unsigned long lastTime = 0;
  static bool islit = false; // first is not lit, we came from prior state
  if (on) 
  {
      if (lastTime == 0) 
      {
        lastTime = millis();
        digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led off
      }
      else if (millis() > lastTime + 500)
      {
        lastTime = millis();
        if (islit == false) 
        {
          digitalWrite(OUTPUTIDLE, LOW);         // will turn the led on
          islit = true;
        }
        else 
        {
          digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led off
          islit = false;
        }
      }
      digitalWrite(OUTPUTSTANDBY, HIGH);         // will turn the led off
  }
  else 
  {
      digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led off
  }
}

// all off
void LEDSleep()
{
  digitalWrite(OUTPUTPIN1, HIGH);         // will turn the led off
  digitalWrite(OUTPUTSTANDBY, HIGH);      // will turn the led off
  digitalWrite(OUTPUTIDLE, HIGH);         // will turn the led off
}

void LEDOff()
{
  LEDSleep();
}

// red and laser on
void LEDLaser(bool on)
{
  onShootCharacteristic.setValue(on || debugLaser);
  if ((on && enableLaser) || debugLaser)
  {
      digitalWrite(OUTPUTPIN1, LOW);         // will turn the led on
      digitalWrite(OUTPUTPIN2, HIGH);         // will turn the laser on
  }
  else 
  {
      digitalWrite(OUTPUTPIN1, HIGH);         // will turn the led off
      digitalWrite(OUTPUTPIN2, LOW);         // will turn the laser off
  }
}

uint16_t ShootValue()
{
  uint16_t rt = persistanceData.id;
  rt <<= 8;
  rt |= magazineCount;
  return rt;
}

void SetDefaultPersistanceData(int16_t profile, PersistanceData *data) 
{
  if (data) 
  {
    data->profileID = profile;
    switch (profile)
    {
      default: 
        data->difftarget = DIFFTARGET;
        data->latchshow = LATCHSHOW;
        data->inertTimeout = INERTTIMEOUT;
        data->standbyTimeout = STANDBYTIMEOUT;
        data->sleepTimeout = SLEEPTIMEOUT;
        break;
    }
  }
}

void InitLoadPersistanceData(PersistanceData *data) 
{
  if (data) 
  {
    memset(data, 0, sizeof(PersistanceData));
    data->size = sizeof(PersistanceData);
    SetDefaultPersistanceData(0, data);

    // begin initialization
    Serial.println("Read record...");
    int rc = myFlashPrefs.readPrefs(data, sizeof(uint16_t));
    if (rc == FDS_SUCCESS && data->size == sizeof(PersistanceData))
    {
      rc = myFlashPrefs.readPrefs(data, sizeof(PersistanceData));
      if (rc == FDS_SUCCESS)
      {
       
      }
      else
      {
          Serial.print("No persistance found. Return code: ");
          Serial.println(myFlashPrefs.errorString(rc));
          settingsChanged = true;
      }
    }
    else 
    {
      Serial.print("Record size is ");
      Serial.println(data->size);
      rc = myFlashPrefs.readPrefs(data, 16);  // try to recover
      if (rc == FDS_SUCCESS) 
      {
        Serial.println("Wrong setting found, recover 1st segment success");
      }
      else 
      {
        Serial.println("Wrong setting found, restore default");
        SetDefaultPersistanceData(0, data);
      }
      myFlashPrefs.deletePrefs();
      persistanceData.size = sizeof(persistanceData);
      SavePersistanceData(&persistanceData);
    }

    Serial.print("Persistance : ID: ");
    Serial.print(data->id);
    Serial.print(", counter_all: ");
    Serial.println(data->counter_all);
  }
  else
  {
    Serial.println("No place to save the data");
  }
}

int SavePersistanceData(PersistanceData *data)
{
  Serial.println("Write persistance data...");
  myFlashPrefs.writePrefs(data, sizeof(PersistanceData));
}

int CheckIMU() 
{
  int rt = 0;

  float accelx = myIMU.readFloatAccelX();
  float accely = myIMU.readFloatAccelY();
  float accelz = myIMU.readFloatAccelZ();

  // detect aims
  if (abs(accelx) < 0.4) 
  {
    onaims = true;
  }
  else 
  {
    onaims = false;
  }

  if (accelx_last != 0 && accely_last != 0 && accelz != 0) 
  {
      float diffx = abs(accelx - accelx_last);
      float diffy = abs(accely - accely_last);
      float diffz = abs(accelz - accelz_last);

#if SHOWACCEL == 1

      Serial.print("Accelerometer Diff:");
      Serial.print(" X1 = ");
      Serial.print(diffx, 4);
      Serial.print(", Y1 = ");
      Serial.print(diffy, 4);
      Serial.print(", Z1 = ");
      Serial.println(diffz, 4);

#endif
      float inertdiff = 0.03;
      if (diffx < inertdiff && diffy < inertdiff && diffz < inertdiff)
      {
          // under inert
      }
      else
      {
          inertstartMillis = millis();
      }

      rt = 1;
  }
  accelx_last = accelx;
  accely_last = accely;
  accelz_last = accelz;

  if (inertstartMillis > 0) 
  {
    unsigned long currentMillis = millis();
    if (currentMillis > inertstartMillis + (persistanceData.inertTimeout * 1000)) 
    {
      oninert = true;
    }
    else 
    {
      oninert = false;
    }
  }
  else 
  {
    oninert = false;
    inertstartMillis = millis();
  }

  return rt;
}


static int lastbatteryStagte = -1;
static unsigned long lastbatteryCheck = 0;
int CheckBattery()
{
  int rt = 0;

  if (lastbatteryCheck == 0 || millis() > lastbatteryCheck + 5000) 
  {
    int val = digitalRead(P0_17);
    if (val != lastbatteryStagte) 
    { 
      lastbatteryStagte = val;

      batteryCharacteristic.setValue(val);

      Serial.print("Battery stage is : ");
      Serial.println(val);
      rt = 1;
    }
    lastbatteryCheck = millis();
  }
  return rt;
}

static BLEDevice central;
static bool connected = false;

void BLESetLocalName() 
{
    char szLocalName[64];
    sprintf(szLocalName, "ArLaserTS%03d v%d", persistanceData.id, VERSION);
    BLE.setDeviceName("ArLaserTS");
    BLE.setLocalName(szLocalName);

    BLE.setAdvertisedService(tslaserService);

    Serial.print("Devicename: ");
    Serial.println(szLocalName);
}

int DoBLE()
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  central = BLE.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.println("central detected");
    if (!connected) 
    {
      Serial.print("Connected to central: ");
      // print the central's MAC address:
      Serial.println(central.address());
      connected = true;
      connectedToMaster = true;

      static BLEDevice central = BLE.central();

      if (central && !connected) {
        Serial.print("🔗 Connected to: ");
        Serial.println(central.address());
        connected = true;
      }

      if (central && central.connected()) {
        BLE.poll();

        byte startFlag = 0;
        startCharacteristic.readValue(&startFlag, sizeof(startFlag));
        if (startFlag == 1 && !started) {
          Serial.println("🚦 START received. Begin sensing...");
          started = true;
        }

        // ตรวจสอบการเขียนอื่น ๆ ที่ Master ส่งมา (ตามโค้ดเดิม)
      }
      else if (connected) {
        Serial.println("❌ Disconnected.");
        connected = false;
        started = false;
        thresholdSet = false;
      }
 
    }

    // while the central is still connected to peripheral:
    if (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (debugLaserCharacteristic.written()) {
        if (debugLaserCharacteristic.value()) {   // any value other than 0
          Serial.println("Laser on");
          debugLaser = true;
          LEDLaser(debugLaser);
        } else {                              // a 0 value
          Serial.println("Laser off");
          debugLaser = false;
          LEDLaser(debugLaser);
        }
      }
      if(debugMicCharacteristic.written()) {
        if (debugMicCharacteristic.value() == 1) {   // exactly 1
          Serial.println("Mic debug on");
          debugMic = true;
        } else {                              // a 0 value
          Serial.println("Mic debug off");
          debugMic = false;
        }
      }
      if(deviceIDCharacteristic.written()) {
        unsigned char value = deviceIDCharacteristic.value();
        persistanceData.id = value;
        settingsChanged = true;
        Serial.print("Device ID changed to ");
        Serial.println(value);
        BLESetLocalName();
      }

      if (enableLaserCharacteristic.written()) {
        if (enableLaserCharacteristic.value()) {   // any value other than 0
          Serial.println("Laser enabled");
          enableLaser = true;
        } else {                              // a 0 value
          Serial.println("Laser disabled");
          enableLaser = false;
        }
      }

      if(micLevelDetectCharacteristic.written()) {
        uint16_t value = micLevelDetectCharacteristic.value();
        persistanceData.difftarget = value;
        settingsChanged = true;
        Serial.print("Detect level changed to ");
        Serial.println(value);
      }
      if(latchLaserCharacteristic.written()) {
        uint16_t value = latchLaserCharacteristic.value();
        persistanceData.latchshow = value;
        settingsChanged = true;
        Serial.print("Laser Latch value changed to ");
        Serial.println(value);
      }
      if(counterAccCharacteristic.written()) {
        persistanceData.counter_acc = 0;
        settingsChanged = true;
        Serial.println("Reset acc counter");
      }
      if (profileCharacteristic.written()) {
        int16_t value = profileCharacteristic.value();
        SetDefaultPersistanceData(value, &persistanceData);

        micLevelDetectCharacteristic.writeValue(persistanceData.difftarget);
        latchLaserCharacteristic.writeValue(persistanceData.latchshow);
        profileCharacteristic.writeValue(persistanceData.profileID);

        settingsChanged = true;
        
        Serial.print("Profile cmd is ");
        Serial.println(value);  
      }
    }
    else
    {
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      connected = false;
      started = false;
      connectedToMaster = false;
      thresholdSet = false;
    }
  }
}