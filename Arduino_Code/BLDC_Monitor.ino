/*******************************************************
  BLDC MOTOR CONDITION MONITORING SYSTEM
  ESP32 + SSD1327 OLED + MPU6050
  ACS712 30A + Voltage Sensor + DS18B20
*******************************************************/
#define BLYNK_TEMPLATE_ID "TMPL32sYZqjir"
#define BLYNK_TEMPLATE_NAME "BLDC Motor Monitor"
#define BLYNK_AUTH_TOKEN "2znKd9J8p2XAl8sEqStefhfqH5_tet_K"

#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>


char ssid[] = "AndroidAP_9188";
char pass[] = "12345678";

//================ OLED ===================

#define OLED_CS   5
#define OLED_DC   27
#define OLED_RST  14

U8G2_SSD1327_MIDAS_128X128_F_4W_HW_SPI oled(
  U8G2_R0, OLED_CS, OLED_DC, OLED_RST
);

//================ MPU6050 =================

Adafruit_MPU6050 mpu;
bool mpuAvailable = false;

//================ DS18B20 =================

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

//============== ANALOG PINS ==============

#define CURRENT_PIN  34
#define VOLTAGE_PIN  35

//============== ADC SETTINGS ==============

const float ADC_REFERENCE = 3.3;
const int ADC_MAX = 4095;

// Replace with your voltage sensor's verified ratio.
const float VOLTAGE_RATIO = 7.3;

// ACS712 30A nominal sensitivity
const float ACS_SENSITIVITY = 0.066; // V/A

// Actual sensor OUT voltage / GPIO34 voltage.
// 1.0 means no voltage scaling.
// Set this using the verified divider ratio.
const float CURRENT_DIVIDER_RATIO = 1.0;

// Calibrated at startup with zero current
float zeroCurrentADCVoltage = 0.0;

//============== VARIABLES =================

float Voltage = 0.0;
float Current = 0.0;
float Power = 0.0;
float Energy = 0.0;
float Temperature = 0.0;
unsigned long RunTime = 0;
unsigned long SaveTimer = 0;
unsigned long RunTimer = 0;
Preferences preferences;
//================= STATISTICS =================

float MaxVoltage=0;
float MinVoltage=1000;

float MaxCurrent=0;

float MaxTemperature=0;

float MaxPower=0;

float MaxEnergy=0;

float MaxVibration=0;

float AvgVoltage=0;
float AvgCurrent=0;
float AvgTemperature=0;
float AvgPower=0;
float AvgVibration=0;
//================ PERFORMANCE ANALYSIS ================

float MotorEfficiency = 0;

float PowerFactor = 0;

float DailyEnergy = 0;

float AverageDailyPower = 0;

float OperatingHours = 0;

float LoadPercentage = 0;

float PerformanceIndex = 100;

float ReliabilityIndex = 100;

String PerformanceStatus = "EXCELLENT";
String FirmwareVersion="Version 1.0";

unsigned long SampleCount=0;

float VoltageSum=0;
float CurrentSum=0;
float TemperatureSum=0;
float PowerSum=0;
float VibrationSum=0;

float Vibration=0;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

unsigned long lastEnergyTime = 0;
unsigned long pageTimer = 0;
byte page = 0;



//================ Predictive Maintenance ================

String MaintenanceStatus = "NORMAL";

//========== ADVANCED PREDICTIVE MAINTENANCE ==========

float MotorHealth = 100;

float BearingHealth = 100;

float CoolingHealth = 100;

float ElectricalHealth = 100;

float RemainingLife = 100;

float WearPercentage = 0;

String MaintenanceRecommendation = "NONE";

bool ServiceRequired = false;

int HealthScore = 100;

String HealthCondition = "EXCELLENT";
//================== MOTOR CONTROL ==================

#define MOTOR_ENABLE_PIN 25      // Change if your driver uses another pin

bool MotorRunning = true;
//================ PWM ==================

#define PWM_PIN 26        // Change if your driver PWM pin is different

#define PWM_CHANNEL 0
#define PWM_FREQ 10000
#define PWM_RESOLUTION 8

int SpeedPercent = 100;
int DutyCycle = 255;





//================== FAULT CODE ==================

String FaultCode = "F00";

String EventLog = "System Started";
//================ FAULT HISTORY ================

int FaultCounter=0;

String PreviousFault="F00";

String FaultHistory[10];

int FaultIndex=0;

unsigned long FaultTime[10];

unsigned long FaultStartTime=0;

unsigned long TotalFaultTime=0;




//========== FAULT VARIABLES ==========

String MotorStatus = "NORMAL";

bool OverCurrent = false;
bool UnderVoltage = false;
bool OverVoltage = false;
bool OverTemperature = false;
bool HighVibration = false;
//========= ADDITIONAL FAULTS ==========

bool NoLoad = false;
bool MotorStall = false;
bool SensorFailure = false;
bool PowerOverload = false;
bool CurrentFluctuation = false;
bool RapidTemperatureRise = false;
bool ShockDetected = false;
bool HighEnergy = false;
bool BlynkOffline = false;
//================ SENSOR STATUS =================

bool VoltageSensorOK = true;
bool CurrentSensorOK = true;
bool TemperatureSensorOK = true;
bool MPU6050OK = true;
bool WiFiOK = true;
bool BlynkOK = true;

String SystemStatus = "HEALTHY";

int HealthySensors = 0;

float PreviousCurrent = 0;
float PreviousTemperature = 0;

unsigned long TempTimer = 0;
unsigned long NoLoadTimer = 0;

//============== FUNCTION DECLARATIONS =====

float readADCVoltage(int pin);
float readCurrentADCVoltage();
void calibrateCurrentSensor();
void readVoltage();
void readCurrent();
void readTemperature();
void readMPU();
void calculatePower();
void calculateEnergy();
void detectFaults();
void predictiveMaintenance();
void sendBlynk();
void selfTest();
void updateRunTime();
void updateStatistics();
void advancedMaintenance();
void systemDiagnostics();


void setMotorSpeed(int speed);

void updateHealth();

void motorProtection();

void eventLogger(String event);
void displayPage1();
void displayPage2();
void displayPage3();
void displayPage4();
void displayPage5();
void displayPage6();
void displayPage7();

//================ SETUP ===================

void setup() {
  Serial.begin(115200);
  preferences.begin("BLDC", false);

Energy = preferences.getFloat("Energy",0);

RunTime = preferences.getULong("RunTime",0);

FaultCounter = preferences.getInt("FaultCount",0);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);
  ledcSetup(PWM_CHANNEL,PWM_FREQ,PWM_RESOLUTION);

  ledcAttachPin(PWM_PIN,PWM_CHANNEL);

  ledcWrite(PWM_CHANNEL,255);

RunTimer = millis();

  digitalWrite(MOTOR_ENABLE_PIN,HIGH);

    analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);

    Wire.begin(21, 22);
    SPI.begin(18, -1, 23, OLED_CS);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);
  delay(100);

  oled.begin();
  oled.setPowerSave(0);
  oled.setContrast(255);
  selfTest();

  oled.clearBuffer();
  
  oled.setFont(u8g2_font_7x14B_tf);
  oled.drawStr(5, 25, "BLDC MONITOR");
  oled.drawStr(10, 55, "Initializing");
  oled.sendBuffer();

  // MPU6050
  if (mpu.begin()) {
    mpuAvailable = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 not found; continuing");
  }

  // DS18B20
  ds18b20.begin();
  ds18b20.setWaitForConversion(true);

  // ACS712 calibration prompt
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);
  oled.drawStr(0, 25, "CURRENT CALIBRATION");
  oled.drawStr(0, 50, "Motor must be OFF");
  oled.drawStr(0, 70, "Zero current required");
  oled.sendBuffer();

  Serial.println();
  Serial.println("ACS712 calibration starts in 5 seconds.");
  Serial.println("Keep motor OFF and ensure zero current flows.");
  delay(5000);

  calibrateCurrentSensor();

  lastEnergyTime = millis();
  TempTimer = millis();
  pageTimer = millis();

  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tf);
  oled.drawStr(20, 45, "READY");
  oled.sendBuffer();

  Serial.println("System ready.");
  delay(1000);
  for(int i=0;i<10;i++)
{
    FaultHistory[i]="EMPTY";
    FaultTime[i]=0;
}
}

//================ MAIN LOOP ===============

void loop() {
  Blynk.run();
  readTemperature();
  readVoltage();
  readCurrent();
  readMPU();
  updateStatistics();
  advancedMaintenance();
  systemDiagnostics();
  performanceAnalysis();
  saveProjectData();

  calculatePower();
  calculateEnergy();
  detectFaults();
  updateFaultHistory();
  if(FaultCode!="F00")
{
    TotalFaultTime+=500;
}
  motorProtection();
  predictiveMaintenance();
  updateHealth();

  updateRunTime();

  if (millis() - pageTimer >= 3000) {
    pageTimer = millis();
    page = (page + 1) % 12;
  }

  switch (page) {
    case 0: displayPage1(); break;
    case 1: displayPage2(); break;
    case 2: displayPage3(); break;
    case 3: displayPage4(); break;
    case 4: displayPage5(); break;
    case 5: displayPage6(); break;
    case 6: displayPage7();break;
    case 7: displayPage8();break;
    case 8: displayPage9();break;
    case 9: displayPage10();break;
    case 10: displayPage11();break;
    case 11: displayPage12();break;
  }

  Serial.println("------------------------------");

  Serial.print("Temperature: ");
  Serial.print(Temperature, 2);
  Serial.println(" C");

  Serial.print("Voltage: ");
  Serial.print(Voltage, 2);
  Serial.println(" V");

  Serial.print("Current: ");
  Serial.print(Current, 2);
  Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(Power, 2);
  Serial.println(" W");

  Serial.print("Energy: ");
  Serial.print(Energy, 4);
  Serial.println(" Wh");

  Serial.print("Status : ");
  Serial.println(MotorStatus);

  Serial.print("Health Score : ");
  Serial.println(HealthScore);

  Serial.print("Health State : ");
  Serial.println(HealthCondition);

  Serial.print("Maintenance : ");
  Serial.println(MaintenanceStatus);



  Serial.print("ACS GPIO voltage: ");
  Serial.print(readCurrentADCVoltage(), 3);
  Serial.println(" V");

  Serial.print("ACS zero baseline: ");
  Serial.print(zeroCurrentADCVoltage, 3);
  Serial.println(" V");

  Serial.print("ACS voltage delta: ");
  Serial.print(
    (readCurrentADCVoltage() - zeroCurrentADCVoltage)
      * CURRENT_DIVIDER_RATIO,
    3
  );
  Serial.println(" V");

  if (mpuAvailable) {
    Serial.print("Accel X/Y/Z: ");
    Serial.print(ax, 2);
    Serial.print(" / ");
    Serial.print(ay, 2);
    Serial.print(" / ");
    Serial.println(az, 2);
  } else {
    Serial.println("MPU6050 data unavailable");
  }
  static unsigned long BlynkTimer = 0;

  if (millis() - BlynkTimer > 1000)
{
    BlynkTimer = millis();

    sendBlynk();
}
  delay(500);
}


//=========== ADC VOLTAGE AVERAGING =========

float readADCVoltage(int pin) {
  const int samples = 500;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }

  float averageADC = (float)total / samples;
  return averageADC * ADC_REFERENCE / ADC_MAX;
}

float readCurrentADCVoltage() {
  return readADCVoltage(CURRENT_PIN);
}

//=========== ACS712 CALIBRATION ============

void calibrateCurrentSensor() {
  delay(1000);

  const int samples = 2000;
  uint64_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(CURRENT_PIN);
    delayMicroseconds(200);
  }

  float averageADC = (float)total / samples;
  zeroCurrentADCVoltage =
      averageADC * ADC_REFERENCE / ADC_MAX;

  Serial.print("ZERO BASELINE = ");
  Serial.print(zeroCurrentADCVoltage, 4);
  Serial.println(" V");
}

//=========== READ VOLTAGE SENSOR ===========

void readVoltage() {
  float adcVoltage = readADCVoltage(VOLTAGE_PIN);

  Voltage = adcVoltage * VOLTAGE_RATIO;

  Serial.print("ADC voltage: ");
  Serial.print(adcVoltage, 3);
  Serial.print(" V | Supply voltage: ");
  Serial.print(Voltage, 2);
  Serial.println(" V");
}

//=========== READ ACS712 CURRENT ===========

void readCurrent() {
  const int samples = 1000;
  uint64_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(CURRENT_PIN);
    delayMicroseconds(200);
  }

  float averageADC = (float)total / samples;
  float adcVoltage =
      averageADC * ADC_REFERENCE / ADC_MAX;

  float deltaVoltage =
      (adcVoltage - zeroCurrentADCVoltage)
      * CURRENT_DIVIDER_RATIO;

  float signedCurrent = deltaVoltage / ACS_SENSITIVITY;

  Current = fabs(signedCurrent);

  if (Current < 0.05) {
    Current = 0.0;
  }

  Serial.print("ADC=");
  Serial.print(adcVoltage, 4);
  Serial.print(" V | ZERO=");
  Serial.print(zeroCurrentADCVoltage, 4);
  Serial.print(" V | DELTA=");
  Serial.print(deltaVoltage, 4);
  Serial.print(" V | CURRENT=");
  Serial.print(Current, 2);
  Serial.println(" A");
}

//=========== READ DS18B20 ==================

void readTemperature() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println("DS18B20 Error");
    return;
  }

  Temperature = t;
}

//=========== READ MPU6050 ==================

void readMPU() {
  if (!mpuAvailable) return;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;

  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;
  Vibration = sqrt(ax*ax + ay*ay + az*az);
}

//=========== POWER =========================

void calculatePower() {
  Power = Voltage * Current;
}

//=========== ENERGY ========================

void selfTest()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"SYSTEM SELF TEST");
    oled.drawStr(0,35,"OLED.............OK");
    oled.sendBuffer();
    delay(700);

    oled.drawStr(0,50,"DS18B20.............OK");
    oled.sendBuffer();
    delay(700);

    oled.drawStr(0,65,"MPU6050.............OK");
    oled.sendBuffer();
    delay(700);

    oled.drawStr(0,80,"ACS712.............OK");
    oled.sendBuffer();
    delay(700);

    oled.drawStr(0,95,"Voltage.............OK");
    oled.sendBuffer();
    delay(700);

    oled.drawStr(0,115,"System Ready");
    oled.sendBuffer();
    delay(1500);
}
void calculateEnergy() {
 
  unsigned long now = millis();

  float elapsedHours =
    (now - lastEnergyTime) / 3600000.0;

  Energy += Power * elapsedHours;
  lastEnergyTime = now;
}
 
void eventLogger(String event)
{

EventLog=event;

Serial.print("EVENT : ");

Serial.println(EventLog);

}

void updateFaultHistory()
{
    if(FaultCode!=PreviousFault)
    {
        FaultCounter++;

        FaultHistory[FaultIndex]=FaultCode;

        FaultTime[FaultIndex]=millis()/1000;

        FaultIndex++;

        if(FaultIndex>=10)
            FaultIndex=0;

        PreviousFault=FaultCode;
    }
}


void motorProtection()
{
    if (OverCurrent || OverTemperature || OverVoltage)
    {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        MotorRunning = false;
    }
    else
    {
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
        MotorRunning = true;
    }
}
void updateRunTime()
{
    if(MotorRunning)
    {
        unsigned long now = millis();

        RunTime += (now - RunTimer);

        RunTimer = now;
    }
    else
    {
        RunTimer = millis();
    }
}
void setMotorSpeed(int speed)
{
    if(speed < 0)
        speed = 0;

    if(speed > 100)
        speed = 100;

    SpeedPercent = speed;

    DutyCycle = map(speed,0,100,0,255);

    ledcWrite(PWM_CHANNEL,DutyCycle);
}
void detectFaults()
{
    OverCurrent = false;
    UnderVoltage = false;
    OverVoltage = false;
    OverTemperature = false;
    HighVibration = false;
    NoLoad = false;
    MotorStall = false;
    SensorFailure = false;
    PowerOverload = false;
    CurrentFluctuation = false;
    RapidTemperatureRise = false;
    ShockDetected = false;
    HighEnergy = false;
    BlynkOffline = false;

    if (Current > 8.0)
        OverCurrent = true;

    if (Voltage < 20)
        UnderVoltage = true;

    if (Voltage > 28)
        OverVoltage = true;

    if (Temperature > 60)
        OverTemperature = true;

    if (abs(ax) > 15 || abs(ay) > 15 || abs(az) > 20)
        HighVibration = true;
    //------------------- NO LOAD -------------------

if(Current < 0.20)
{
    if(millis()-NoLoadTimer > 5000)
        NoLoad = true;
}
else
{
    NoLoadTimer = millis();
}

//------------------- MOTOR STALL -------------------

if(Current > 6.0 &&
   abs(ax) < 0.5 &&
   abs(ay) < 0.5 &&
   abs(az) < 1.5)
{
    MotorStall = true;
}

//------------------- SENSOR FAILURE -------------------

if(Temperature == DEVICE_DISCONNECTED_C)
    SensorFailure = true;

if(Voltage <= 0)
    SensorFailure = true;

if(Current < 0)
    SensorFailure = true;

if(!mpuAvailable)
    SensorFailure = true;

//------------------- POWER OVERLOAD -------------------

if(Power > 180)
    PowerOverload = true;

//------------------- CURRENT FLUCTUATION -------------------

if(abs(Current-PreviousCurrent) > 2.0)
    CurrentFluctuation = true;

PreviousCurrent = Current;

//------------------- RAPID TEMP RISE -------------------

if(millis()-TempTimer > 30000)
{
    if((Temperature-PreviousTemperature) > 5)
        RapidTemperatureRise = true;

    PreviousTemperature = Temperature;

    TempTimer = millis();
}

//------------------- SHOCK -------------------

if(abs(ax)>25 ||
   abs(ay)>25 ||
   abs(az)>30)
{
    ShockDetected = true;
}

//------------------- ENERGY -------------------

if(Energy > 1000)
    HighEnergy = true;

//------------------- BLYNK -------------------

if(!Blynk.connected())
    BlynkOffline = true;

    if (OverCurrent)
    {
        MotorStatus = "OVER CURRENT";
        FaultCode = "F01";
        eventLogger("Over Current");
    }
    else if (OverVoltage)
    {
        MotorStatus = "OVER VOLTAGE";
        FaultCode = "F03";
        eventLogger("Over Voltage");
    }
    else if (UnderVoltage)
    {
        MotorStatus = "UNDER VOLTAGE";
        FaultCode = "F02";
        eventLogger("Under Voltage");
    }
    else if (OverTemperature)
    {
        MotorStatus = "OVER TEMP";
        FaultCode = "F04";
        eventLogger("Over Temperature");
    }
    else if (HighVibration)
    {
        MotorStatus = "HIGH VIBRATION";
        FaultCode = "F05";
        eventLogger("High Vibration");
    }
    else if(NoLoad)
{
    MotorStatus="NO LOAD";
    FaultCode="F07";
    eventLogger("No Load");
}

else if(MotorStall)
{
    MotorStatus="MOTOR STALL";
    FaultCode="F08";
    eventLogger("Motor Stall");
}

else if(SensorFailure)
{
    MotorStatus="SENSOR FAILURE";
    FaultCode="F09";
    eventLogger("Sensor Failure");
}

else if(PowerOverload)
{
    MotorStatus="POWER OVERLOAD";
    FaultCode="F10";
    eventLogger("Power Overload");
}

else if(CurrentFluctuation)
{
    MotorStatus="CURRENT FLUCT";
    FaultCode="F11";
    eventLogger("Current Fluctuation");
}

else if(RapidTemperatureRise)
{
    MotorStatus="TEMP RISE";
    FaultCode="F12";
    eventLogger("Rapid Temperature");
}

else if(ShockDetected)
{
    MotorStatus="SHOCK";
    FaultCode="F13";
    eventLogger("Shock");
}

else if(HighEnergy)
{
    MotorStatus="HIGH ENERGY";
    FaultCode="F14";
    eventLogger("High Energy");
}

else if(BlynkOffline)
{
    MotorStatus="BLYNK OFFLINE";
    FaultCode="F15";
    eventLogger("Blynk Offline");
}
    else
    {
        MotorStatus = "NORMAL";
        FaultCode = "F00";
        eventLogger("Normal");
    }
}

void predictiveMaintenance()
{
    HealthScore = 100;

    ServiceRequired = false;

    // Temperature
    if(Temperature > 45)
        HealthScore -= 15;

    if(Temperature > 55)
        HealthScore -= 20;

    // Current
    if(Current > 5)
        HealthScore -= 15;

    if(Current > 7)
        HealthScore -= 20;

    // Voltage
    if(Voltage < 22)
        HealthScore -= 10;

    if(Voltage > 26)
        HealthScore -= 10;

    // Vibration
    if(abs(ax) > 10)
        HealthScore -= 10;

    if(abs(ay) > 10)
        HealthScore -= 10;

    if(abs(az) > 15)
        HealthScore -= 10;

    // Limit health score
    if(HealthScore < 0)
        HealthScore = 0;

    // Health Condition
    if(HealthScore >= 90)
        HealthCondition = "EXCELLENT";

    else if(HealthScore >= 75)
        HealthCondition = "GOOD";

    else if(HealthScore >= 60)
        HealthCondition = "WARNING";

    else if(HealthScore >= 40)
        HealthCondition = "SERVICE";

    else
        HealthCondition = "CRITICAL";

    // Maintenance Decision
    if(HealthScore < 60)
    {
        ServiceRequired = true;
        MaintenanceStatus = "SERVICE REQUIRED";
    }
    else
    {
        ServiceRequired = false;
        MaintenanceStatus = "NORMAL";
    }
}

void updateHealth()
{
    HealthScore = 100;

    if (Temperature > 45) HealthScore -= 10;
    if (Temperature > 55) HealthScore -= 20;

    if (Current > 5) HealthScore -= 20;
    if (Current > 7) HealthScore -= 20;

    if (Voltage < 22) HealthScore -= 10;
    if (Voltage > 26) HealthScore -= 10;

    if (abs(ax) > 10) HealthScore -= 10;
    if (abs(ay) > 10) HealthScore -= 10;
    if (abs(az) > 15) HealthScore -= 10;

    if (HealthScore >= 90)
        HealthCondition = "EXCELLENT";
    else if (HealthScore >= 75)
        HealthCondition = "GOOD";
    else if (HealthScore >= 60)
        HealthCondition = "WARNING";
    else if (HealthScore >= 40)
        HealthCondition = "SERVICE";
    else
        HealthCondition = "CRITICAL";
}


void updateStatistics()
{
    //================ MAXIMUM VALUES =================

    if(Voltage > MaxVoltage)
        MaxVoltage = Voltage;

    if(Current > MaxCurrent)
        MaxCurrent = Current;

    if(Temperature > MaxTemperature)
        MaxTemperature = Temperature;

    if(Power > MaxPower)
        MaxPower = Power;

    if(Energy > MaxEnergy)
        MaxEnergy = Energy;

    if(Vibration > MaxVibration)
        MaxVibration = Vibration;

    //================ MINIMUM VOLTAGE =================

    if(Voltage < MinVoltage)
        MinVoltage = Voltage;

    //================ AVERAGES =================

    SampleCount++;

    VoltageSum += Voltage;

    CurrentSum += Current;

    TemperatureSum += Temperature;

    PowerSum += Power;

    VibrationSum += Vibration;

    AvgVoltage = VoltageSum / SampleCount;

    AvgCurrent = CurrentSum / SampleCount;

    AvgTemperature = TemperatureSum / SampleCount;

    AvgPower = PowerSum / SampleCount;

    AvgVibration = VibrationSum / SampleCount;
}

void advancedMaintenance()
{
    BearingHealth = 100;
    CoolingHealth = 100;
    ElectricalHealth = 100;

    //---------------- Bearing ----------------

    if(Vibration > 18)
        BearingHealth -= 20;

    if(Vibration > 25)
        BearingHealth -= 40;

    //---------------- Cooling ----------------

    if(Temperature > 45)
        CoolingHealth -= 15;

    if(Temperature > 55)
        CoolingHealth -= 25;

    if(Temperature > 65)
        CoolingHealth -= 40;

    //---------------- Electrical ----------------

    if(Current > 5)
        ElectricalHealth -= 15;

    if(Current > 7)
        ElectricalHealth -= 25;

    if(Voltage < 22 || Voltage > 26)
        ElectricalHealth -= 15;

    //---------------- Overall ----------------

    MotorHealth =
        (BearingHealth +
         CoolingHealth +
         ElectricalHealth)/3.0;

    if(MotorHealth < 0)
        MotorHealth = 0;

    RemainingLife = MotorHealth;

    WearPercentage = 100 - RemainingLife;

    //---------------- Recommendation ----------------

    if(BearingHealth < 60)
        MaintenanceRecommendation="CHECK BEARING";

    else if(CoolingHealth < 60)
        MaintenanceRecommendation="CHECK COOLING";

    else if(ElectricalHealth < 60)
        MaintenanceRecommendation="CHECK POWER";

    else if(MotorHealth < 60)
        MaintenanceRecommendation="GENERAL SERVICE";

    else
        MaintenanceRecommendation="SYSTEM NORMAL";
}
void systemDiagnostics()
{
    HealthySensors = 0;

    //---------------- Voltage ----------------

    VoltageSensorOK = (Voltage > 0 && Voltage < 35);

    //---------------- Current ----------------

    CurrentSensorOK = (Current >= 0 && Current < 35);

    //---------------- Temperature ----------------

    TemperatureSensorOK =
    (Temperature != -127 &&
     Temperature > -20 &&
     Temperature < 125);

    //---------------- MPU6050 ----------------

    MPU6050OK = mpuAvailable;

    //---------------- WiFi ----------------

    WiFiOK = (WiFi.status() == WL_CONNECTED);

    //---------------- Blynk ----------------

    BlynkOK = Blynk.connected();

    //---------------- Count ----------------

    if(VoltageSensorOK) HealthySensors++;
    if(CurrentSensorOK) HealthySensors++;
    if(TemperatureSensorOK) HealthySensors++;
    if(MPU6050OK) HealthySensors++;
    if(WiFiOK) HealthySensors++;
    if(BlynkOK) HealthySensors++;

    //---------------- Overall ----------------

    if(HealthySensors == 6)
        SystemStatus = "HEALTHY";

    else if(HealthySensors >= 4)
        SystemStatus = "WARNING";

    else
        SystemStatus = "CRITICAL";
}
void performanceAnalysis()
{
    //---------------- Operating Hours ----------------

    OperatingHours = RunTime / 3600000.0;

    //---------------- Daily Energy ----------------

    DailyEnergy = Energy;

    //---------------- Average Power ----------------

    if(OperatingHours > 0)
        AverageDailyPower = Energy / OperatingHours;

    //---------------- Load Percentage ----------------

    LoadPercentage = (Current / 10.0) * 100;

    if(LoadPercentage > 100)
        LoadPercentage = 100;

    //---------------- Estimated Efficiency ----------------

    MotorEfficiency = 100;

    if(Current > 5)
        MotorEfficiency -= 5;

    if(Temperature > 45)
        MotorEfficiency -= 5;

    if(Vibration > 15)
        MotorEfficiency -= 5;

    if(Voltage < 22 || Voltage > 26)
        MotorEfficiency -= 5;

    if(MotorEfficiency < 0)
        MotorEfficiency = 0;

    //---------------- Estimated Power Factor ----------------

    PowerFactor = 0.95;

    if(Current > 7)
        PowerFactor = 0.88;

    if(Current > 9)
        PowerFactor = 0.82;

    //---------------- Reliability ----------------

    ReliabilityIndex = MotorHealth;

    //---------------- Performance ----------------

    PerformanceIndex =
        (MotorEfficiency +
         ReliabilityIndex) / 2;

    //---------------- Performance Status ----------------

    if(PerformanceIndex >= 90)
        PerformanceStatus = "EXCELLENT";

    else if(PerformanceIndex >= 75)
        PerformanceStatus = "GOOD";

    else if(PerformanceIndex >= 60)
        PerformanceStatus = "AVERAGE";

    else if(PerformanceIndex >= 40)
        PerformanceStatus = "POOR";

    else
        PerformanceStatus = "CRITICAL";
}
void saveProjectData()
{
    if(millis()-SaveTimer>30000)
    {
        SaveTimer=millis();

        preferences.putFloat("Energy",Energy);

        preferences.putULong("RunTime",RunTime);

        preferences.putInt("FaultCount",FaultCounter);
    }
}
void sendBlynk()
{
    Blynk.virtualWrite(V0, Voltage);
    Blynk.virtualWrite(V1, Current);
    Blynk.virtualWrite(V2, Temperature);
    Blynk.virtualWrite(V3, Power);
    Blynk.virtualWrite(V4, Energy);

    Blynk.virtualWrite(V5, FaultCode);

    Blynk.virtualWrite(V6, MotorStatus);

    Blynk.virtualWrite(V7, HealthScore);

    Blynk.virtualWrite(V8, MaintenanceStatus);

    Blynk.virtualWrite(V9, RunTime/3600000.0);
    Blynk.virtualWrite(V10, ax);
    Blynk.virtualWrite(V11, ay);
    Blynk.virtualWrite(V12, az);

    Blynk.virtualWrite(V13, gx);
    Blynk.virtualWrite(V14, gy);
    Blynk.virtualWrite(V15, gz);
    Blynk.virtualWrite(V16,FaultCounter);

    Blynk.virtualWrite(V17,TotalFaultTime/1000);

    Blynk.virtualWrite(V18,FaultCode);
    Blynk.virtualWrite(V19,MaxCurrent);

    Blynk.virtualWrite(V20,MaxTemperature);

    Blynk.virtualWrite(V21,MaxPower);

    Blynk.virtualWrite(V22,MaxVibration);

    Blynk.virtualWrite(V23,AvgCurrent);

    Blynk.virtualWrite(V24,AvgTemperature);

    Blynk.virtualWrite(V25,AvgPower);

    Blynk.virtualWrite(V26,AvgVibration);
    Blynk.virtualWrite(V27,MotorHealth);

    Blynk.virtualWrite(V28,BearingHealth);

    Blynk.virtualWrite(V29,CoolingHealth);

    Blynk.virtualWrite(V30,ElectricalHealth);

    Blynk.virtualWrite(V31,RemainingLife);

    Blynk.virtualWrite(V32,WearPercentage);

    Blynk.virtualWrite(V33,MaintenanceRecommendation);
    Blynk.virtualWrite(V34,SystemStatus);

Blynk.virtualWrite(V35,HealthySensors);

Blynk.virtualWrite(V36,WiFiOK);

Blynk.virtualWrite(V37,BlynkOK);

Blynk.virtualWrite(V38,MPU6050OK);

Blynk.virtualWrite(V39,TemperatureSensorOK);

Blynk.virtualWrite(V40,VoltageSensorOK);

Blynk.virtualWrite(V41,CurrentSensorOK);

Blynk.virtualWrite(V42,MotorEfficiency);

Blynk.virtualWrite(V43,PerformanceIndex);

Blynk.virtualWrite(V44,ReliabilityIndex);

Blynk.virtualWrite(V45,LoadPercentage);

Blynk.virtualWrite(V46,OperatingHours);

Blynk.virtualWrite(V47,AverageDailyPower);

Blynk.virtualWrite(V48,DailyEnergy);

Blynk.virtualWrite(V49,PerformanceStatus);
}



//=========== OLED PAGE 1 ===================

void displayPage1() {
  oled.clearBuffer();

oled.setFont(u8g2_font_7x14B_tf);

oled.drawStr(5,20,"AMRITA");

oled.drawStr(5,40,"VISHWA");

oled.drawStr(5,60,"VIDYAPEETHAM");

oled.drawStr(5,90,"POWER ELECTRONICS LAB ");

oled.drawStr(5,110,"PROJECT");

oled.sendBuffer();

}

void displayPage2() {
  oled.clearBuffer();

  oled.setFont(u8g2_font_7x14B_tf);

oled.drawStr(0,10,"BLDC DRIVE SYSTEM");



oled.setFont(u8g2_font_6x12_tf);


  oled.setCursor(0, 40);
  oled.print("Voltage: ");
  oled.print(Voltage, 1);
  oled.print(" V");

  oled.setCursor(0, 60);
  oled.print("Current: ");
  oled.print(Current, 2);
  oled.print(" A");

  oled.setCursor(0, 80);
  oled.print("Power: ");
  oled.print(Power, 1);
  oled.print(" W");

  oled.setCursor(0, 100);
  oled.print("Energy: ");
  oled.print(Energy, 2);
  oled.print(" Wh");

  oled.sendBuffer();
}

//=========== OLED PAGE 2 ===================

void displayPage3() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tf);
  oled.drawStr(5, 15, "TEMPERATURE");
  oled.setFont(u8g2_font_6x12_tf);

  oled.setCursor(0, 45);
  oled.print("Temp: ");
  oled.print(Temperature, 1);
  oled.print(" C");

  oled.setCursor(0, 70);
  oled.print("Status: ");
  oled.print(Temperature > 60 ? "HOT" : "NORMAL");

  oled.sendBuffer();
}

//=========== OLED PAGE 3 ===================

void displayPage4() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tf);
  oled.drawStr(5, 15, "VIBRATION");
  oled.setFont(u8g2_font_6x12_tf);

  oled.setCursor(0, 40);
  oled.print("AX: "); oled.print(ax, 2);

  oled.setCursor(0, 60);
  oled.print("AY: "); oled.print(ay, 2);

  oled.setCursor(0, 80);
  oled.print("AZ: "); oled.print(az, 2);

  oled.sendBuffer();
}

//=========== OLED PAGE 4 ===================

void displayPage5() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tf);
  oled.drawStr(5, 15, "SYSTEM");
  oled.setFont(u8g2_font_6x12_tf);

  oled.setCursor(0, 40);
  oled.print("Motor: ");
  oled.print(Current > 0.30 ? "RUNNING" : "STOPPED");

  oled.setCursor(0, 60);
  oled.print("Current: ");
  oled.print(Current > 15 ? "HIGH" : "NORMAL");

  oled.setCursor(0, 80);
  if (Voltage < 18) {
    oled.print("Voltage: LOW");
  } else if (Voltage > 26) {
    oled.print("Voltage: HIGH");
  } else {
    oled.print("Voltage: NORMAL");
  }

  oled.setCursor(0, 100);
  if (fabs(ax) > 15 || fabs(ay) > 15 || fabs(az) > 20) {
    oled.print("Vibration: HIGH");
  } else {
    oled.print("Vibration: NORMAL");
  }

  oled.sendBuffer();
}

void displayPage6()
{

oled.clearBuffer();

oled.setFont(u8g2_font_7x14B_tf);

oled.drawStr(0,15,"FAULT STATUS");

oled.setFont(u8g2_font_6x12_tf);

oled.setCursor(0,40);

oled.print("Code : ");

oled.print(FaultCode);

oled.setCursor(0,60);

oled.print(MotorStatus);

oled.setCursor(0,80);

oled.print("Motor : ");

if(MotorRunning)
oled.print("RUNNING");

else
oled.print("STOPPED");

oled.setCursor(0,100);

oled.print("Event:");

oled.setCursor(0,115);

oled.print(EventLog);

oled.setCursor(0,127);

oled.print("Count:");

oled.print(FaultCounter);

oled.sendBuffer();

}
void displayPage7()
{

oled.clearBuffer();

oled.setFont(u8g2_font_7x14B_tf);

oled.drawStr(0,15,"MOTOR HEALTH");

oled.setFont(u8g2_font_6x12_tf);

oled.setCursor(0,40);

oled.print("Health:");

oled.print(HealthScore);

oled.print("%");

oled.setCursor(0,60);

oled.print(HealthCondition);

oled.setCursor(0,80);

oled.print("Run:");

oled.print(RunTime/3600000);

oled.print(" hr ");

oled.print((RunTime%3600000)/60000);

oled.print(" min");

oled.setCursor(0,105);

oled.print("Speed:");

oled.print(SpeedPercent);

oled.print("%");

oled.sendBuffer();

}
void displayPage8()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"FAULT HISTORY");

    for(int i=0;i<5;i++)
    {
        oled.setCursor(0,35+i*15);

        oled.print(FaultHistory[i]);
    }

    oled.sendBuffer();
}
void displayPage9()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"STATISTICS");

    oled.setCursor(0,35);
    oled.print("Max I:");
    oled.print(MaxCurrent,1);

    oled.setCursor(0,50);
    oled.print("Max T:");
    oled.print(MaxTemperature,1);

    oled.setCursor(0,65);
    oled.print("Max P:");
    oled.print(MaxPower,1);

    oled.setCursor(0,80);
    oled.print("Max Vib:");
    oled.print(MaxVibration,1);

    oled.setCursor(0,95);
    oled.print("Avg I:");
    oled.print(AvgCurrent,1);

    oled.setCursor(0,110);
    oled.print("Avg T:");
    oled.print(AvgTemperature,1);

    oled.sendBuffer();
}
void displayPage10()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"PREDICTIVE");

    oled.setCursor(0,35);
    oled.print("Motor:");
    oled.print(MotorHealth,0);
    oled.print("%");

    oled.setCursor(0,50);
    oled.print("Bearing:");
    oled.print(BearingHealth,0);
    oled.print("%");

    oled.setCursor(0,65);
    oled.print("Cooling:");
    oled.print(CoolingHealth,0);
    oled.print("%");

    oled.setCursor(0,80);
    oled.print("Electrical:");
    oled.print(ElectricalHealth,0);
    oled.print("%");

    oled.setCursor(0,95);
    oled.print("Life:");
    oled.print(RemainingLife,0);
    oled.print("%");

    oled.setCursor(0,110);
    oled.print(MaintenanceRecommendation);

    oled.sendBuffer();
}
void displayPage11()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"SYSTEM STATUS");

    oled.setCursor(0,35);
    oled.print("WiFi : ");
    oled.print(WiFiOK ? "OK":"FAIL");

    oled.setCursor(0,50);
    oled.print("Blynk: ");
    oled.print(BlynkOK ? "OK":"FAIL");

    oled.setCursor(0,65);
    oled.print("MPU : ");
    oled.print(MPU6050OK ? "OK":"FAIL");

    oled.setCursor(0,80);
    oled.print("Temp: ");
    oled.print(TemperatureSensorOK ? "OK":"FAIL");

    oled.setCursor(0,95);
    oled.print("Volt: ");
    oled.print(VoltageSensorOK ? "OK":"FAIL");

    oled.setCursor(0,110);
    oled.print("Curr: ");
    oled.print(CurrentSensorOK ? "OK":"FAIL");

    oled.sendBuffer();
}
void displayPage12()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    oled.drawStr(0,15,"PERFORMANCE");

    oled.setCursor(0,35);
    oled.print("Eff:");
    oled.print(MotorEfficiency,0);
    oled.print("%");

    oled.setCursor(0,50);
    oled.print("Load:");
    oled.print(LoadPercentage,0);
    oled.print("%");

    oled.setCursor(0,65);
    oled.print("PF:");
    oled.print(PowerFactor,2);

    oled.setCursor(0,80);
    oled.print("Perf:");
    oled.print(PerformanceIndex,0);
    oled.print("%");

    oled.setCursor(0,95);
    oled.print("Reli:");
    oled.print(ReliabilityIndex,0);
    oled.print("%");

    oled.setCursor(0,110);
    oled.print(PerformanceStatus);
    oled.setCursor(0,125);
    oled.print(FirmwareVersion);

    oled.sendBuffer();
}
