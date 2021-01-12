/* CHANGE LOG
   1.0 PRE VER
   1.1
   -- CHANGED : VIBRATION CALCULATION REVISED TO TOTAL MAGNITUDE INSTEAD OF SINGLE AXIS
   1.2
   -- ADDED   : THING SPEAK IMPLEMENTATION
   -- CHANGED : WIFI CLIENT MODE
   -- REMOVED : MODBUS SUPPORT
*/
#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <ESP8266WiFi.h>

bool printDebug = false;
long DebugLastPrintTime = 0;
long timeout;
enum ConditionStatus {
  NORMAL,
  DANGER,
};
ConditionStatus conditionStatus = NORMAL;

enum ScreenDisplay {
  INIT,
  POSTINIT,
  MPUFAIL,
  WIFIFAIL,
  SERVFAIL,
  CALIB,
};

// USER DEFINED CONST
const int MAX_WIND = 1, MAX_LEVEL = 60, MAX_VIBR = 1;
const int WIND_HYS = 0, LEVEL_HYS = 5, VIBR_HYS = 1;
//-------------------

long lastNormalFlag, lastDangerFlag;
long lastNormalTime, lastDangerTime;

//WIFI PARAM
const char *ssid = "inspirasi_kita_bersama";
const char *pass = "ifmelectronic";

//THINGSPEAK PARAM
unsigned long chNumber = 1278883;
const char *apiKey = "9UHHAHLE6R1U07XA";
WiFiClient _wifiClient;
long lastPushTime;

//ACCELERO PARAM
float normVibration, normTemperature;
float accelGravityCompensator = 0;
Adafruit_MPU6050 mpu;

//WIND SPEED PARAM
float MMA_TACH_SUM, MMA_TACH_AVER;
const uint16_t PULSE_MINIMUM = 18;
const uint8_t PULSE_PER_REV = 18;
const float DISTANCE_PER_REV = 0.1257;
const byte RPM_MMA_SAMPLING_DATA_COUNT = 2;
volatile int TOTAL_PULSE = 0;
float normWindSpeed;
long timeold;
const uint8_t windPin = 14; //D5
volatile uint32_t lastMicros;

//WATER LEVEL PARAM
int normWaterLevel;
long lastAnalogReadTime = 0;

//SERVO PARAM
Servo gateControl;
const byte gatePin = 12; //D6

//LCD PARAM
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int updateInterval = 1000;
long LCDlastUpdateTime;

//buzzer param
const byte buzzerPin = 13; //D7

ICACHE_RAM_ATTR void rpmFun() {
  if (micros() - lastMicros > 500) {
    TOTAL_PULSE++;
    lastMicros = micros();
  }
}

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);

  WiFi.begin(ssid, pass);

  timeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    initLCD(INIT);
    if (millis() - timeout >= 10000) {
      lcd.clear();
      while (1) initLCD(WIFIFAIL);
    }
  }
 
  while (!mpu.begin()) {
    initLCD(MPUFAIL);
    delay(1000);
  }

  initLCD(CALIB);
  calibrateVibration();

  timeout = millis();
  while (!ThingSpeak.begin(_wifiClient)) {
    if (millis() - timeout >= 10000) {
      lcd.clear();
      while (1) initLCD(SERVFAIL);
    }
  }

  initLCD(POSTINIT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  attachInterrupt(digitalPinToInterrupt(windPin), rpmFun, FALLING);
  gateControl.attach(gatePin);
}

void loop() {
  getSensorValue();

  if (normWindSpeed > MAX_WIND + WIND_HYS  || normWaterLevel > MAX_LEVEL + LEVEL_HYS || normVibration > MAX_VIBR + VIBR_HYS) {
    conditionStatus = DANGER;
  }
  else if (normWindSpeed < MAX_WIND - WIND_HYS  || normWaterLevel < MAX_LEVEL - LEVEL_HYS || normVibration < MAX_VIBR - VIBR_HYS) {
    conditionStatus = NORMAL;
  }

  switch (conditionStatus) {
    case NORMAL:
      lastNormalTime = millis();
      if (lastNormalTime - lastDangerFlag >= 5000) {
        gateControl.write(90);
        digitalWrite(buzzerPin, HIGH);
      }
      lastNormalFlag = millis();
      break;
    case DANGER:
      lastDangerTime = millis();
      if (lastDangerTime - lastNormalFlag >= 5000) {
        gateControl.write(0);
        digitalWrite(buzzerPin, LOW);
      }
      lastDangerFlag = millis();
      break;
  }

  if (millis() - lastPushTime >= 20000) {
    lastPushTime = millis();
    ThingSpeak.setField(1, normTemperature);
    ThingSpeak.setField(2, normVibration);
    ThingSpeak.setField(3, normWaterLevel);
    ThingSpeak.setField(4, normWindSpeed);
    if (ThingSpeak.writeFields(chNumber, apiKey) != 200) {
      lcd.setCursor(15, 0);
      lcd.print("!");
    } else {
      lcd.setCursor(15, 0);
      lcd.print(" ");
    }
  }

  if (Serial.available() > 0) {
    char a = Serial.read();
    if ( a == 'p' ) printDebug = true;
    else printDebug = false;
  }

  if ((millis() - LCDlastUpdateTime) >= updateInterval) {
    LCDlastUpdateTime = millis();
    char buf[6] = "";
    dtostrf(normVibration, 5, 2, buf);
    lcd.setCursor(2, 0); lcd.print(buf);
    sprintf(buf, "%03i", normWaterLevel);
    lcd.setCursor(11, 0); lcd.print(buf);
    dtostrf(normWindSpeed, 5, 2, buf);
    lcd.setCursor(2, 1); lcd.print(buf);
    dtostrf(normTemperature, 4, 1, buf);
    lcd.setCursor(11, 1); lcd.print(buf);
  }

  if (printDebug && millis() - DebugLastPrintTime >= 1000) {
    DebugLastPrintTime = millis();
    Serial.print(normVibration); Serial.print(" ");
    Serial.print(normTemperature); Serial.print(" ");
    Serial.print(normWindSpeed); Serial.print(" ");
    Serial.print(normWaterLevel); Serial.print(" ");
    Serial.print(conditionStatus); Serial.println();
  }
}
//void getSensorValue() {
//  normVibration = random(0, 500);
//  normTemperature = random(0, 100);
//  normWindSpeed = random(0, 100);
//  normWaterLevel = random(0, 100);
//}
void getSensorValue() {
  // vibration
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  normVibration = fabs(sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z) - accelGravityCompensator);
  normTemperature = temp.temperature;

  //wind speed
  if (millis() - timeold >= 2000) normWindSpeed = 0;
  if (TOTAL_PULSE >= PULSE_MINIMUM) {
    double totalTime = (double)(millis() - timeold) / 1000;
    float revPerPulse = (float) TOTAL_PULSE / PULSE_PER_REV;
    MMA_TACH_SUM -= MMA_TACH_AVER;
    MMA_TACH_SUM += (revPerPulse / totalTime);
    MMA_TACH_AVER = MMA_TACH_SUM / RPM_MMA_SAMPLING_DATA_COUNT;
    TOTAL_PULSE = 0;
    timeold = millis();
    normWindSpeed = MMA_TACH_AVER * DISTANCE_PER_REV;
  }

  if (millis() - lastAnalogReadTime >= 1000) {
    lastAnalogReadTime = millis();
    normWaterLevel = map(analogRead(A0), 0, 1024, 0, 100);
    if (normWaterLevel > 100) normWaterLevel = 100;
  }
}

void calibrateVibration() {
  float calibrationBuffer = 0;
  for (int i = 0; i < 50; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    calibrationBuffer += sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  }
  accelGravityCompensator = calibrationBuffer / 50;
  delay(2000);
}

void initLCD(ScreenDisplay stat) {
  switch (stat) {
    case INIT:
      lcd.setCursor(0, 0);
      lcd.print("INITIALIZE");
      lcd.setCursor(0, 1);
      lcd.print("CONNECTING..");
      delay(2000);
      break;
    case WIFIFAIL:
      lcd.setCursor(0, 0);
      lcd.print("WIFI FAULT ");
      lcd.setCursor(0, 1);
      lcd.print("FAIL TO CONNECT");
      break;
    case MPUFAIL:
      lcd.setCursor(0, 0);
      lcd.print("MPU FAULT ");
      lcd.setCursor(0, 1);
      lcd.print("VIB SENSOR FAIL");
      break;
    case CALIB:
      lcd.setCursor(0, 0);
      lcd.print("INITIALIZE");
      lcd.setCursor(0, 1);
      lcd.print("CALIBRATING MPU");
      break;
    case SERVFAIL:
      lcd.setCursor(0, 0);
      lcd.print("SERVER FAULT");
      lcd.setCursor(0, 1);
      lcd.print("FAIL TO CONNECT");
      break;
    case POSTINIT:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IP ADDR :");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      delay(3000);
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("V:"); //2-6
      lcd.setCursor(8, 0); lcd.print("L:"); //10-12
      lcd.setCursor(0, 1); lcd.print("W:"); //2-6
      lcd.setCursor(8, 1); lcd.print("T:"); //10-13
      break;
  }
}
