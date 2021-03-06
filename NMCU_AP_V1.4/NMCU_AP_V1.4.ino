/* CHANGE LOG
   1.0 PRE VER
   1.1
   -- CHANGED : VIBRATION CALCULATION REVISED TO TOTAL MAGNITUDE INSTEAD OF SINGLE AXIS
   1.2
   -- ADDED   : THING SPEAK IMPLEMENTATION
   -- ADDED   : LCD STATS
   -- ADDED   : COMMENT
   -- CHANGED : WIFI CLIENT MODE
   -- REMOVED : MODBUS SUPPORT
   1.3
   -- ADDED   : MOVING AVERAGE FOR VIBRATION
   1.4
   -- ADDED   : LOAD CELL IMPLEMENTATION
   -- ADDED   : NEW THINGSPEAK FIELD 5 FOR LOADCELL
   -- REMOVED : TEMPERATURE DISPLAY IN LCD
   -- CHANGED : LOADCELL DISPLAY IN LCD
   -- CHANGED : PINOUT BUZZER, AND WIND SENSOR
*/

#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <HX711.h>

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
  CALIBMPU,
  CALIBCELL,
};

// USER DEFINED CONST
const float MAX_WIND = 0.25, MAX_LEVEL = 30, MAX_VIBR = 0.40, MAX_WEIGHT = 0.2, WEIGHT_HYS = 0.1;
const int WIND_HYS = 0, LEVEL_HYS = 0, VIBR_HYS = 0;
//-------------------

long lastNormalFlag, lastDangerFlag;
long lastNormalTime, lastDangerTime;

//WIFI PARAM
//ubah sesuai kebutuhan
const char *ssid = "fahmi";
const char *pass = "indonesiapusaka";

//THINGSPEAK PARAM
unsigned long chNumber = 1278883;
const char *apiKey = "9UHHAHLE6R1U07XA";
WiFiClient _wifiClient;
long lastPushTime;

//ACCELERO PARAM
float normVibration, normTemperature;
float accelGravityCompensator = 0;
float MMA_VIBR_SUM, MMA_VIBR_AVER;
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
const uint8_t windPin = 13; //D7
volatile uint32_t lastMicros;

//WATER LEVEL PARAM
int normWaterLevel;
long lastAnalogReadTime = 0;

//LOAD CELLL PARAM
HX711 scale;
const uint8_t dataPin = 14; //d5
const uint8_t clockPin = 0; //d3
float normLoad;
const int SCALE_FACTOR = 400811.111f;
long LoadlastUpdateTime;

//SERVO PARAM
Servo gateControl;
const byte gatePin = 12; //D6

//LCD PARAM
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int updateInterval = 1000;
long LCDlastUpdateTime;

//buzzer param
const byte buzzerPin = 2; //D4

//fungsi interupt untuk mencatat pulse anemometer
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
  /* proses koneksi ke wifi, timeout 10 detik
     jika gagal tampilkan pesan wifi gagal di lcd
  */
  timeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    initLCD(INIT);
    if (millis() - timeout >= 10000) {
      lcd.clear();
      while (1) initLCD(WIFIFAIL);
    }
  }
  //inisialisasi sensor vibrasi, jika gagal tampilkan pesan mpu fail di lcd
  while (!mpu.begin()) {
    initLCD(MPUFAIL);
    delay(1000);
  }

  //tampilkan pesan proses kalibrasi di lcd
  initLCD(CALIBMPU);
  calibrateVibration();

  initLCD(CALIBCELL);
  scale.begin(dataPin, clockPin);
  calibrateLoadcell();

  //koneksi ke thingspeak, timeot 10 dtk, jika gagal tampilkan pesan server fail di lcd
  timeout = millis();
  while (!ThingSpeak.begin(_wifiClient)) {
    if (millis() - timeout >= 10000) {
      lcd.clear();
      while (1) initLCD(SERVFAIL);
    }
  }

  //tampilkan variabel ukur di lcd
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
  // logic untuk menentukan status danger atau normal
  if (normWindSpeed > MAX_WIND + WIND_HYS  ||
      normWaterLevel > MAX_LEVEL + LEVEL_HYS ||
      normVibration > MAX_VIBR + VIBR_HYS ||
      normLoad > MAX_WEIGHT + WEIGHT_HYS) {
    conditionStatus = DANGER;
  }
  else if (normWindSpeed < MAX_WIND - WIND_HYS  ||
           normWaterLevel < MAX_LEVEL - LEVEL_HYS ||
           normVibration < MAX_VIBR - VIBR_HYS ||
           normLoad < MAX_WEIGHT - WEIGHT_HYS) {
    conditionStatus = NORMAL;
  }

  //setelah logic ditentukan, tutup palang dan bunyikan alarm jika danger
  switch (conditionStatus) {
    case NORMAL:
      lastNormalTime = millis();
      //buka palang dan matikan alarm pada saat keadaan sudah normal minimal 5 dtk
      if (lastNormalTime - lastDangerFlag >= 3000) {
        gateControl.write(90);
        digitalWrite(buzzerPin, HIGH);
      }
      lastNormalFlag = millis();
      break;
    case DANGER:
      lastDangerTime = millis();
      //tutup palang dan nyalakan alarm pada saat keadaan sudah normal minimal 5 dtk
      if (lastDangerTime - lastNormalFlag >= 3000) {
        gateControl.write(0);
        digitalWrite(buzzerPin, LOW);
      }
      lastDangerFlag = millis();
      break;
  }
//  push data ke thing speak, setiap 20 dtk
    if (millis() - lastPushTime >= 15100) {
      lastPushTime = millis();
      ThingSpeak.setField(1, normLoad);
      ThingSpeak.setField(2, normVibration);
      ThingSpeak.setField(3, normWaterLevel);
      ThingSpeak.setField(4, normWindSpeed);
      ThingSpeak.setField(5, normTemperature);
  
      //jika gagal push data, tampilkan tanda seru di lcd.
      //hapus tanda seru, jika push selanjutnya berhasil
      if (ThingSpeak.writeFields(chNumber, apiKey) != 200) {
        lcd.setCursor(15, 0);
        lcd.print("!");
      } else {
        lcd.setCursor(15, 0);
        lcd.print(" ");
      }
    }
  //untuk debug, print nilai sensor jika ada serial msg 'p'.
  if (Serial.available() > 0) {
    char a = Serial.read();
    if ( a == 'p' ) printDebug = true;
    else printDebug = false;
  }
  //update tampilan lcd setiap 1 dtk
  if ((millis() - LCDlastUpdateTime) >= updateInterval) {
    LCDlastUpdateTime = millis();
    char buf[6] = "";
    dtostrf(normVibration, 5, 2, buf);
    lcd.setCursor(2, 0); lcd.print(buf);
    sprintf(buf, "%03i", normWaterLevel);
    lcd.setCursor(11, 0); lcd.print(buf);
    dtostrf(normWindSpeed, 5, 2, buf);
    lcd.setCursor(2, 1); lcd.print(buf);
    dtostrf(normLoad, 4, 3, buf);
    lcd.setCursor(11, 1); lcd.print(buf);
  }
  // debug print setiap 1 dtk jika true
  if (printDebug && millis() - DebugLastPrintTime >= 1000) {
    DebugLastPrintTime = millis();
    Serial.print(normVibration); Serial.print(" ");
    Serial.print(normTemperature); Serial.print(" ");
    Serial.print(normWindSpeed); Serial.print(" ");
    Serial.print(normWaterLevel); Serial.print(" ");
    Serial.print(normLoad); Serial.print(" ");
    Serial.print(conditionStatus); Serial.println();
  }
}

void getSensorValue() {
  // vibration
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //perhitungan magnitudo akselerasi total di sumbu xyz
  float tempVibration = fabs(sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z) - accelGravityCompensator);
  MMA_VIBR_SUM -= MMA_VIBR_AVER;
  MMA_VIBR_SUM += tempVibration;
  MMA_VIBR_AVER = MMA_VIBR_SUM / 100;
  normVibration = MMA_VIBR_AVER;
  normTemperature = temp.temperature;

  //wind speed
  //apabila setiap 2 dtk tidak ada pulse yg diterima dari sensor angin
  //reset kecepatan angin ke 0
  if (millis() - timeold >= 2000) normWindSpeed = 0;
  //logic untuk menghitung kecepatan angin
  if (TOTAL_PULSE >= PULSE_MINIMUM) {
    double totalTime = (double)(millis() - timeold) / 1000;
    float revPerPulse = (float) TOTAL_PULSE / PULSE_PER_REV;
    //filter moving average, untuk menghaluskan hasil perhitungan kecepatan angin
    MMA_TACH_SUM -= MMA_TACH_AVER;
    MMA_TACH_SUM += (revPerPulse / totalTime);
    MMA_TACH_AVER = MMA_TACH_SUM / RPM_MMA_SAMPLING_DATA_COUNT;
    TOTAL_PULSE = 0;
    timeold = millis();
    normWindSpeed = MMA_TACH_AVER * DISTANCE_PER_REV;
  }

  //water level
  //lakukan pengukuran water level setiap 1 dtk
  if (millis() - lastAnalogReadTime >= 1000) {
    lastAnalogReadTime = millis();
    // mapping pembacaan analog dari 0-1024 ke 0 sampai 100 persen
    normWaterLevel = map(analogRead(A0), 0, 500, 0, 100);
    //normWaterLevel = analogRead(A0);
    if (normWaterLevel > 100) normWaterLevel = 100;
  }

  //loadcell
  //lakukan pengukuran berat setiap 1 dtk
  if (millis() - LoadlastUpdateTime >= 1000) {
    LoadlastUpdateTime = millis();
    if (scale.wait_ready_timeout(1000)) normLoad = fabs(scale.get_units(5));
    else normLoad = 0;
  }
}

void calibrateVibration() {
  //untuk kalibrasi
  float calibrationBuffer = 0;
  for (int i = 0; i < 50; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    calibrationBuffer += sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  }
  accelGravityCompensator = calibrationBuffer / 50;
  delay(2000);
}

void calibrateLoadcell() {
  //tera loadcell
  scale.set_scale(SCALE_FACTOR);
  delay(1000);
  scale.tare(10);
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
    case CALIBMPU:
      lcd.setCursor(0, 0);
      lcd.print("INITIALIZE");
      lcd.setCursor(0, 1);
      lcd.print("CALIBRATING MPU");
      break;
    case CALIBCELL:
      lcd.setCursor(0, 0);
      lcd.print("INITIALIZE");
      lcd.setCursor(0, 1);
      lcd.print("CALIBRATING CELL");
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
      lcd.setCursor(8, 1); lcd.print("S:"); //10-13
      break;
  }
}
