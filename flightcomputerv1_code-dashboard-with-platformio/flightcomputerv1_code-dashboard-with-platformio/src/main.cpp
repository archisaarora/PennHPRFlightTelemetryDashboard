#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "BMI088.h"

/* ===================== PIN MAP ===================== */
constexpr uint8_t PIN_BARO_CS  = 10;
constexpr uint8_t PIN_ACCEL_CS = 37;
constexpr uint8_t PIN_GYRO_CS  = 36;
constexpr uint8_t PIN_MAG_CS   = 38;   // ⭐ CORRECT PIN ⭐

/* ===================== BMI088 ===================== */
Bmi088Accel accel(SPI, PIN_ACCEL_CS);
Bmi088Gyro  gyro (SPI, PIN_GYRO_CS);

/* ===================== SPI SETTINGS ===================== */
SPISettings spiBaro(20000000, MSBFIRST, SPI_MODE0);
SPISettings spiMag (10000000, MSBFIRST, SPI_MODE0);

/* ===================== BAROMETER ===================== */
uint16_t baroC[8];

void baroReset() {
  SPI.beginTransaction(spiBaro);
  digitalWrite(PIN_BARO_CS, LOW);
  SPI.transfer(0x1E);
  digitalWrite(PIN_BARO_CS, HIGH);
  SPI.endTransaction();
}

void baroReadPROM() {
  for (int i = 0; i < 8; i++) {
    SPI.beginTransaction(spiBaro);
    digitalWrite(PIN_BARO_CS, LOW);
    SPI.transfer(0xA0 | (i << 1));
    uint8_t hi = SPI.transfer(0);
    uint8_t lo = SPI.transfer(0);
    digitalWrite(PIN_BARO_CS, HIGH);
    SPI.endTransaction();
    baroC[i] = (hi << 8) | lo;
  }
}

uint32_t baroADC(uint8_t cmd) {
  SPI.beginTransaction(spiBaro);
  digitalWrite(PIN_BARO_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_BARO_CS, HIGH);
  SPI.endTransaction();
  delay(10);

  SPI.beginTransaction(spiBaro);
  digitalWrite(PIN_BARO_CS, LOW);
  SPI.transfer(0x00);
  uint32_t v = (uint32_t)SPI.transfer(0) << 16 |
               (uint32_t)SPI.transfer(0) << 8  |
               (uint32_t)SPI.transfer(0);
  digitalWrite(PIN_BARO_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void readBaro(float &T, float &P) {
  uint32_t D2 = baroADC(0x58);
  double dT = D2 - (double)baroC[5] * 256.0;
  double TEMP = 2000.0 + dT * baroC[6] / 8388608.0;

  uint32_t D1 = baroADC(0x48);
  double OFF  = baroC[2] * 131072.0 + baroC[4] * dT / 64.0;
  double SENS = baroC[1] * 65536.0  + baroC[3] * dT / 128.0;
  double PRES = (D1 * SENS / 2097152.0 - OFF) / 32768.0;

  T = TEMP / 100.0;
  P = PRES;
}

/* ===================== MAGNETOMETER (LIS2MDL) ===================== */
#define MAG_WHOAMI  0x4F
#define MAG_STATUS  0x67
#define MAG_OUTX_L  0x68
#define MAG_CFG_A   0x60
#define MAG_CFG_B   0x61
#define MAG_CFG_C   0x62

uint8_t magRead(uint8_t reg) {
  SPI.beginTransaction(spiMag);
  digitalWrite(PIN_MAG_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_MAG_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void magWrite(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(spiMag);
  digitalWrite(PIN_MAG_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(PIN_MAG_CS, HIGH);
  SPI.endTransaction();
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(PIN_BARO_CS, OUTPUT);
  pinMode(PIN_ACCEL_CS, OUTPUT);
  pinMode(PIN_GYRO_CS, OUTPUT);
  pinMode(PIN_MAG_CS, OUTPUT);

  digitalWrite(PIN_BARO_CS, HIGH);
  digitalWrite(PIN_ACCEL_CS, HIGH);
  digitalWrite(PIN_GYRO_CS, HIGH);
  digitalWrite(PIN_MAG_CS, HIGH);

  SPI.begin();
  delay(50);

  // BMI088 INIT
  if (accel.begin() < 0 || gyro.begin() < 0) {
    Serial.println("BMI088 FAIL");
    while (1);
  }

  accel.setRange(Bmi088Accel::RANGE_6G);
  accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);
  gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
  gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);

  Serial.println("BMI088 OK");

  // BARO INIT
  baroReset();
  delay(10);
  baroReadPROM();

  Serial.println("MS5607 OK");

  // ⭐ MAG INIT (PROVEN CONFIG) ⭐
  magWrite(MAG_CFG_C, 0x34);
  magWrite(MAG_CFG_B, 0x0C);
  magWrite(MAG_CFG_A, 0x8C);
  delay(10);

  uint8_t mag_id = magRead(MAG_WHOAMI);

  Serial.print("MAG WHOAMI = 0x");
  Serial.println(mag_id, HEX);

  Serial.println("TIME,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,T,P");
}

/* ===================== LOOP ===================== */
void loop() {

  accel.readSensor();
  gyro.readSensor();

  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();

  float gx = gyro.getGyroX_rads() * 180.0f / PI;
  float gy = gyro.getGyroY_rads() * 180.0f / PI;
  float gz = gyro.getGyroZ_rads() * 180.0f / PI;

  static int16_t mx = 0;
  static int16_t my = 0;
  static int16_t mz = 0;

  if (magRead(MAG_STATUS) & 0x08) {
      mx = (int16_t)(magRead(MAG_OUTX_L+1) << 8 | magRead(MAG_OUTX_L));
      my = (int16_t)(magRead(MAG_OUTX_L+3) << 8 | magRead(MAG_OUTX_L+2));
      mz = (int16_t)(magRead(MAG_OUTX_L+5) << 8 | magRead(MAG_OUTX_L+4));
  }

  float T = 0, P = 0;
  readBaro(T, P);

  Serial.print(">");

  Serial.print("AX:"); Serial.print(ax,3); Serial.print(",");
  Serial.print("AY:"); Serial.print(ay,3); Serial.print(",");
  Serial.print("AZ:"); Serial.print(az,3); Serial.print(",");

  Serial.print("GX:"); Serial.print(gx,2); Serial.print(",");
  Serial.print("GY:"); Serial.print(gy,2); Serial.print(",");
  Serial.print("GZ:"); Serial.print(gz,2); Serial.print(",");

  Serial.print("MX:"); Serial.print(mx); Serial.print(",");
  Serial.print("MY:"); Serial.print(my); Serial.print(",");
  Serial.print("MZ:"); Serial.print(mz); Serial.print(",");

  Serial.print("T:"); Serial.print(T,2); Serial.print(",");
  Serial.print("P:"); Serial.println(P/1000.0,3);

  delay(100);
}