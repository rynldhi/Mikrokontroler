#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "BluetoothSerial.h"

// ====== Bluetooth Setup ======
BluetoothSerial SerialBT;

// ====== DS18B20 Setup ======
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long lastTempRequest = 0;
const unsigned long tempInterval = 2000;

// ====== MAX30102 Setup ======
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Diabetes");  // Nama Bluetooth terlihat di HP

  Serial.println("Bluetooth aktif: ESP32-Diabetes");
  SerialBT.println("🔗 Bluetooth ESP32-Diabetes tersambung.");

  delay(500);

  // Inisialisasi DS18B20
  sensors.begin();
  if (sensors.getDeviceCount() == 0) {
    Serial.println("❌ DS18B20 tidak terdeteksi!");
    SerialBT.println("❌ DS18B20 tidak terdeteksi!");
  } else {
    Serial.println("✅ DS18B20 terdeteksi.");
    SerialBT.println("✅ DS18B20 terdeteksi.");
  }

  // Inisialisasi MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("❌ MAX30102 tidak terdeteksi!");
    SerialBT.println("❌ MAX30102 tidak terdeteksi!");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);
  Serial.println("✅ MAX30102 siap digunakan.");
  SerialBT.println("✅ MAX30102 siap digunakan.");

  sensors.requestTemperatures();
  lastTempRequest = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // ====== Pembacaan suhu berkala ======
  if (currentMillis - lastTempRequest >= tempInterval) {
    float suhu = sensors.getTempCByIndex(0);

    if (suhu != DEVICE_DISCONNECTED_C) {
      Serial.print("🌡 Suhu: ");
      Serial.print(suhu);
      Serial.println(" °C");

      SerialBT.print("🌡 Suhu: ");
      SerialBT.print(suhu);
      SerialBT.println(" °C");
    } else {
      Serial.println("❌ Gagal baca suhu.");
      SerialBT.println("❌ Gagal baca suhu.");
    }

    sensors.requestTemperatures();
    lastTempRequest = currentMillis;
  }

  // ====== Detak Jantung ======
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = currentMillis - lastBeat;
    lastBeat = currentMillis;

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }

    Serial.print("❤ BPM: ");
    Serial.print(beatsPerMinute);
    Serial.print(" | Avg: ");
    Serial.println(beatAvg);

    SerialBT.print("❤ BPM: ");
    SerialBT.print(beatsPerMinute);
    SerialBT.print(" | Avg: ");
    SerialBT.println(beatAvg);
  }

  delay(20);
}