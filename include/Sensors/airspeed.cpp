#include "airspeed.h"

KalmanFilter kalmanFilter;

// Mendapatkan data mentah dari sensor
byte Airspeed::getRawAirspeed() {
    byte status, press_h, press_l, temp_h, temp_l;
    Wire2.beginTransmission(MS4525DAddress);
    delay(100);
    Wire2.requestFrom(MS4525DAddress, static_cast<uint8_t>(4), static_cast<uint8_t>(false));
    delay(100);

    press_h = Wire2.read();
    press_l = Wire2.read();
    temp_h  = Wire2.read();
    temp_l  = Wire2.read();

    Wire2.endTransmission();

    status  = (press_h >> 6) & 0x03;
    press_h = press_h & 0x3f;
    press_data = (((uint16_t)press_h) << 8) | press_l;

    temp_l = ((temp_l) >> 5);
    temp_data = (((uint16_t)temp_h) << 3) | temp_l;

    return status;
}

// Kalibrasi sensor kecepatan angin
void Airspeed::calibrateAirspeed() {
    stats      = getRawAirspeed();
    init_press = 20.0f;
    for (int i = 0; i < 100; i++) {
        press_psi = ((static_cast<float>(static_cast<int16_t>(press_data) - MS4525ZeroCounts)) /
                     static_cast<float>(MS4525Span) *
                     static_cast<float>(MS4525FullScaleRange) - psi_offset);
        press_psi = abs(press_psi);
        v_ms = sqrt((press_psi * 13789.5144) / 1.225);
        if (v_ms < init_press) {
            init_press = v_ms;
        }
        delay(50);
    }
}

// Inisialisasi sensor
void Airspeed::initAirspeed() {
    Wire2.begin();
    stats      = getRawAirspeed();
    psi_offset = abs((static_cast<float>(static_cast<int16_t>(press_data) - MS4525ZeroCounts)) /
                     static_cast<float>(MS4525Span) *
                     static_cast<float>(MS4525FullScaleRange));

    Serial.print("Press raw offset: ");
    Serial.print(press_data);
    Serial.print("\t ");
    Serial.print("Psi offset: ");
    Serial.println(psi_offset, 6);

    kalmanFilter.initializeKalman(4, 0.3, 0.1);
    randomSeed(A15);
}

// Memperbarui data kecepatan angin
void Airspeed::updateAirspeed() {
    stats     = getRawAirspeed();
    press_psi = ((static_cast<float>(static_cast<int16_t>(press_data) - MS4525ZeroCounts)) /
                 static_cast<float>(MS4525Span) *
                 static_cast<float>(MS4525FullScaleRange) - psi_offset);
    press_psi = abs(press_psi);

    v_ms  = sqrt((press_psi * 13789.5144) / 1.225);
    // v_ms = kalmanFilter.kalmanFilter(v_ms);
    v_kmh  = v_ms * 3.6;
    prspsi = press_psi * 13789.5144;
}