#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;

String targetName = "HC-05";
String btPassword = "1234";

bool deviceFound = false;
bool btConnected = false;
esp_bd_addr_t targetAddr;

// ---------- MPU6050 ----------
const int MPU_ADDR = 0x68;

float baseX = 0, baseY = 0;
float angleX, angleY;
bool calibrated = false;

int16_t acc_x, acc_y, acc_z;

// ---------------- GAP CALLBACK ----------------
void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {

    case ESP_BT_GAP_DISC_RES_EVT: {
      char name[64] = {0};
      uint8_t len = 0;

      for (int i = 0; i < param->disc_res.num_prop; i++) {
        if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
          len = param->disc_res.prop[i].len;
          memcpy(name, param->disc_res.prop[i].val, len);
          name[len] = 0;
        }
      }

      if (len && String(name) == targetName && !deviceFound) {
        memcpy(targetAddr, param->disc_res.bda, sizeof(esp_bd_addr_t));
        deviceFound = true;
      }
      break;
    }

    default:
      break;
  }
}

// ---------------- MPU FUNCTIONS ----------------
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  calibrateNeutral();
}

void readAccel() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  angleX = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / PI;
  angleY = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / PI;
}

void calibrateNeutral() {
  float sumX = 0, sumY = 0;

  for (int i = 0; i < 100; i++) {
    readAccel();
    sumX += angleX;
    sumY += angleY;
    delay(10);
  }

  baseX = sumX / 100;
  baseY = sumY / 100;
  calibrated = true;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESP32_Scanner", true);
  esp_bt_gap_register_callback(bt_gap_cb);

  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

  unsigned long start = millis();
  while (!deviceFound && millis() - start < 15000) {
    delay(200);
  }

  esp_bt_gap_cancel_discovery();

  if (!deviceFound) return;

  esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, (uint8_t*)btPassword.c_str());
  delay(300);

  if (SerialBT.connect(targetAddr)) {
    btConnected = true;
    setupMPU();
  }
}

// ---------------- LOOP ----------------
void loop() {
  if (!btConnected || !calibrated) return;

  readAccel();

  float diffX = angleX - baseX;
  float diffY = angleY - baseY;

  char command = 'S';

  if (diffY > 15) command = 'L';
  else if (diffY < -15) command = 'R';
  else if (diffX > 15) command = 'F';
  else if (diffX < -15) command = 'B';

  if (SerialBT.hasClient()) {
    SerialBT.write(command);  // send gesture command
  }

  delay(30);  // smooth output
}
