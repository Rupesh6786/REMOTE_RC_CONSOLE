/*
  ------------- CREATED BY ROBOBOX TEAM ( Version 0.2 )
  ------------------
  ESP32 Bluetooth RC Controller (Classic SPP)
  - 16x2 I2C LCD menu (Connect / scan / device list / connect / back)
  - Rotary encoder navigation (CLK/DT/SW)
  - Two analog joysticks streamed over SPP (CSV: x1,y1,x2,y2\n)
  - Simple back "slide" animation
  - Gesure Control operation 
  Tested with Arduino-ESP32 core 2.0.x+ and HC-05 SPP modules.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>

// Low-level Classic BT GAP for discovery
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

// ------------------ LCD ------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ------------------ Bluetooth ------------------
BluetoothSerial SerialBT;

#define MAX_BT_DEVICES 10
typedef struct {
  esp_bd_addr_t addr;
  char name[64];
  bool hasName;
} BtDevice;

BtDevice devices[MAX_BT_DEVICES];
int deviceCount = 0;
bool scanning = false;
bool isConnected = false;
bool connectInProgress = false;

// ------------------ Rotary Encoder ------------------
#define ENC_CLK 32
#define ENC_DT 33
#define ENC_SW 25

volatile int encDelta = 0;
bool swPressed = false;
bool swWasDown = false;
unsigned long swDownAt = 0;
const unsigned long LONG_PRESS_MS = 600;

// ------------------ Joysticks ------------------
#define JOY_X1 34
#define JOY_Y2 26

// Calibration values
int centerX = 2870;
int centerY = 3700;

// Deadzones (separate for X and Y)
int deadZoneX = 400;  // wider for X drift
int deadZoneY = 300;  // normal for Y

// ----------Gesture Control ( MPU6050 )-----------

// === Accel globals ===
float baseX = 0, baseY = 0;
const int MPU_ADDR = 0x68;  // MPU6050 I2C address

float angleX, angleY;
bool calibrated = false;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 150;  // Send every 150ms

// ------------------ UI / State ------------------
enum UiState {
  BOOT_LOGO,
  MAIN_MENU,
  SUB_MENU,  // for Bluetooth Car / Gesture Car
  BT_SCANNING,
  BT_LIST,
  BT_CONNECTING,
  CONNECTED,
  ERROR_SCREEN,
  ABOUT_SCREEN
};

UiState ui = BOOT_LOGO;

int mainMenuIndex = 0;
const char* mainMenuItems[] = { "Bluetooth Car", "Gesture Car", "About" };
const int mainMenuCount = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);

int subMenuIndex = 0;
const char* subMenuItems[] = { "Connect", "<- Back" };
const int subMenuCount = 2;

int listIndex = 0;  // For device list
int topIndex = 0;   // For scrolling lists (if you ever add more rows)

unsigned long lastUiUpdate = 0;
const unsigned long UI_REDRAW_MS = 120;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 50;

int currentMode = 0;  // 0 = Joystick Car, 1 = Gesture Car

// ------------------ Helpers ------------------
void lcdCenterPrint(int row, const char* text) {
  int len = strlen(text);
  int col = max(0, (16 - len) / 2);
  lcd.setCursor(col, row);
  lcd.print(text);
}

void drawBootLogo() {
  lcd.clear();
  lcdCenterPrint(0, "RoboBox RC");
  lcdCenterPrint(1, "Controller");

  // simple animated loading dots
  for (int i = 0; i < 6; i++) {
    lcd.setCursor(7, 1);
    for (int j = 0; j <= (i % 4); j++) {
      lcd.print(".");
    }
    delay(400);
  }
  lcd.clear();
  lcdCenterPrint(0, "Ready!");
  delay(800);
}

void drawSubMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(subMenuItems[subMenuIndex]);
  lcd.setCursor(0, 1);
  lcd.print("  ");
  lcd.print(subMenuItems[(subMenuIndex + 1) % subMenuCount]);
}

void drawMainMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menu:");
  lcd.setCursor(0, 1);
  lcd.print("> ");
  lcd.print(mainMenuItems[mainMenuIndex]);
}
void drawAbout() {
  lcd.clear();
  lcdCenterPrint(0, "RoboBox RC");
  lcdCenterPrint(1, "Controller");
  delay(2000);
  drawMainMenu();
  ui = MAIN_MENU;
}


void drawScanning() {
  lcd.clear();
  lcdCenterPrint(0, "Scanning BT...");
  static uint8_t dots = 0;
  String d = "";
  for (uint8_t i = 0; i < dots; i++) d += ".";
  lcdCenterPrint(1, d.c_str());
  dots = (dots + 1) % 4;
}

void drawDeviceList() {
  lcd.clear();
  if (deviceCount == 0) {
    lcdCenterPrint(0, "No devices");
    lcdCenterPrint(1, "found");
    return;
  }

  // Show either "<- Back" or device name
  // Row 0:
  lcd.setCursor(0, 0);
  if (listIndex == 0) lcd.print("> ");
  else lcd.print("  ");
  lcd.print("<- Back");

  // Row 1:
  lcd.setCursor(0, 1);
  if (deviceCount > 0) {
    int devIdxShown = max(0, listIndex - 1);
    // If cursor is at 0 (Back), show first device on row 1
    if (listIndex == 0) devIdxShown = 0;
    else devIdxShown = listIndex - 1;

    if (devIdxShown >= 0 && devIdxShown < deviceCount) {
      if (listIndex == devIdxShown + 1) lcd.print("> ");
      else lcd.print("  ");

      if (devices[devIdxShown].hasName) {
        String nm = String(devices[devIdxShown].name);
        if (nm.length() > 14) nm = nm.substring(0, 14);
        lcd.print(nm);
      } else {
        // Show address if no name
        char buf[18];
        sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
                devices[devIdxShown].addr[0], devices[devIdxShown].addr[1], devices[devIdxShown].addr[2],
                devices[devIdxShown].addr[3], devices[devIdxShown].addr[4], devices[devIdxShown].addr[5]);
        lcd.print(buf);
      }
    }
  }
}

void drawConnecting(const char* target) {
  lcd.clear();
  lcdCenterPrint(0, "Connecting to");
  // Row 1 trimmed
  String s = String(target);
  if (s.length() > 16) s = s.substring(0, 16);
  lcd.setCursor(0, 1);
  lcd.print(s);
}

void drawConnected() {
  lcd.clear();
  lcdCenterPrint(0, "Connected!");
  lcd.setCursor(0, 1);
  lcd.print("Send: ON  Back<");
}

void drawError(const char* msg) {
  lcd.clear();
  lcdCenterPrint(0, "Error");
  String s = String(msg);
  if (s.length() > 16) s = s.substring(0, 16);
  lcd.setCursor(0, 1);
  lcd.print(s);
}

// Simple slide animation when going "Back"
void slideBackAnimation(const char* line0, const char* line1) {
  lcd.clear();
  String l0 = String(line0);
  String l1 = String(line1);
  for (int i = 0; i <= 16; i++) {
    lcd.clear();
    lcd.setCursor(max(0, 16 - i), 0);
    lcd.print(l0);
    lcd.setCursor(max(0, 16 - i), 1);
    lcd.print(l1);
    delay(12);
  }
}

// ------------------ Rotary Encoder (polling) ------------------
void pollEncoder() {
  static int lastCLK = HIGH;
  int newCLK = digitalRead(ENC_CLK);
  if (newCLK == LOW && lastCLK == HIGH) {
    if (digitalRead(ENC_DT) == HIGH) encDelta++;
    else encDelta--;
  }
  lastCLK = newCLK;

  // Button
  bool down = (digitalRead(ENC_SW) == LOW);
  if (down && !swWasDown) {
    swDownAt = millis();
  } else if (!down && swWasDown) {
    unsigned long held = millis() - swDownAt;
    if (held >= LONG_PRESS_MS) {
      // long press event
      swPressed = false;  // handled via state
      // We'll handle "back" on long press in each state.
    } else {
      // short press event
      swPressed = true;
    }
  }
  swWasDown = down;
}

// Normalize encoder delta to +/-1 steps
int takeEncStep() {
  int d = 0;
  if (encDelta > 0) {
    d = 1;
    encDelta = 0;
  } else if (encDelta < 0) {
    d = -1;
    encDelta = 0;
  }
  return d;
}

// ------------------ Bluetooth Discovery ------------------

static char* get_name_from_eir(uint8_t* eir, uint8_t* length) {
  if (!eir) return NULL;
  uint8_t* p = eir;
  while (p[0]) {
    uint8_t len = p[0];
    if (p[1] == ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME || p[1] == ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME) {
      *length = len - 1;
      return (char*)&p[2];
    }
    p += (len + 1);
  }
  return NULL;
}

bool addrEquals(const esp_bd_addr_t a, const esp_bd_addr_t b) {
  return memcmp(a, b, sizeof(esp_bd_addr_t)) == 0;
}

void addOrUpdateDevice(const esp_bd_addr_t bda, const char* name, uint8_t name_len) {
  // Check existing
  for (int i = 0; i < deviceCount; i++) {
    if (addrEquals(devices[i].addr, bda)) {
      if (name && name_len > 0) {
        int cpy = min((int)name_len, (int)sizeof(devices[i].name) - 1);
        strncpy(devices[i].name, name, cpy);
        devices[i].name[cpy] = 0;
        devices[i].hasName = true;
      }
      return;
    }
  }
  if (deviceCount >= MAX_BT_DEVICES) return;
  memcpy(devices[deviceCount].addr, bda, sizeof(esp_bd_addr_t));
  if (name && name_len > 0) {
    int cpy = min((int)name_len, (int)sizeof(devices[deviceCount].name) - 1);
    strncpy(devices[deviceCount].name, name, cpy);
    devices[deviceCount].name[cpy] = 0;
    devices[deviceCount].hasName = true;
  } else {
    devices[deviceCount].name[0] = 0;
    devices[deviceCount].hasName = false;
  }
  deviceCount++;
}

void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
      {
        esp_bd_addr_t bda;
        memcpy(bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
        uint8_t* eir = NULL;
        uint8_t len = 0;
        char* nm = NULL;
        for (int i = 0; i < param->disc_res.num_prop; i++) {
          if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
            eir = (uint8_t*)param->disc_res.prop[i].val;
            nm = get_name_from_eir(eir, &len);
          } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
            uint8_t* bdn = (uint8_t*)param->disc_res.prop[i].val;
            len = param->disc_res.prop[i].len;
            nm = (char*)bdn;
          }
        }
        if (nm && len) {
          addOrUpdateDevice(bda, nm, len);
        } else {
          addOrUpdateDevice(bda, NULL, 0);
        }
        break;
      }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
          scanning = false;
        }
        break;
      }
    default:
      break;
  }
}

void startDiscovery() {
  deviceCount = 0;
  scanning = true;
  // GIAC: General Inquiry Access Code, Limited time
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 8, 0);  // ~10 seconds scan
}

void stopDiscovery() {
  if (scanning) {
    esp_bt_gap_cancel_discovery();
    scanning = false;
  }
}

// ------------------ Connection ------------------

bool connectToDeviceIndex(int idx) {
  if (idx < 0 || idx >= deviceCount) return false;
  stopDiscovery();

  connectInProgress = true;

  drawConnecting(devices[idx].hasName ? devices[idx].name : "Unknown");
  // Attempt connect by address
  bool ok = SerialBT.connect(devices[idx].addr);
  connectInProgress = false;
  isConnected = ok;
  return ok;
}

// ------------------ Setup ------------------
void setup() {
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  analogReadResolution(12);  // ESP32 ADC 0..4095

  Serial.begin(115200);

  lcd.begin();
  lcd.backlight();

  drawBootLogo();

  // Init Classic BT SPP (Controller role, we act as client when connecting)
  if (!SerialBT.begin("ESP32_RC_Controller", true)) {  // true = enable Master mode
    drawError("BT init fail");
    ui = ERROR_SCREEN;
    delay(1500);
  } else {
    // Register GAP callback to receive discovery events
    if (esp_bt_gap_register_callback(bt_gap_cb) != ESP_OK) {
      drawError("GAP cb fail");
      ui = ERROR_SCREEN;
      delay(1500);
    } else {
      // Make device connectable/discoverable off by default (we’re a controller)
      esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
      ui = MAIN_MENU;
      drawMainMenu();
    }
  }
}

// ------------------ Loop ------------------
void loop() {
  pollEncoder();

  // Handle UI navigation per state
  int step = takeEncStep();
  bool shortPress = false;
  bool longPress = false;

  // Button press classification
  if (!swWasDown && swPressed) {  // completed short press event latched
    shortPress = true;
    swPressed = false;
  }
  // Long press detection (held currently)
  if (swWasDown && (millis() - swDownAt) >= LONG_PRESS_MS) {
    longPress = true;
  }

  // Periodic redraws
  if (millis() - lastUiUpdate >= UI_REDRAW_MS) {
    lastUiUpdate = millis();
    switch (ui) {
      case BOOT_LOGO: drawBootLogo(); break;
      case MAIN_MENU: /* no periodic animation */ break;
      case BT_SCANNING: drawScanning(); break;
      case BT_LIST: /* static */ break;
      case BT_CONNECTING: /* static */ break;
      case CONNECTED: /* static */ break;
      case ERROR_SCREEN: /* static */ break;
    }
  }

  switch (ui) {
    case MAIN_MENU:
      {
        if (step != 0) {
          mainMenuIndex = (mainMenuIndex + step + mainMenuCount) % mainMenuCount;
          drawMainMenu();
        }
        if (shortPress) {
          if (mainMenuIndex == 0 || mainMenuIndex == 1) {
            // Bluetooth Car or Gesture Car → go to sub menu
            ui = SUB_MENU;
            subMenuIndex = 0;
            drawSubMenu();
          } else if (mainMenuIndex == 2) {
            ui = ABOUT_SCREEN;
            drawAbout();
          }
        }
        break;
      }
    case SUB_MENU:
      {
        if (step != 0) {
          subMenuIndex = (subMenuIndex + step + subMenuCount) % subMenuCount;
          drawSubMenu();
        }
        if (shortPress) {
          if (subMenuIndex == 0) {  // ✅ "Connect"
            slideBackAnimation("Going to", "Connect...");
            ui = BT_SCANNING;
            drawScanning();

            // Store which mode we’re in (Car or Gesture)
            if (mainMenuIndex == 0) currentMode = 0;  // 0 = Joystick Car
            else if (mainMenuIndex == 1) {
              currentMode = 1;  // 1 = Gesture Car
              setupMPU();
            }

            startDiscovery();
          } else {  // "<- Back"
            slideBackAnimation("Back to", "Menu");
            ui = MAIN_MENU;
            drawMainMenu();
          }
        }
        if (longPress) {
          slideBackAnimation("Back to", "Menu");
          ui = MAIN_MENU;
          drawMainMenu();
        }
        break;
      }
    case BT_SCANNING:
      {
        // Allow cancel with long press -> back to main
        if (longPress) {
          stopDiscovery();
          slideBackAnimation("Back to", "Menu");
          ui = MAIN_MENU;
          drawMainMenu();
        }
        // When scan stops (callback sets scanning=false), move to list
        if (!scanning) {
          ui = BT_LIST;
          listIndex = 0;  // 0 is "<- Back", devices are 1..deviceCount
          drawDeviceList();
        }
        break;
      }

    case BT_LIST:
      {
        // Encoder cycles 0..deviceCount (0 = Back)
        if (step != 0) {
          listIndex += step;
          if (listIndex < 0) listIndex = deviceCount;  // wrap from back to last device
          if (listIndex > deviceCount) listIndex = 0;
          drawDeviceList();
        }
        if (shortPress) {
          if (listIndex == 0) {
            slideBackAnimation("Back to", "Menu");
            ui = MAIN_MENU;
            drawMainMenu();
          } else {
            // Connect to selected device (index-1 in array)
            int di = listIndex - 1;
            ui = BT_CONNECTING;
            drawConnecting(devices[di].hasName ? devices[di].name : "Unknown");
            bool ok = connectToDeviceIndex(di);
            if (ok) {
              ui = CONNECTED;
              drawConnected();
            } else {
              ui = ERROR_SCREEN;
              drawError("Conn failed");
              delay(1200);
              ui = BT_LIST;
              drawDeviceList();
            }
          }
        }
        if (longPress) {
          slideBackAnimation("Back to", "Menu");
          ui = MAIN_MENU;
          drawMainMenu();
        }
        break;
      }

    case BT_CONNECTING:
      {
        // Just wait (connectToDeviceIndex runs sync)
        break;
      }

    case CONNECTED:
      {
        // Long press to disconnect/back
        if (longPress) {
          SerialBT.disconnect();
          isConnected = false;
          slideBackAnimation("Disconnected", "Back");
          ui = MAIN_MENU;
          drawMainMenu();
        }

        // Send joystick command periodically
        if (millis() - lastSend >= SEND_INTERVAL_MS) {
          lastSend = millis();
          char command = 'S';

          // ---------------------------------------
          // MODE 0: JOYSTICK CONTROL MODE
          // ---------------------------------------
          if (currentMode == 0) {

            // Read joysticks
            int xVal = analogRead(JOY_X1);
            int yVal = analogRead(JOY_Y2);

            // Apply calibrated deadzone
            bool forward = (yVal < centerY - deadZoneY);
            bool backward = (yVal > centerY + deadZoneY);
            bool right = (xVal > centerX + deadZoneX);
            bool left = (xVal < centerX - deadZoneX);

            // Decide movement
            if (forward && !left && !right) command = 'F';
            else if (backward && !left && !right) command = 'B';
            else if (left && !forward && !backward) command = 'L';
            else if (right && !forward && !backward) command = 'R';
            else if (forward && left) command = 'G';
            else if (forward && right) command = 'I';
            else if (backward && left) command = 'H';
            else if (backward && right) command = 'J';
            else command = 'S';  // Stop if no input
          }
          // ---------------------------------------
          // MODE 1: GESTURE CONTROL MODE (MPU6050)
          // ---------------------------------------
          else if (currentMode == 1) {
            readAccel();  // Read current MPU6050 gyro values

            // Calculate relative angle difference from base
            float diffX = angleX - baseX;
            float diffY = angleY - baseY;

            // Adjust threshold sensitivity here
            if (diffY > 15) command = 'L';        // Tilt forward
            else if (diffY < -15) command = 'R';  // Tilt backward
            else if (diffX > 15) command = 'F';   // Tilt right
            else if (diffX < -15) command = 'B';  // Tilt left
            else command = 'S';

            delay(30);  // Smooth response
          }


          // Send command
          if (SerialBT.hasClient()) {
            SerialBT.write(command);  // 🚀 only send single char
          } else {
            // Remote may have dropped; go back
            isConnected = false;
            ui = ERROR_SCREEN;
            drawError("Link lost");
            delay(1000);
            ui = MAIN_MENU;
            drawMainMenu();
          }
        }
        break;
      }
    case ERROR_SCREEN:
      {
        // Long press back to menu
        if (longPress || shortPress) {
          slideBackAnimation("Back to", "Menu");
          ui = MAIN_MENU;
          drawMainMenu();
        }
        break;
      }

    default: break;
  }
}

//++ Declare variable for the calculation of the MPU data
int16_t acc_x, acc_y, acc_z;

//++ Function to Initialize the MPU6050 sensor
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  calibrateNeutral();
}

void readAccel() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start with accelerometer registers
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  // Convert to tilt angles (approx.)
  angleX = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / PI;
  angleY = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / PI;
}

void calibrateNeutral() {
  float sumX = 0, sumY = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    readAccel();
    sumX += angleX;
    sumY += angleY;
    delay(10);
  }
  baseX = sumX / samples;
  baseY = sumY / samples;
  calibrated = true;
}
