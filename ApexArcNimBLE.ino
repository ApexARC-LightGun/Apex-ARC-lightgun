/*******************************************************************************
 * ApexArc Controller - Main Controller Code
 * 
 * This code implements a BLE-enabled IR light gun controller with multiple
 * firing modes and web-based configuration. It uses an IR camera to track
 * position and emulates a mouse over Bluetooth LE.
 *******************************************************************************/

// Required Libraries
#include <WiFi.h>              // ESP32 WiFi functionality
#include <ESPAsyncWebServer.h> // Asynchronous Web Server
#include <LITTLEFS.h>          // File system for storing settings
#include <ArduinoJson.h>       // JSON parsing and creation
#include <NimBLEDevice.h>      // Bluetooth Low Energy functionality
#include <NimBLEHIDDevice.h>   // BLE HID (Human Interface Device) support
#include <NimBLEServer.h>      // BLE Server functionality
#include <DFRobotIRPosition.h> // IR Camera interface

/************************ PIN DEFINITIONS ************************/
// Hardware interface pins
int TRIGGER_BUTTON_PIN = 12;  // Default pin for main trigger button
int RELOAD_BUTTON_PIN = 14;   // Default pin for reload button
int FEEDBACK_PIN = 27;        // Default pin for haptic/LED feedback

/************************ DEFAULT SETTINGS ************************/
// Default configuration values
#define DEFAULT_BLE_NAME "ApexArc"           // Default Bluetooth device name
#define DEFAULT_WIFI_SSID "ApexArc"          // Default WiFi network name
#define DEFAULT_WIFI_PASSWORD ""              // Default WiFi password (empty)
#define DEFAULT_SCREEN_WIDTH 1920            // Default target screen width
#define DEFAULT_SCREEN_HEIGHT 1080           // Default target screen height
#define DEFAULT_SMOOTHING_FACTOR 5           // Default movement smoothing factor

/************************ GLOBAL VARIABLES ************************/
// Device Configuration Variables
String bleDeviceName = DEFAULT_BLE_NAME;
String wifiSSID = DEFAULT_WIFI_SSID;
String wifiPassword = DEFAULT_WIFI_PASSWORD;
int triggerMode = 1;                   // 1: Single, 2: Burst, 3: Full Auto
int burstClickCount = 3;               // Number of clicks in burst mode
int burstDelay = 100;                 // Delay between burst clicks (ms)
int fullAutoClicksPerSecond = 10;      // Rate of fire in full auto mode
int feedbackDuration = 100;            // Duration of feedback pulse (ms)
int feedbackDelay = 50;               // Delay after feedback (ms)
bool feedbackEnabled = true;           // Enable/disable feedback
int screenWidth = DEFAULT_SCREEN_WIDTH;
int screenHeight = DEFAULT_SCREEN_HEIGHT;
int smoothingFactor = DEFAULT_SMOOTHING_FACTOR;  // Movement smoothing factor

// Calibration System Variables
bool isCalibrated = false;             // Calibration status flag
int calibrationPoints[4][2];           // Screen corner coordinates
int calibrationIRPoints[4][2] = {      // Default IR camera coordinates
    {0, 0}, {1023, 0}, {0, 767}, {1023, 767}
};

// System Components
AsyncWebServer server(80);             // Web server on port 80
DFRobotIRPosition irCamera;            // IR camera object
bool triggerPressed = false;           // Trigger state tracking

// BLE HID Components
NimBLEHIDDevice* hid;                 // BLE HID device object
NimBLECharacteristic* inputMouse;      // Mouse input characteristic
NimBLEServer* pServer;                // BLE server instance
bool connected = false;               // BLE connection status

// Position Tracking Variables
int positionX[4];                     // X coordinates from IR camera
int positionY[4];                     // Y coordinates from IR camera
int previousX;                        // Last known X position
int previousY;                        // Last known Y position

// Input Debouncing Variables
unsigned long lastDebounceTime = 0;    // Last trigger debounce time
unsigned long debounceDelay = 50;      // Debounce delay in ms
bool reloadButtonPressed = false;      // Reload button state
unsigned long lastReloadDebounceTime = 0; // Last reload debounce time

// Logging System Configuration
const char* logFilePath = "/log.txt";  // Log file path
const int maxLogSize = 10000;          // Max log file size in bytes

/************************ HID REPORT DESCRIPTOR ************************/
// Standard HID report descriptor for mouse functionality
static const uint8_t hidReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        // Usage (Pointer)
    0xA1, 0x00,        // Collection (Physical)
    0x85, 0x01,        // Report ID (1)
    0x95, 0x03,        // Report Count (3)
    0x75, 0x01,        // Report Size (1)
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        // Usage Minimum (Button 1)
    0x29, 0x03,        // Usage Maximum (Button 3)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x95, 0x01,        // Report Count (1)
    0x75, 0x05,        // Report Size (5)
    0x81, 0x01,        // Input (Cnst,Arr,Abs)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x02,        // Report Count (2)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7F,        // Logical Maximum (127)
    0x81, 0x06,        // Input (Data,Var,Rel)
    0xC0,              // End Collection
    0xC0               // End Collection
};

/************************ LOGGING SYSTEM ************************/
/**
 * Logs a message to the filesystem
 * Implements log rotation when size limit is reached
 */
void logMessage(const String& message) {
    File logFile = LITTLEFS.open(logFilePath, FILE_APPEND);
    if (logFile) {
        logFile.println(message);
        logFile.close();

        // Check file size and rotate if necessary
        if (LITTLEFS.exists(logFilePath)) {
            File file = LITTLEFS.open(logFilePath, FILE_READ);
            if (file.size() > maxLogSize) {
                file.close();
                LITTLEFS.remove(logFilePath);
                File newFile = LITTLEFS.open(logFilePath, FILE_WRITE);
                newFile.println("Log file truncated due to size limit");
                newFile.close();
            } else {
                file.close();
            }
        }
    }
}

/************************ BLUETOOTH LE CALLBACKS ************************/
/**
 * Handles BLE connection events
 */
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        connected = true;
        logMessage("BLE device connected");
    }

    void onDisconnect(NimBLEServer* pServer) {
        connected = false;
        logMessage("BLE device disconnected");
        NimBLEDevice::startAdvertising();
    }
};

/************************ BLE INITIALIZATION ************************/
/**
 * Initializes the BLE HID device with mouse functionality
 */
void initBLE() {
    NimBLEDevice::init(bleDeviceName.c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    hid = new NimBLEHIDDevice(pServer);
    inputMouse = hid->inputReport(1);
    hid->manufacturer()->setValue("Anthropic");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x01);

    hid->reportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));
    hid->startServices();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    logMessage("BLE HID device initialized");
}

/************************ MOUSE CONTROL FUNCTIONS ************************/
/**
 * Sends a mouse click event over BLE
 */
void mouseClick(uint8_t buttons) {
    if (!connected) return;
    
    uint8_t mouseReport[] = {buttons, 0, 0};
    inputMouse->setValue(mouseReport, sizeof(mouseReport));
    inputMouse->notify();
    
    delay(10);
    
    mouseReport[0] = 0;
    inputMouse->setValue(mouseReport, sizeof(mouseReport));
    inputMouse->notify();
}

/**
 * Sends a mouse movement event over BLE
 */
void mouseMove(int8_t x, int8_t y) {
    if (!connected) return;
    
    uint8_t mouseReport[] = {0, x, y};
    inputMouse->setValue(mouseReport, sizeof(mouseReport));
    inputMouse->notify();
}

/************************ SETTINGS MANAGEMENT ************************/
/**
 * Loads device settings from filesystem
 */
void loadSettings() {
    if (!LITTLEFS.exists("/settings.json")) {
        logMessage("Settings file not found. Using default settings.");
        return;
    }

    File settingsFile = LITTLEFS.open("/settings.json", "r");
    if (!settingsFile) {
        logMessage("Failed to open settings file.");
        return;
    }

    StaticJsonDocument<1024> jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, settingsFile);
    if (error) {
        logMessage("Failed to read settings file, using default settings");
        return;
    }

    // Load all settings from JSON
    bleDeviceName = jsonDoc["bleDeviceName"].as<String>();
    wifiSSID = jsonDoc["wifiSSID"].as<String>();
    wifiPassword = jsonDoc["wifiPassword"].as<String>();
    triggerMode = jsonDoc["triggerMode"] | 1;
    burstClickCount = jsonDoc["burstClickCount"] | 3;
    burstDelay = jsonDoc["burstDelay"] | 100;
    fullAutoClicksPerSecond = jsonDoc["fullAutoClicksPerSecond"] | 10;
    feedbackDuration = jsonDoc["feedbackDuration"] | 100;
    feedbackDelay = jsonDoc["feedbackDelay"] | 50;
    feedbackEnabled = jsonDoc["feedbackEnabled"] | true;
    isCalibrated = jsonDoc["isCalibrated"] | false;
    screenWidth = jsonDoc["screenWidth"] | DEFAULT_SCREEN_WIDTH;
    screenHeight = jsonDoc["screenHeight"] | DEFAULT_SCREEN_HEIGHT;
    TRIGGER_BUTTON_PIN = jsonDoc["triggerPin"] | 12;
    RELOAD_BUTTON_PIN = jsonDoc["reloadPin"] | 14;
    FEEDBACK_PIN = jsonDoc["feedbackPin"] | 27;
    smoothingFactor = jsonDoc["smoothingFactor"] | DEFAULT_SMOOTHING_FACTOR;

    // Load calibration data if available
    if (isCalibrated) {
        for (int i = 0; i < 4; i++) {
            calibrationIRPoints[i][0] = jsonDoc["calibrationIRPoints"][i][0] | calibrationIRPoints[i][0];
            calibrationIRPoints[i][1] = jsonDoc["calibrationIRPoints"][i][1] | calibrationIRPoints[i][1];
            calibrationPoints[i][0] = jsonDoc["calibrationPoints"][i][0] | (i % 2 == 0 ? 0 : screenWidth);
            calibrationPoints[i][1] = jsonDoc["calibrationPoints"][i][1] | (i < 2 ? 0 : screenHeight);
        }
    } else {
        // Set default calibration points
        calibrationPoints[0][0] = 0;
        calibrationPoints[0][1] = 0;
        calibrationPoints[1][0] = screenWidth;
        calibrationPoints[1][1] = 0;
        calibrationPoints[2][0] = 0;
        calibrationPoints[2][1] = screenHeight;
        calibrationPoints[3][0] = screenWidth;
        calibrationPoints[3][1] = screenHeight;
    }

    settingsFile.close();
    logMessage("Settings loaded successfully.");

    previousX = screenWidth / 2;
    previousY = screenHeight / 2;
}

/**
 * Saves current settings to filesystem
 */
bool saveSettings() {
    File settingsFile = LITTLEFS.open("/settings.json", "w");
    if (!settingsFile) {
        logMessage("Failed to open settings file for writing.");
        return false;
    }

    StaticJsonDocument<1024> jsonDoc;
    // Save all current settings to JSON
    jsonDoc["bleDeviceName"] = bleDeviceName;
    jsonDoc["wifiSSID"] = wifiSSID;
    jsonDoc["wifiPassword"] = wifiPassword;
    jsonDoc["triggerMode"] = triggerMode;
    jsonDoc["burstClickCount"] = burstClickCount;
    jsonDoc["burstDelay"] = burstDelay;
    jsonDoc["fullAutoClicksPerSecond"] = fullAutoClicksPerSecond;
    jsonDoc["feedbackDuration"] = feedbackDuration;
    jsonDoc["feedbackDelay"] = feedbackDelay;
    jsonDoc["feedbackEnabled"] = feedbackEnabled;
    jsonDoc["isCalibrated"] = isCalibrated;
    jsonDoc["screenWidth"] = screenWidth;
    jsonDoc["screenHeight"] = screenHeight;
    jsonDoc["triggerPin"] = TRIGGER_BUTTON_PIN;
    jsonDoc["reloadPin"] = RELOAD_BUTTON_PIN;
    jsonDoc["feedbackPin"] = FEEDBACK_PIN;
    jsonDoc["smoothingFactor"] = smoothingFactor;

    // Save calibration data
    JsonArray calibrationIRArray = jsonDoc.createNestedArray("calibrationIRPoints");
    JsonArray calibrationArray = jsonDoc.createNestedArray("calibrationPoints");
    for (int i = 0; i < 4; i++) {
        JsonArray irPoint = calibrationIRArray.createNestedArray();
        irPoint.add(calibrationIRPoints[i][0]);
        irPoint.add(calibrationIRPoints[i][1]);

        JsonArray point = calibrationArray.createNestedArray();
        point.add(calibrationPoints[i][0]);
        point.add(calibrationPoints[i][1]);
    }

    if (serializeJson(jsonDoc, settingsFile) == 0) {
        logMessage("Failed to write settings to file.");
        return false;
    }

    settingsFile.close();
    logMessage("Settings saved successfully.");
    return true;
}

/************************ FEEDBACK SYSTEM ************************/
/**
 * Provides haptic/LED feedback based on current settings
 */
void giveFeedback() {
    if (feedbackEnabled) {
        digitalWrite(FEEDBACK_PIN, HIGH);
        delay(feedbackDuration);
        digitalWrite(FEEDBACK_PIN, LOW);
        delay(feedbackDelay);
    }
}

/************************ FIRING MODES ************************/
// Single shot mode - One click per trigger pull
void singleShot() {
  mouseClick(1); // Left click
  giveFeedback();
}

// Burst fire mode - Multiple clicks per trigger pull based on burstClickCount setting
void burstMode() {
  for (int i = 0; i < burstClickCount; i++) {
    mouseClick(1); // Left click
    giveFeedback();
    delay(burstDelay);
  }
}

// Full auto mode - Continuous firing while trigger held, rate controlled by fullAutoClicksPerSecond
void fullAutoMode() {
  unsigned long interval = 1000 / fullAutoClicksPerSecond;
  unsigned long currentTime = millis();
  static unsigned long lastClickTime = 0;

  if (currentTime - lastClickTime >= interval) {
    mouseClick(1); // Left click
    giveFeedback();
    lastClickTime = currentTime;
  }
}

/************************ INPUT HANDLING SYSTEM ************************/
// Main trigger handler - Processes trigger input with debouncing and manages firing modes
void handleTrigger() {
  static bool lastButtonState = HIGH;
  bool reading = digitalRead(TRIGGER_BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && !triggerPressed) {
      triggerPressed = true;
      switch (triggerMode) {
        case 1:
          singleShot();
          break;
        case 2:
          burstMode();
          break;
        case 3:
          // Full-auto mode handled in continuous loop
          break;
      }
    } else if (reading == HIGH) {
      triggerPressed = false;
    }
  }

  // Continuous firing for full-auto mode
  if (triggerMode == 3 && triggerPressed) {
    fullAutoMode();
  }

  lastButtonState = reading;
}

// Reload button handler - Processes reload button with debouncing
void handleReload() {
  static bool lastReloadButtonState = HIGH;
  bool reading = digitalRead(RELOAD_BUTTON_PIN);

  if (reading != lastReloadButtonState) {
    lastReloadDebounceTime = millis();
  }

  if ((millis() - lastReloadDebounceTime) > debounceDelay) {
    if (reading == LOW && !reloadButtonPressed) {
      reloadButtonPressed = true;
      mouseClick(2); // Right click for reload action
      giveFeedback();
    } else if (reading == HIGH) {
      reloadButtonPressed = false;
    }
  }

  lastReloadButtonState = reading;
}

/************************ POSITION TRACKING SYSTEM ************************/
// Maps IR camera coordinates to screen coordinates using calibration data
void mapIRToScreen(int irX, int irY, int &screenX, int &screenY) {
  float xRatio = (float)(irX - calibrationIRPoints[0][0]) / (calibrationIRPoints[1][0] - calibrationIRPoints[0][0]);
  float yRatio = (float)(irY - calibrationIRPoints[0][1]) / (calibrationIRPoints[2][1] - calibrationIRPoints[0][1]);
  
  screenX = calibrationPoints[0][0] + xRatio * (calibrationPoints[1][0] - calibrationPoints[0][0]);
  screenY = calibrationPoints[2][1] - yRatio * (calibrationPoints[2][1] - calibrationPoints[0][1]);
}

/************************ SYSTEM INITIALIZATION ************************/
// Setup function - Initializes all system components and configurations
void setup() {
  // Initialize filesystem or format if necessary
  if (!LITTLEFS.begin()) {
    if (LITTLEFS.format()) {
      LITTLEFS.begin();
    } else {
      while (1) {
        delay(1000);
      }
    }
  }

  loadSettings();

  // Initialize GPIO pins
  pinMode(TRIGGER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELOAD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FEEDBACK_PIN, OUTPUT);
  digitalWrite(FEEDBACK_PIN, LOW);

  // Initialize core components
  initBLE();
  irCamera.begin();

  // Start WiFi access point
  WiFi.softAP(wifiSSID.c_str(), wifiPassword.c_str());
  logMessage("Access Point started");
  logMessage(WiFi.softAPIP().toString());

  /************************ WEB SERVER ROUTE CONFIGURATION ************************/
  // Home page route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/index.html", "text/html");
  });

  // Logo image route
  server.on("/ApexArc.png", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/ApexArc.png", "image/png");
  });

  // User manual route
  server.on("/userManual", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/userManual.html", "text/html");
  });

  // Get current settings route
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<1024> jsonDoc;
    // Populate JSON with current settings
    jsonDoc["bleDeviceName"] = bleDeviceName;
    jsonDoc["wifiSSID"] = wifiSSID;
    jsonDoc["wifiPassword"] = wifiPassword;
    jsonDoc["triggerMode"] = triggerMode;
    jsonDoc["burstClickCount"] = burstClickCount;
    jsonDoc["burstDelay"] = burstDelay;
    jsonDoc["fullAutoClicksPerSecond"] = fullAutoClicksPerSecond;
    jsonDoc["feedbackDuration"] = feedbackDuration;
    jsonDoc["feedbackDelay"] = feedbackDelay;
    jsonDoc["feedbackEnabled"] = feedbackEnabled;
    jsonDoc["isCalibrated"] = isCalibrated;
    jsonDoc["screenWidth"] = screenWidth;
    jsonDoc["screenHeight"] = screenHeight;
    jsonDoc["triggerPin"] = TRIGGER_BUTTON_PIN;
    jsonDoc["reloadPin"] = RELOAD_BUTTON_PIN;
    jsonDoc["feedbackPin"] = FEEDBACK_PIN;
    jsonDoc["smoothingFactor"] = smoothingFactor;

    String jsonResponse;
    serializeJson(jsonDoc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
  });

  // Update settings route
server.on("/update-settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Update BLE name if provided
    if (request->hasParam("bleDeviceName", true)) {
        bleDeviceName = request->getParam("bleDeviceName", true)->value();
        NimBLEDevice::deinit();
        initBLE();
    }
    // Update WiFi settings if provided
    if (request->hasParam("wifiSSID", true)) {
        wifiSSID = request->getParam("wifiSSID", true)->value();
    }
    if (request->hasParam("wifiPassword", true)) {
        wifiPassword = request->getParam("wifiPassword", true)->value();
    }
    // Update trigger settings if provided
    if (request->hasParam("triggerMode", true)) {
        triggerMode = request->getParam("triggerMode", true)->value().toInt();
    }
    if (request->hasParam("burstClickCount", true)) {
        burstClickCount = request->getParam("burstClickCount", true)->value().toInt();
    }
    if (request->hasParam("burstDelay", true)) {
        burstDelay = request->getParam("burstDelay", true)->value().toInt();
    }
    if (request->hasParam("fullAutoClicksPerSecond", true)) {
        fullAutoClicksPerSecond = request->getParam("fullAutoClicksPerSecond", true)->value().toInt();
    }
    // Update feedback settings if provided
    if (request->hasParam("feedbackDuration", true)) {
        feedbackDuration = request->getParam("feedbackDuration", true)->value().toInt();
    }
    if (request->hasParam("feedbackDelay", true)) {
        feedbackDelay = request->getParam("feedbackDelay", true)->value().toInt();
    }
    if (request->hasParam("feedbackEnabled", true)) {
        feedbackEnabled = (request->getParam("feedbackEnabled", true)->value() == "true");
    }
    // Update screen settings if provided
    if (request->hasParam("screenWidth", true)) {
        screenWidth = request->getParam("screenWidth", true)->value().toInt();
    }
    if (request->hasParam("screenHeight", true)) {
        screenHeight = request->getParam("screenHeight", true)->value().toInt();
    }
    if (request->hasParam("triggerPin", true)) {
        TRIGGER_BUTTON_PIN = request->getParam("triggerPin", true)->value().toInt();
        pinMode(TRIGGER_BUTTON_PIN, INPUT_PULLUP);
    }
    if (request->hasParam("reloadPin", true)) {
        RELOAD_BUTTON_PIN = request->getParam("reloadPin", true)->value().toInt();
        pinMode(RELOAD_BUTTON_PIN, INPUT_PULLUP);
    }
    if (request->hasParam("feedbackPin", true)) {
        FEEDBACK_PIN = request->getParam("feedbackPin", true)->value().toInt();
        pinMode(FEEDBACK_PIN, OUTPUT);
        digitalWrite(FEEDBACK_PIN, LOW);
    }
    if (request->hasParam("smoothingFactor", true)) {
        smoothingFactor = request->getParam("smoothingFactor", true)->value().toInt();
        // Constrain smoothing factor to reasonable values
        smoothingFactor = constrain(smoothingFactor, 1, 20);
    }

    // Handle WiFi reconnection if settings changed
    bool wifiChanged = false;
    if (request->hasParam("wifiSSID", true)) {
        String newSSID = request->getParam("wifiSSID", true)->value();
        if (newSSID != wifiSSID) {
            wifiSSID = newSSID;
            wifiChanged = true;
        }
    }
    if (request->hasParam("wifiPassword", true)) {
        String newPassword = request->getParam("wifiPassword", true)->value();
        if (newPassword != wifiPassword) {
            wifiPassword = newPassword;
            wifiChanged = true;
        }
    }

    if (wifiChanged) {
        WiFi.softAPdisconnect(true);
        WiFi.softAP(wifiSSID.c_str(), wifiPassword.c_str());
    }

    // Initialize default calibration points if not calibrated
    if (!isCalibrated) {
      calibrationPoints[0][0] = 0;
      calibrationPoints[0][1] = 0;
      calibrationPoints[1][0] = screenWidth;
      calibrationPoints[1][1] = 0;
      calibrationPoints[2][0] = 0;
      calibrationPoints[2][1] = screenHeight;
      calibrationPoints[3][0] = screenWidth;
      calibrationPoints[3][1] = screenHeight;
    }

    previousX = screenWidth / 2;
    previousY = screenHeight / 2;

    // Save settings and send response
    if (saveSettings()) {
      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to save settings\"}");
    }
});

  // Calibration route
  server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("point", true) && request->hasParam("x", true) && request->hasParam("y", true)) {
      int point = request->getParam("point", true)->value().toInt();
      int x = request->getParam("x", true)->value().toInt();
      int y = request->getParam("y", true)->value().toInt();

      if (point >= 0 && point < 4) {
        irCamera.requestPosition();
        if (irCamera.available()) {
          calibrationIRPoints[point][0] = irCamera.readX(0);
          calibrationIRPoints[point][1] = irCamera.readY(0);
          calibrationPoints[point][0] = x;
          calibrationPoints[point][1] = y;

          if (point == 3) {
            isCalibrated = true;
            saveSettings();
          }

          request->send(200, "application/json", "{\"status\":\"success\"}");
        } else {
          request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"IR camera not available\"}");
        }
      } else {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid calibration point\"}");
      }
    } else {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing parameters\"}");
    }
  });

  // Log management routes
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LITTLEFS.exists(logFilePath)) {
      request->send(LITTLEFS, logFilePath, "text/plain");
    } else {
      request->send(404, "text/plain", "Log file not found");
    }
  });

  server.on("/clearLogs", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (LITTLEFS.remove(logFilePath)) {
      File newLogFile = LITTLEFS.open(logFilePath, FILE_WRITE);
      if (newLogFile) {
        newLogFile.println("Log file cleared");
        newLogFile.close();
        request->send(200, "text/plain", "Logs cleared successfully");
      } else {
        request->send(500, "text/plain", "Failed to create new log file");
      }
    } else {
      request->send(500, "text/plain", "Failed to clear logs");
    }
  });

  // Feedback test route
  server.on("/testFeedback", HTTP_POST, [](AsyncWebServerRequest *request) {
    giveFeedback();
    logMessage("Feedback test triggered");
    request->send(200, "text/plain", "Feedback test completed");
  });

  server.begin();
  logMessage("Server started");
}

/************************ MAIN CONTROL LOOP ************************/
// Main program loop - Handles continuous operations including input processing and position tracking
void loop() {
  // Process input handlers
  handleTrigger();
  handleReload();

  // Skip processing if not connected
  if (!connected) {
    delay(100);
    return;
  }

  // Request and process IR camera position data
  irCamera.requestPosition();

  if (irCamera.available()) {
    // Read all IR points from camera
    for (int i = 0; i < 4; i++) {
      positionX[i] = irCamera.readX(i);
      positionY[i] = irCamera.readY(i);
    }

    // Calculate average position (centroid)
    int centroidX = (positionX[0] + positionX[1] + positionX[2] + positionX[3]) / 4;
    int centroidY = (positionY[0] + positionY[1] + positionY[2] + positionY[3]) / 4;

    // Map IR coordinates to screen coordinates
    int mappedX, mappedY;
    if (isCalibrated) {
      mapIRToScreen(centroidX, centroidY, mappedX, mappedY);
    } else {
      mappedX = map(centroidX, 0, 1023, screenWidth, 0);
      mappedY = map(centroidY, 0, 767, screenHeight, 0);
    }

    // Apply smoothing to reduce jitter
    int smoothedX = (previousX * (smoothingFactor - 1) + mappedX) / smoothingFactor;
    int smoothedY = (previousY * (smoothingFactor - 1) + mappedY) / smoothingFactor;

    // Calculate movement deltas and constrain to int8_t range (-127 to 127)
    int deltaX = smoothedY - previousY;  // Swapped for correct orientation
    int deltaY = smoothedX - previousX;
    
    deltaX = constrain(deltaX, -127, 127);
    deltaY = constrain(deltaY, -127, 127);
    
    mouseMove(deltaX, deltaY);

    previousX = smoothedX;
    previousY = smoothedY;

    String logMsg = "Centroid: X = " + String(centroidX) + ", Y = " + String(centroidY) +
                    " --> Mapped: X = " + String(mappedX) + ", Y = " + String(mappedY) +
                    " --> Smoothed: X = " + String(smoothedX) + ", Y = " + String(smoothedY);
    logMessage(logMsg);
  } else {
    logMessage("IR camera not detecting any points.");
  }

  delay(10);
};