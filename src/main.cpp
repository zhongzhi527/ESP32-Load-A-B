#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DualScaleBleProtocol.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <math.h>

#if !defined(CONFIG_BT_ENABLED)
#error Bluetooth is not enabled for this ESP32 build.
#endif

namespace {

constexpr int I2C_SDA_PIN = 23;
constexpr int I2C_SCL_PIN = 22;
constexpr int GREEN_LED_PIN = 16;
constexpr int RED_LED_PIN = 33;
constexpr int PHYSICAL_BUTTON_PIN = 4;
constexpr bool STATUS_LEDS_ACTIVE_HIGH = true;
constexpr uint32_t LED_SLOW_BLINK_MS = 500;
constexpr uint32_t LED_FAST_BLINK_MS = 250;
constexpr uint32_t LED_PEER_PATTERN_MS = 1800;
constexpr uint32_t LED_PEER_FLASH_MS = 120;
constexpr uint32_t LED_PEER_SECOND_FLASH_MS = 300;
constexpr uint32_t PHYSICAL_BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t PHYSICAL_BUTTON_LONG_PRESS_MS = 800;
constexpr uint32_t UNLOCK_NOTICE_MS = 700;
constexpr int32_t STANDALONE_LOCK_THRESHOLD_GRAMS = 3000;
constexpr uint32_t STANDALONE_LOCK_SETTLE_MS = 2500;
constexpr uint8_t STANDALONE_LOCK_MIN_SAMPLES = 5;
constexpr uint8_t STANDALONE_LOCK_SAMPLE_CAPACITY = 8;
// Maximum peak-to-peak spread (1 kg) allowed while settling before the
// standalone weight-lock will commit a reading.
constexpr int32_t STANDALONE_LOCK_NOISE_WINDOW_GRAMS = 1000;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;
constexpr uint8_t OLED_ADDRESSES[] = {0x3C, 0x3D};
constexpr uint32_t OLED_STARTUP_TIMEOUT_MS = 2500;
constexpr uint32_t OLED_RETRY_INTERVAL_MS = 100;
constexpr uint32_t OLED_READY_DELAY_MS = 50;

constexpr int WEIGHT_SAMPLES = 12;
constexpr int TARE_SAMPLES = 64;
constexpr int CALIBRATION_SAMPLES = 64;
constexpr uint8_t GUIDED_CALIBRATION_SAMPLES = 160;
constexpr uint32_t RAW_READ_TIMEOUT_MS = 2000;
constexpr uint32_t OPERATION_SETTLE_MS = 1000;
constexpr uint32_t OPERATION_READ_TIMEOUT_MS = 3000;
constexpr uint32_t GUIDED_CALIBRATION_PREP_MS = 5000;
constexpr uint32_t GUIDED_CALIBRATION_READ_TIMEOUT_MS = 3500;
constexpr uint32_t GUIDED_CALIBRATION_RESULT_MS = 2000;
constexpr float GUIDED_CALIBRATION_WEIGHT_KG = 1.0f;
constexpr int32_t MIN_CALIBRATION_DELTA = 100;
constexpr uint32_t SHUTDOWN_CONFIRM_TIMEOUT_MS = 10000;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint8_t BLE_COMMAND_QUEUE_DEPTH = 8;

#ifndef LOAD_CELL_UNIT
#define LOAD_CELL_UNIT 1
#endif

#if LOAD_CELL_UNIT == 1
constexpr char BLE_DEVICE_NAME[] = "LOAD_CELL_1";
constexpr char LOAD_CELL_UNIT_LABEL[] = "SCALE 1";
#elif LOAD_CELL_UNIT == 2
constexpr char BLE_DEVICE_NAME[] = "LOAD_CELL_2";
constexpr char LOAD_CELL_UNIT_LABEL[] = "SCALE 2";
#else
#error LOAD_CELL_UNIT must be 1 or 2.
#endif

constexpr char PREFERENCES_NAMESPACE[] = "loadcell";
constexpr char CALIBRATION_KEY[] = "cal_factor";
constexpr char ZERO_OFFSET_KEY[] = "zero_offset";
constexpr char CALIBRATION_CHECKSUM_KEY[] = "cal_check";
constexpr uint32_t CALIBRATION_CHECKSUM_MAGIC = 0x4C43414CUL;

NAU7802 scale;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
Preferences preferences;
NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *telemetryCharacteristic = nullptr;
QueueHandle_t bleCommandQueue = nullptr;
QueueHandle_t bleRejectedCommandQueue = nullptr;

struct QueuedBleCommand {
  DualScaleBle::CommandPacket packet{};
  uint32_t connectionSession = 0;
  uint16_t connectionHandle = UINT16_MAX;
};

enum class PhysicalButtonAction : uint8_t {
  None,
  RequestUnlock,
  GuidedCalibration,
};

enum class WeightLockOwner : uint8_t {
  None,
  Standalone,
  Touchscreen,
};

float calibrationFactor = 1.0f;
float latestWeightKg = 0.0f;

bool oledReady = false;
bool scaleReady = false;
bool bleStarted = false;
bool bleClientConnected = false;
bool preferencesReady = false;
bool calibrationAvailable = false;
bool zeroOffsetAvailable = false;
bool latestWeightValid = false;
bool loadCellResponding = false;
bool shutdownRequestArmed = false;
bool telemetryDirty = true;
bool peerStatusKnown = false;
bool peerScaleConnected = false;
bool physicalButtonLastRawState = HIGH;
bool physicalButtonStableState = HIGH;
bool physicalButtonReleasedSinceBoot = false;
bool physicalButtonPressArmed = false;
bool physicalButtonOperationActive = false;
bool weightDisplayLocked = false;
bool unlockRequestPending = false;

uint32_t shutdownRequestArmedAtMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t telemetrySequence = 0;
uint32_t physicalButtonChangedAtMs = 0;
uint32_t physicalButtonPressedAtMs = 0;
uint32_t physicalButtonPressCount = 0;
uint8_t unlockRequestGeneration = 0;
float lockedDisplayWeightKg = 0.0f;
int32_t standaloneLockSamples[STANDALONE_LOCK_SAMPLE_CAPACITY] = {};
uint8_t standaloneLockSampleCount = 0;
uint32_t standaloneLockSettlingStartedMs = 0;
WeightLockOwner weightLockOwner = WeightLockOwner::None;
PhysicalButtonAction pendingPhysicalButtonAction =
    PhysicalButtonAction::None;
uint16_t acknowledgedCommandToken = 0;
uint32_t acknowledgedCommandSession = 0;
DualScaleBle::CommandResult acknowledgedCommandResult =
    DualScaleBle::CommandResult::None;
DualScaleBle::Status protocolStatus = DualScaleBle::Status::Starting;

const char *currentStatus = "STARTING";
String serialInput;

volatile bool gattClientConnected = false;
volatile bool telemetryNotificationsEnabled = false;
volatile bool telemetrySnapshotRequested = false;
volatile uint32_t activeBleConnectionSession = 0;
volatile uint16_t activeBleConnectionHandle = UINT16_MAX;

void serviceBleCommands();

void beginPhysicalButton() {
  pinMode(PHYSICAL_BUTTON_PIN, INPUT_PULLUP);
  physicalButtonLastRawState = digitalRead(PHYSICAL_BUTTON_PIN);
  physicalButtonStableState = physicalButtonLastRawState;
  physicalButtonChangedAtMs = millis();
  physicalButtonReleasedSinceBoot = physicalButtonStableState == HIGH;
  physicalButtonPressArmed = false;

  Serial.print("Physical button GPIO");
  Serial.print(PHYSICAL_BUTTON_PIN);
  Serial.print(" initial state: ");
  Serial.println(physicalButtonStableState == LOW ? "PRESSED/LOW"
                                                   : "RELEASED/HIGH");
}

void servicePhysicalButton() {
  const bool rawState = digitalRead(PHYSICAL_BUTTON_PIN);
  const uint32_t now = millis();

  if (rawState != physicalButtonLastRawState) {
    physicalButtonLastRawState = rawState;
    physicalButtonChangedAtMs = now;
  }

  if (now - physicalButtonChangedAtMs < PHYSICAL_BUTTON_DEBOUNCE_MS ||
      rawState == physicalButtonStableState) {
    return;
  }

  physicalButtonStableState = rawState;
  if (physicalButtonStableState == LOW) {
    ++physicalButtonPressCount;
    physicalButtonPressedAtMs = now;
    physicalButtonPressArmed =
        physicalButtonReleasedSinceBoot && !physicalButtonOperationActive;
    Serial.printf("Physical button PRESSED on GPIO%d (count=%lu).\n",
                  PHYSICAL_BUTTON_PIN,
                  static_cast<unsigned long>(physicalButtonPressCount));
  } else {
    const uint32_t heldMs = now - physicalButtonPressedAtMs;
    Serial.printf("Physical button RELEASED on GPIO%d after %lu ms.\n",
                  PHYSICAL_BUTTON_PIN,
                  static_cast<unsigned long>(heldMs));

    if (physicalButtonPressArmed && !physicalButtonOperationActive) {
      const PhysicalButtonAction action =
          heldMs >= PHYSICAL_BUTTON_LONG_PRESS_MS
              ? PhysicalButtonAction::GuidedCalibration
              : PhysicalButtonAction::RequestUnlock;
      if (pendingPhysicalButtonAction == PhysicalButtonAction::None) {
        pendingPhysicalButtonAction = action;
        Serial.println(action == PhysicalButtonAction::GuidedCalibration
                           ? "Queued guided 1 kg calibration."
                           : "Queued weight unlock/re-arm request.");
      } else {
        Serial.println("Button action ignored: another action is pending.");
      }
    } else if (physicalButtonOperationActive) {
      Serial.println("Button release ignored: scale operation is active.");
    }

    physicalButtonPressArmed = false;
    physicalButtonReleasedSinceBoot = true;
  }
}

void writeStatusLed(int pin, bool on) {
  const uint8_t activeLevel = STATUS_LEDS_ACTIVE_HIGH ? HIGH : LOW;
  const uint8_t inactiveLevel = STATUS_LEDS_ACTIVE_HIGH ? LOW : HIGH;
  digitalWrite(pin, on ? activeLevel : inactiveLevel);
}

void updateStatusLeds() {
  const bool slowPhase = ((millis() / LED_SLOW_BLINK_MS) & 1U) == 0;
  const bool fastPhase = ((millis() / LED_FAST_BLINK_MS) & 1U) == 0;
  const uint32_t peerPatternPhase = millis() % LED_PEER_PATTERN_MS;
  const bool peerMissingPhase =
      peerPatternPhase < LED_PEER_FLASH_MS ||
      (peerPatternPhase >= LED_PEER_SECOND_FLASH_MS &&
       peerPatternPhase < LED_PEER_SECOND_FLASH_MS + LED_PEER_FLASH_MS);
  bool greenOn = false;
  bool redOn = false;

  if (strcmp(currentStatus, "READY") == 0) {
    // Connectivity patterns apply only after the local load cell is ready.
    // Solid green: touchscreen and the other scale are both connected.
    // Double green flash: touchscreen connected, other scale absent/unknown.
    // Slow green blink: this scale is waiting for the touchscreen.
    if (!bleClientConnected) {
      greenOn = slowPhase;
    } else if (peerStatusKnown && peerScaleConnected) {
      greenOn = true;
    } else {
      greenOn = peerMissingPhase;
    }
  } else if (strcmp(currentStatus, "CALIBRATING") == 0) {
    // Guided/local calibration uses a red-only flashing pattern.
    redOn = fastPhase;
  } else if (strcmp(currentStatus, "TARING") == 0) {
    greenOn = fastPhase;
    redOn = !fastPhase;
  } else if (strcmp(currentStatus, "NEEDS_TARE") == 0 ||
             strcmp(currentStatus, "NEEDS_CALIBRATION") == 0) {
    redOn = slowPhase;
  } else if (strcmp(currentStatus, "NO_SENSOR") == 0 ||
             strcmp(currentStatus, "SENSOR_ERROR") == 0) {
    redOn = true;
  } else if (strcmp(currentStatus, "SHUTTING_DOWN") != 0) {
    // STARTING and any unknown non-fault state blink both LEDs together.
    greenOn = slowPhase;
    redOn = slowPhase;
  }

  writeStatusLed(GREEN_LED_PIN, greenOn);
  writeStatusLed(RED_LED_PIN, redOn);
}

void beginStatusLeds() {
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  writeStatusLed(GREEN_LED_PIN, false);
  writeStatusLed(RED_LED_PIN, false);
  updateStatusLeds();
}

void statusLedDelay(uint32_t durationMs) {
  const uint32_t startedMs = millis();
  while (millis() - startedMs < durationMs) {
    servicePhysicalButton();
    updateStatusLeds();
    delay(10);
  }
}

bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void drawHeader(const String &title) {
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(title);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawConnectionFooter() {
  if (!oledReady) {
    return;
  }

  display.fillRect(0, 52, SCREEN_WIDTH, 12, SSD1306_BLACK);
  display.drawFastHLine(0, 52, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(bleClientConnected ? "BLE:ON" : "BLE:OFF");
  display.print("  ");
  display.print(loadCellResponding ? "CELL:OK" : "CELL:ERR");
}

void refreshConnectionFooter() {
  if (!oledReady) {
    return;
  }

  drawConnectionFooter();
  display.display();
}

void setLoadCellResponding(bool responding) {
  if (loadCellResponding == responding) {
    return;
  }

  loadCellResponding = responding;
  telemetryDirty = true;
  refreshConnectionFooter();
}

void showStatus(const String &title,
                const String &line1 = "",
                const String &line2 = "",
                const String &line3 = "",
                const String &line4 = "") {
  if (!oledReady) {
    return;
  }

  display.clearDisplay();
  drawHeader(title);
  display.setCursor(0, 13);
  display.println(line1);
  display.setCursor(0, 23);
  display.println(line2);
  display.setCursor(0, 33);
  display.println(line3);
  display.setCursor(0, 43);
  display.println(line4);
  drawConnectionFooter();
  display.display();
}

void showWeight(float weightKg) {
  if (!oledReady) {
    return;
  }

  const float displayedWeightKg =
      weightDisplayLocked ? lockedDisplayWeightKg : weightKg;
  const bool standaloneLockSettling =
      !bleClientConnected && !weightDisplayLocked &&
      standaloneLockSampleCount > 0;

  display.clearDisplay();
  drawHeader(LOAD_CELL_UNIT_LABEL);
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.println(weightDisplayLocked
                      ? "WEIGHT LOCKED"
                      : standaloneLockSettling ? "LOCKING..." : "WEIGHT");

  const String weightLine = String(displayedWeightKg, 2) + " KG";
  display.setTextSize(2);
  int16_t textX = 0;
  int16_t textY = 0;
  uint16_t textWidth = 0;
  uint16_t textHeight = 0;
  display.getTextBounds(weightLine,
                        0,
                        0,
                        &textX,
                        &textY,
                        &textWidth,
                        &textHeight);
  const int16_t centeredX =
      textWidth < SCREEN_WIDTH ? (SCREEN_WIDTH - textWidth) / 2 : 0;
  display.setCursor(centeredX, 29);
  display.print(weightLine);

  drawConnectionFooter();
  display.display();
}

void showCalibrationRequired() {
  showStatus("CALIBRATION",
             "Required",
             "Use touch display",
             "Tare, add weight,",
             "then start CAL");
}

void beginOled() {
  const uint32_t startupStartedMs = millis();
  uint8_t attempt = 0;

  do {
    ++attempt;
    for (uint8_t address : OLED_ADDRESSES) {
      if (!i2cDevicePresent(address)) {
        continue;
      }

      // Some panels acknowledge I2C slightly before their controller is ready
      // to accept the complete SSD1306 initialization command sequence.
      statusLedDelay(OLED_READY_DELAY_MS);
      if (!i2cDevicePresent(address)) {
        continue;
      }

      if (display.begin(SSD1306_SWITCHCAPVCC, address, false, false)) {
        display.ssd1306_command(0xAD);
        display.ssd1306_command(0x30);
        display.ssd1306_command(0x81);
        display.ssd1306_command(0xFF);
        display.ssd1306_command(0xD9);
        display.ssd1306_command(0x22);
        display.ssd1306_command(0xDB);
        display.ssd1306_command(0x30);
        display.ssd1306_command(SSD1306_DISPLAYON);

        oledReady = true;
        Serial.printf(
            "SSD1306 OLED found at I2C address 0x%02X after %u attempt(s).\n",
            address,
            attempt);
        showStatus("STARTING", "OLED ready");
        return;
      }
    }

    statusLedDelay(OLED_RETRY_INTERVAL_MS);
  } while (millis() - startupStartedMs < OLED_STARTUP_TIMEOUT_MS);

  Serial.println("SSD1306 OLED not found at 0x3C or 0x3D after startup retries.");
  Serial.println("Continuing without the local OLED.");
}

bool validCalibrationFactor(float factor) {
  return isfinite(factor) && fabsf(factor) > 0.000001f &&
         fabsf(factor) < 1000000000.0f;
}

uint32_t calibrationChecksum(float factor, int32_t zeroOffset) {
  uint32_t factorBits = 0;
  memcpy(&factorBits, &factor, sizeof(factorBits));
  return CALIBRATION_CHECKSUM_MAGIC ^ factorBits ^
         (static_cast<uint32_t>(zeroOffset) * 2654435761UL);
}

bool loadCalibrationData() {
  if (!preferencesReady || !preferences.isKey(CALIBRATION_KEY)) {
    return false;
  }

  const float storedFactor = preferences.getFloat(CALIBRATION_KEY, NAN);
  if (!validCalibrationFactor(storedFactor)) {
    Serial.println("Stored calibration factor is invalid; calibration required.");
    return false;
  }

  calibrationFactor = storedFactor;
  Serial.print("Loaded calibration factor: ");
  Serial.println(calibrationFactor, 6);

  zeroOffsetAvailable = false;
  if (!preferences.isKey(ZERO_OFFSET_KEY) ||
      !preferences.isKey(CALIBRATION_CHECKSUM_KEY)) {
    Serial.println(
        "Stored factor has no verified zero offset; startup tare required.");
    return true;
  }

  const int32_t storedZeroOffset =
      preferences.getInt(ZERO_OFFSET_KEY, 0);
  const uint32_t storedChecksum =
      preferences.getUInt(CALIBRATION_CHECKSUM_KEY, 0);
  const uint32_t expectedChecksum =
      calibrationChecksum(storedFactor, storedZeroOffset);
  if (storedChecksum != expectedChecksum) {
    Serial.println(
        "Stored zero/factor checksum is invalid; startup tare required.");
    return true;
  }

  scale.setZeroOffset(storedZeroOffset);
  zeroOffsetAvailable = true;
  Serial.print("Loaded verified zero offset: ");
  Serial.println(storedZeroOffset);
  return true;
}

bool saveCalibrationData() {
  if (!preferencesReady || !scaleReady || !zeroOffsetAvailable ||
      !validCalibrationFactor(calibrationFactor)) {
    Serial.println("Warning: calibration storage is unavailable.");
    return false;
  }

  const int32_t zeroOffset = scale.getZeroOffset();
  const uint32_t checksum =
      calibrationChecksum(calibrationFactor, zeroOffset);

  const bool factorSaved =
      preferences.putFloat(CALIBRATION_KEY, calibrationFactor) ==
      sizeof(float);
  const bool zeroSaved =
      preferences.putInt(ZERO_OFFSET_KEY, zeroOffset) == sizeof(int32_t);
  const bool checksumSaved =
      factorSaved && zeroSaved &&
      preferences.putUInt(CALIBRATION_CHECKSUM_KEY, checksum) ==
          sizeof(uint32_t);

  if (factorSaved && zeroSaved && checksumSaved) {
    Serial.println(
        "Calibration factor and zero offset saved to NVS flash.");
    return true;
  }

  Serial.println(
      "Warning: could not save the complete calibration record.");
  return false;
}

DualScaleBle::Status statusFromText(const char *status) {
  if (strcmp(status, "READY") == 0) {
    return DualScaleBle::Status::Ready;
  }
  if (strcmp(status, "NEEDS_CALIBRATION") == 0) {
    return DualScaleBle::Status::NeedsCalibration;
  }
  if (strcmp(status, "NEEDS_TARE") == 0) {
    return DualScaleBle::Status::NeedsTare;
  }
  if (strcmp(status, "TARING") == 0) {
    return DualScaleBle::Status::Taring;
  }
  if (strcmp(status, "CALIBRATING") == 0) {
    return DualScaleBle::Status::Calibrating;
  }
  if (strcmp(status, "NO_SENSOR") == 0) {
    return DualScaleBle::Status::NoSensor;
  }
  if (strcmp(status, "SENSOR_ERROR") == 0) {
    return DualScaleBle::Status::SensorError;
  }
  if (strcmp(status, "SHUTTING_DOWN") == 0) {
    return DualScaleBle::Status::ShuttingDown;
  }
  return DualScaleBle::Status::Starting;
}

uint8_t telemetryFlags() {
  uint8_t flags = 0;
  if (scaleReady) {
    flags |= DualScaleBle::TelemetryFlag::SensorPresent;
  }
  if (loadCellResponding) {
    flags |= DualScaleBle::TelemetryFlag::AdcResponding;
  }
  if (zeroOffsetAvailable) {
    flags |= DualScaleBle::TelemetryFlag::Tared;
  }
  if (calibrationAvailable) {
    flags |= DualScaleBle::TelemetryFlag::Calibrated;
  }
  if (latestWeightValid) {
    flags |= DualScaleBle::TelemetryFlag::WeightValid;
  }
  if (oledReady) {
    flags |= DualScaleBle::TelemetryFlag::OledPresent;
  }
  if (weightDisplayLocked) {
    flags |= DualScaleBle::TelemetryFlag::WeightLocked;
  }
  if (unlockRequestPending) {
    flags |= DualScaleBle::TelemetryFlag::UnlockRequested;
  }
  return flags;
}

int32_t latestWeightGrams() {
  if (!latestWeightValid || !isfinite(latestWeightKg)) {
    return 0;
  }

  const double grams = static_cast<double>(latestWeightKg) * 1000.0;
  if (grams >= static_cast<double>(INT32_MAX)) {
    return INT32_MAX;
  }
  if (grams <= static_cast<double>(INT32_MIN)) {
    return INT32_MIN;
  }
  return static_cast<int32_t>(lround(grams));
}

DualScaleBle::TelemetryPacket makeTelemetryPacket() {
  DualScaleBle::TelemetryPacket packet{};
  packet.magic = DualScaleBle::TELEMETRY_MAGIC;
  packet.version = DualScaleBle::PROTOCOL_VERSION;
  packet.unit = LOAD_CELL_UNIT;
  // This is a measurement generation, not a notification counter. Status
  // and command acknowledgements reuse it so the touchscreen cannot mistake
  // repeated BLE packets for fresh load-cell samples.
  packet.sequence = telemetrySequence;
  packet.weightGrams = latestWeightGrams();
  const uint32_t connectionSession = activeBleConnectionSession;
  const bool acknowledgementBelongsToClient =
      gattClientConnected &&
      acknowledgedCommandSession == connectionSession;
  packet.acknowledgedToken =
      acknowledgementBelongsToClient ? acknowledgedCommandToken : 0;
  packet.status = static_cast<uint8_t>(protocolStatus);
  packet.flags = telemetryFlags();
  packet.commandResult = static_cast<uint8_t>(
      acknowledgementBelongsToClient
          ? acknowledgedCommandResult
          : DualScaleBle::CommandResult::None);
  packet.unlockRequestGeneration = unlockRequestGeneration;
  return packet;
}

void publishTelemetry(bool force = false) {
  if (!bleStarted || telemetryCharacteristic == nullptr) {
    return;
  }

  const uint32_t now = millis();
  if (!force && !telemetryDirty &&
      now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  const DualScaleBle::TelemetryPacket packet = makeTelemetryPacket();
  telemetryCharacteristic->setValue(
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet));

  if (bleClientConnected && telemetryNotificationsEnabled) {
    telemetryCharacteristic->notify(
        reinterpret_cast<const uint8_t *>(&packet),
        sizeof(packet));
  }

  telemetryDirty = false;
  lastTelemetryMs = now;
}

void resetStandaloneWeightLockCandidate(const char *reason = nullptr) {
  const bool wasSettling = standaloneLockSampleCount > 0;
  standaloneLockSampleCount = 0;
  standaloneLockSettlingStartedMs = 0;

  if (wasSettling && reason != nullptr && reason[0] != '\0') {
    Serial.print("Standalone weight-lock timer reset (");
    Serial.print(reason);
    Serial.println(").");
  }
}

void appendStandaloneWeightLockSample(int32_t weightGrams) {
  if (standaloneLockSampleCount < STANDALONE_LOCK_SAMPLE_CAPACITY) {
    standaloneLockSamples[standaloneLockSampleCount++] = weightGrams;
    return;
  }

  for (uint8_t index = 1;
       index < STANDALONE_LOCK_SAMPLE_CAPACITY;
       ++index) {
    standaloneLockSamples[index - 1] = standaloneLockSamples[index];
  }
  standaloneLockSamples[STANDALONE_LOCK_SAMPLE_CAPACITY - 1] =
      weightGrams;
}

int32_t medianStandaloneWeightLockSample() {
  int32_t sorted[STANDALONE_LOCK_SAMPLE_CAPACITY] = {};
  for (uint8_t index = 0; index < standaloneLockSampleCount; ++index) {
    sorted[index] = standaloneLockSamples[index];
  }

  for (uint8_t index = 1; index < standaloneLockSampleCount; ++index) {
    const int32_t selected = sorted[index];
    int8_t insertAt = static_cast<int8_t>(index) - 1;
    while (insertAt >= 0 && sorted[insertAt] > selected) {
      sorted[insertAt + 1] = sorted[insertAt];
      --insertAt;
    }
    sorted[insertAt + 1] = selected;
  }

  return sorted[standaloneLockSampleCount / 2];
}

void beginStandaloneWeightLockSettling(int32_t firstWeightGrams,
                                        const char *reason) {
  resetStandaloneWeightLockCandidate();
  standaloneLockSettlingStartedMs = millis();
  appendStandaloneWeightLockSample(firstWeightGrams);

  Serial.print("Standalone weight-lock timer started");
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println(".");
}

void redrawWeightLockState() {
  if (!oledReady || physicalButtonOperationActive ||
      strcmp(currentStatus, "READY") != 0) {
    return;
  }

  if (latestWeightValid || weightDisplayLocked) {
    showWeight(latestWeightKg);
  } else {
    showStatus(LOAD_CELL_UNIT_LABEL,
               weightDisplayLocked ? "WEIGHT LOCKED" : "UNLOCKED",
               weightDisplayLocked ? "Frozen reading" : "Waiting for weight",
               "Lock threshold >3kg");
  }
}

void commitStandaloneWeightLock(int32_t lockedWeightGrams) {
  lockedDisplayWeightKg =
      static_cast<float>(lockedWeightGrams) / 1000.0f;
  weightDisplayLocked = true;
  weightLockOwner = WeightLockOwner::Standalone;
  resetStandaloneWeightLockCandidate();
  telemetryDirty = true;

  Serial.print("Standalone OLED weight LOCKED at ");
  Serial.print(lockedDisplayWeightKg, 2);
  Serial.println(" kg.");
  redrawWeightLockState();
  publishTelemetry(true);
}

void serviceStandaloneWeightLock() {
  // The GATT callback flag closes the small gap before the main-loop mirror
  // updates. As soon as a touchscreen central connects, it owns locking.
  const bool touchscreenOwnsLocking = gattClientConnected;
  if (touchscreenOwnsLocking || bleClientConnected) {
    resetStandaloneWeightLockCandidate("touchscreen connected");
    return;
  }

  if (weightDisplayLocked || physicalButtonOperationActive ||
      strcmp(currentStatus, "READY") != 0 || !latestWeightValid) {
    resetStandaloneWeightLockCandidate();
    return;
  }

  const int32_t liveWeightGrams = latestWeightGrams();
  if (liveWeightGrams <= STANDALONE_LOCK_THRESHOLD_GRAMS) {
    resetStandaloneWeightLockCandidate("weight at or below 3 kg");
    return;
  }

  if (standaloneLockSampleCount == 0) {
    beginStandaloneWeightLockSettling(liveWeightGrams,
                                      "local weight above 3 kg");
    return;
  }

  int32_t minimumGrams = liveWeightGrams;
  int32_t maximumGrams = liveWeightGrams;
  for (uint8_t index = 0; index < standaloneLockSampleCount; ++index) {
    minimumGrams = min(minimumGrams, standaloneLockSamples[index]);
    maximumGrams = max(maximumGrams, standaloneLockSamples[index]);
  }

  const int64_t candidateRangeGrams =
      static_cast<int64_t>(maximumGrams) - minimumGrams;
  if (candidateRangeGrams > STANDALONE_LOCK_NOISE_WINDOW_GRAMS) {
    beginStandaloneWeightLockSettling(liveWeightGrams,
                                      "noise exceeded 1 kg window");
    return;
  }

  appendStandaloneWeightLockSample(liveWeightGrams);
  if (millis() - standaloneLockSettlingStartedMs <
          STANDALONE_LOCK_SETTLE_MS ||
      standaloneLockSampleCount < STANDALONE_LOCK_MIN_SAMPLES) {
    return;
  }

  commitStandaloneWeightLock(medianStandaloneWeightLockSample());
}

void requestWeightUnlock(const char *reason) {
  weightDisplayLocked = false;
  weightLockOwner = WeightLockOwner::None;
  unlockRequestPending = true;
  ++unlockRequestGeneration;
  resetStandaloneWeightLockCandidate();
  telemetryDirty = true;

  Serial.print("Weight unlock/re-arm requested");
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.printf("; generation=%u.\n", unlockRequestGeneration);

  redrawWeightLockState();
  publishTelemetry(true);
}

bool clearWeightLockRequest(uint8_t observedGeneration) {
  if (observedGeneration != unlockRequestGeneration) {
    Serial.println(
        "Ignored stale weight-unlock acknowledgement from touchscreen.");
    return false;
  }

  const bool stateChanged =
      weightDisplayLocked || unlockRequestPending ||
      weightLockOwner != WeightLockOwner::None;
  weightDisplayLocked = false;
  unlockRequestPending = false;
  weightLockOwner = WeightLockOwner::None;
  resetStandaloneWeightLockCandidate();
  if (stateChanged) {
    telemetryDirty = true;
    redrawWeightLockState();
    publishTelemetry(true);
  }
  return true;
}

bool applyWeightLock(int32_t lockedWeightGrams,
                     uint8_t observedGeneration) {
  if (observedGeneration != unlockRequestGeneration ||
      unlockRequestPending || physicalButtonOperationActive ||
      strcmp(currentStatus, "READY") != 0) {
    Serial.println(
        "Weight-lock command rejected because it is stale or the scale is busy.");
    return false;
  }

  lockedDisplayWeightKg =
      static_cast<float>(lockedWeightGrams) / 1000.0f;
  weightDisplayLocked = true;
  weightLockOwner = WeightLockOwner::Touchscreen;
  resetStandaloneWeightLockCandidate();
  telemetryDirty = true;
  Serial.print("OLED weight locked at ");
  Serial.print(lockedDisplayWeightKg, 2);
  Serial.println(" kg.");
  redrawWeightLockState();
  publishTelemetry(true);
  return true;
}

void setCommandAcknowledgement(
    uint16_t token,
    DualScaleBle::CommandResult result,
    uint32_t connectionSession) {
  if (!gattClientConnected ||
      connectionSession != activeBleConnectionSession) {
    return;
  }

  acknowledgedCommandToken = token;
  acknowledgedCommandSession = connectionSession;
  acknowledgedCommandResult = result;
  telemetryDirty = true;
  publishTelemetry(true);
}

void setSystemStatus(const char *status) {
  currentStatus = status;
  if (strcmp(status, "READY") != 0) {
    resetStandaloneWeightLockCandidate("scale is not ready");
  }
  protocolStatus = statusFromText(status);
  telemetryDirty = true;
  updateStatusLeds();
  Serial.print("Status: ");
  Serial.println(currentStatus);
  publishTelemetry(true);
}

class BleServerCallbacks final : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *,
                 NimBLEConnInfo &connInfo) override {
    uint32_t nextSession = activeBleConnectionSession + 1;
    if (nextSession == 0) {
      nextSession = 1;
    }
    activeBleConnectionSession = nextSession;
    activeBleConnectionHandle = connInfo.getConnHandle();
    telemetryNotificationsEnabled = false;
    gattClientConnected = true;
    telemetrySnapshotRequested = true;
  }

  void onDisconnect(NimBLEServer *,
                    NimBLEConnInfo &connInfo,
                    int) override {
    if (connInfo.getConnHandle() != activeBleConnectionHandle) {
      return;
    }
    gattClientConnected = false;
    telemetryNotificationsEnabled = false;
    activeBleConnectionHandle = UINT16_MAX;
  }
};

class BleTelemetryCallbacks final : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *,
                   NimBLEConnInfo &connInfo,
                   uint16_t subValue) override {
    if (connInfo.getConnHandle() != activeBleConnectionHandle) {
      return;
    }
    telemetryNotificationsEnabled = (subValue & 0x01U) != 0;
    if (telemetryNotificationsEnabled) {
      telemetrySnapshotRequested = true;
    }
  }
};

class BleCommandCallbacks final : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &connInfo) override {
    const NimBLEAttValue value = characteristic->getValue();
    if (bleCommandQueue == nullptr ||
        value.size() != sizeof(DualScaleBle::CommandPacket)) {
      return;
    }

    QueuedBleCommand command{};
    memcpy(&command.packet, value.data(), sizeof(command.packet));
    command.connectionSession = activeBleConnectionSession;
    command.connectionHandle = connInfo.getConnHandle();
    if (command.connectionHandle != activeBleConnectionHandle) {
      return;
    }

    if (xQueueSend(bleCommandQueue, &command, 0) != pdTRUE &&
        bleRejectedCommandQueue != nullptr) {
      // Report overload from the Arduino loop; do not notify from this
      // NimBLE callback.
      xQueueOverwrite(bleRejectedCommandQueue, &command);
    }
  }
};

BleServerCallbacks bleServerCallbacks;
BleTelemetryCallbacks bleTelemetryCallbacks;
BleCommandCallbacks bleCommandCallbacks;

void updateBleConnection() {
  if (!bleStarted) {
    return;
  }

  const bool connectedNow = gattClientConnected;
  if (connectedNow != bleClientConnected) {
    bleClientConnected = connectedNow;
    resetStandaloneWeightLockCandidate(
        bleClientConnected ? "touchscreen connected"
                           : "touchscreen disconnected");
    if (bleClientConnected) {
      Serial.println("Touchscreen connected over BLE.");
      if (weightDisplayLocked &&
          weightLockOwner == WeightLockOwner::Standalone) {
        Serial.println(
            "Standalone OLED lock preserved until touchscreen synchronization.");
      }
    } else {
      Serial.println("Touchscreen BLE connection closed.");
      if (weightDisplayLocked &&
          weightLockOwner == WeightLockOwner::Touchscreen) {
        weightLockOwner = WeightLockOwner::Standalone;
        Serial.println(
            "Last local OLED contribution remains locked in standalone mode.");
      }
      peerStatusKnown = false;
      peerScaleConnected = false;
    }
    telemetryDirty = true;
    refreshConnectionFooter();
  }

  if (telemetrySnapshotRequested) {
    telemetrySnapshotRequested = false;
    telemetryDirty = true;
    publishTelemetry(true);
  }
}

void beginBle() {
  Serial.println("Starting BLE load-cell server...");
  bleCommandQueue =
      xQueueCreate(BLE_COMMAND_QUEUE_DEPTH,
                   sizeof(QueuedBleCommand));
  bleRejectedCommandQueue =
      xQueueCreate(1, sizeof(QueuedBleCommand));
  if (bleCommandQueue == nullptr || bleRejectedCommandQueue == nullptr) {
    Serial.println("ERROR: Could not create BLE command queue.");
    return;
  }

  if (!NimBLEDevice::init(BLE_DEVICE_NAME)) {
    Serial.println("ERROR: BLE controller failed to start.");
    return;
  }

  NimBLEDevice::setPower(3);
  bleServer = NimBLEDevice::createServer();
  if (bleServer == nullptr) {
    Serial.println("ERROR: BLE server could not be created.");
    return;
  }

  bleServer->setCallbacks(&bleServerCallbacks, false);
  bleServer->advertiseOnDisconnect(true);

  NimBLEService *service =
      bleServer->createService(DualScaleBle::SERVICE_UUID);
  if (service == nullptr) {
    Serial.println("ERROR: BLE service could not be created.");
    return;
  }

  telemetryCharacteristic = service->createCharacteristic(
      DualScaleBle::TELEMETRY_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
      sizeof(DualScaleBle::TelemetryPacket));
  NimBLECharacteristic *commandCharacteristic =
      service->createCharacteristic(
          DualScaleBle::COMMAND_UUID,
          NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
          sizeof(DualScaleBle::CommandPacket));

  if (telemetryCharacteristic == nullptr ||
      commandCharacteristic == nullptr) {
    Serial.println("ERROR: BLE characteristics could not be created.");
    return;
  }

  telemetryCharacteristic->setCallbacks(&bleTelemetryCallbacks);
  commandCharacteristic->setCallbacks(&bleCommandCallbacks);
  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->setName(BLE_DEVICE_NAME);
  advertising->addServiceUUID(DualScaleBle::SERVICE_UUID);
  advertising->enableScanResponse(true);
  if (!advertising->start()) {
    Serial.println("ERROR: BLE advertising failed to start.");
    return;
  }

  bleStarted = true;
  telemetryDirty = true;
  publishTelemetry(true);
  Serial.print("BLE name: ");
  Serial.println(BLE_DEVICE_NAME);
}

[[noreturn]] void enterSenderDeepSleep() {
  latestWeightValid = false;
  setSystemStatus("SHUTTING_DOWN");
  showStatus("POWERING OFF",
             "Stopping system...",
             "",
             "Press RST or",
             "cycle power to wake");
  Serial.println("Entering deep sleep. Press RST or cycle power to wake.");

  // Keep BLE alive long enough for the touchscreen to read the final state.
  delay(1000);

  if (oledReady) {
    // Clear the OLED as well as sending DISPLAYOFF. If a particular module
    // ignores DISPLAYOFF, it will still show a blank screen.
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
  if (scaleReady) {
    scale.powerDown();
  }
  Wire.end();
  if (preferencesReady) {
    preferences.end();
  }
  if (bleStarted) {
    NimBLEDevice::deinit(true);
    bleStarted = false;
  }

  Serial.flush();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();

  while (true) {
    delay(1000);
  }
}

void serviceScaleOperationBackground() {
  servicePhysicalButton();
  updateBleConnection();
  if (physicalButtonOperationActive) {
    // While a local button operation owns the ADC, drain BLE commands so the
    // display receives BUSY instead of timing out or filling the queue.
    serviceBleCommands();
  }
  publishTelemetry();
  updateStatusLeds();
}

void localOperationDelay(uint32_t durationMs) {
  const uint32_t startedMs = millis();
  while (millis() - startedMs < durationMs) {
    serviceScaleOperationBackground();
    delay(10);
  }
}

bool readAverageRaw(uint8_t sampleCount,
                    uint32_t timeoutMs,
                    int32_t &averageReading) {
  int64_t sum = 0;
  uint8_t samplesRead = 0;
  const uint32_t start = millis();

  while (samplesRead < sampleCount && millis() - start < timeoutMs) {
    serviceScaleOperationBackground();
    if (scale.available()) {
      sum += scale.getReading();
      ++samplesRead;
    } else {
      delay(1);
    }
  }

  if (samplesRead != sampleCount) {
    setLoadCellResponding(false);
    return false;
  }

  averageReading = static_cast<int32_t>(sum / sampleCount);
  setLoadCellResponding(true);
  return true;
}

bool getStableWeightKg(float &weightKg) {
  int32_t averageReading = 0;
  if (!readAverageRaw(WEIGHT_SAMPLES,
                      RAW_READ_TIMEOUT_MS,
                      averageReading)) {
    return false;
  }

  weightKg = static_cast<float>(averageReading - scale.getZeroOffset()) /
             calibrationFactor;
  return isfinite(weightKg);
}

void reportScaleReadFailure(const String &operation) {
  latestWeightValid = false;
  const bool firstFailure = String(currentStatus) != "SENSOR_ERROR";
  if (firstFailure) {
    setSystemStatus("SENSOR_ERROR");
    Serial.print("ERROR: NAU7802 ");
    Serial.print(operation);
    Serial.println(" timed out.");
  }

  showStatus("LOAD CELL ERROR",
             "No ADC samples",
             "Check sensor and",
             "load-cell wiring",
             "Retrying...");
}

void recoverFromScaleReadFailure() {
  if (String(currentStatus) != "SENSOR_ERROR") {
    return;
  }

  Serial.println("NAU7802 readings recovered.");
  if (!zeroOffsetAvailable) {
    setSystemStatus("NEEDS_TARE");
  } else {
    setSystemStatus(calibrationAvailable ? "READY" : "NEEDS_CALIBRATION");
  }
}

bool performTare() {
  if (!scaleReady) {
    Serial.println("ERROR: Load cell is not available.");
    return false;
  }

  requestWeightUnlock("tare");
  latestWeightValid = false;
  setSystemStatus("TARING");
  showStatus("TARING SCALE",
             "Remove all weight",
             "from the load cell",
             "Please wait...");
  Serial.println("Taring scale. Remove all weight.");

  // Give the platform and strain gauge time to stop moving before capturing
  // the zero point. This affects tare only, not normal live readings.
  statusLedDelay(OPERATION_SETTLE_MS);

  int32_t newZeroOffset = 0;
  if (!readAverageRaw(TARE_SAMPLES,
                      OPERATION_READ_TIMEOUT_MS,
                      newZeroOffset)) {
    reportScaleReadFailure("tare");
    return false;
  }

  scale.setZeroOffset(newZeroOffset);
  zeroOffsetAvailable = true;
  scale.setCalibrationFactor(calibrationFactor);
  Serial.println("Tare complete.");

  if (calibrationAvailable && !saveCalibrationData()) {
    Serial.println(
        "Warning: tare works in RAM but its zero offset was not saved.");
  }

  if (calibrationAvailable) {
    setSystemStatus("READY");
    showStatus("TARE COMPLETE", "Scale is at zero", "Ready");
  } else {
    setSystemStatus("NEEDS_CALIBRATION");
    showCalibrationRequired();
  }
  telemetryDirty = true;
  publishTelemetry(true);
  return true;
}

bool performCalibration(float knownWeightKg) {
  if (!scaleReady) {
    Serial.println("ERROR: Load cell is not available.");
    return false;
  }

  if (!isfinite(knownWeightKg) || knownWeightKg <= 0.0f ||
      knownWeightKg > 100.0f) {
    Serial.println("Invalid calibration weight.");
    return false;
  }

  if (!zeroOffsetAvailable) {
    setSystemStatus("NEEDS_TARE");
    showStatus("TARE REQUIRED",
               "Remove all weight",
               "then press TARE",
               "before calibration");
    Serial.println("Calibration refused: no valid tare is available.");
    return false;
  }

  requestWeightUnlock("calibration");
  const float previousFactor = calibrationFactor;
  latestWeightValid = false;
  setSystemStatus("CALIBRATING");
  showStatus("CALIBRATING",
             "Known weight:",
             String(knownWeightKg, 3) + " kg",
             "Keep weight still",
             "Please wait...");

  Serial.print("Calibrating with ");
  Serial.print(knownWeightKg, 3);
  Serial.println(" kg...");

  // Avoid saving a factor from the short mechanical transient immediately
  // after the user places the known mass or taps the button.
  statusLedDelay(OPERATION_SETTLE_MS);

  int32_t loadedReading = 0;
  if (!readAverageRaw(CALIBRATION_SAMPLES,
                      OPERATION_READ_TIMEOUT_MS,
                      loadedReading)) {
    reportScaleReadFailure("calibration read");
    return false;
  }

  const int64_t rawDelta =
      static_cast<int64_t>(loadedReading) - scale.getZeroOffset();
  if (rawDelta > -MIN_CALIBRATION_DELTA &&
      rawDelta < MIN_CALIBRATION_DELTA) {
    calibrationFactor = previousFactor;
    scale.setCalibrationFactor(calibrationFactor);
    setSystemStatus(calibrationAvailable ? "READY" : "NEEDS_CALIBRATION");
    showStatus("CALIBRATION",
               "Failed",
               "Add the selected",
               "known weight and",
               "try again");
    Serial.println("Calibration failed: raw change was too small.");
    return false;
  }

  const float newFactor = static_cast<float>(rawDelta) / knownWeightKg;
  if (!validCalibrationFactor(newFactor)) {
    calibrationFactor = previousFactor;
    scale.setCalibrationFactor(calibrationFactor);
    setSystemStatus(calibrationAvailable ? "READY" : "NEEDS_CALIBRATION");
    showStatus("CALIBRATION", "Failed", "Try again");
    Serial.println("Calibration failed: invalid factor returned.");
    return false;
  }

  calibrationFactor = newFactor;
  calibrationAvailable = true;
  scale.setCalibrationFactor(calibrationFactor);
  const bool calibrationSaved = saveCalibrationData();

  Serial.print("New calibration factor: ");
  Serial.println(calibrationFactor, 6);
  if (!calibrationSaved) {
    Serial.println("ERROR: Calibration works but was not saved.");
  }

  showStatus("CALIBRATION",
             calibrationSaved ? "Complete" : "Complete (RAM)",
             "Factor:",
             String(calibrationFactor, 3),
             calibrationSaved ? "Saved to flash" : "Not saved");
  setSystemStatus("READY");
  delay(800);
  telemetryDirty = true;
  publishTelemetry(true);
  // Calibration is operational in RAM even when persistence fails. The OLED
  // and Serial log report the storage failure separately.
  return true;
}

void restoreCalibrationRuntime(float previousFactor,
                               int32_t previousZeroOffset,
                               bool previousCalibrationAvailable,
                               bool previousZeroOffsetAvailable) {
  calibrationFactor = previousFactor;
  calibrationAvailable = previousCalibrationAvailable;
  zeroOffsetAvailable = previousZeroOffsetAvailable;
  scale.setCalibrationFactor(calibrationFactor);
  scale.setZeroOffset(previousZeroOffset);
}

void returnToIdleAfterLocalOperation(bool sensorFailure = false) {
  latestWeightValid = false;
  if (sensorFailure) {
    setSystemStatus("SENSOR_ERROR");
    showStatus("LOAD CELL ERROR",
               "No ADC samples",
               "Check sensor and",
               "load-cell wiring",
               "Retrying...");
  } else if (!zeroOffsetAvailable) {
    setSystemStatus("NEEDS_TARE");
    showStatus("TARE REQUIRED",
               "Remove all weight",
               "then use TARE");
  } else if (!calibrationAvailable) {
    setSystemStatus("NEEDS_CALIBRATION");
    showCalibrationRequired();
  } else {
    setSystemStatus("READY");
    showStatus(LOAD_CELL_UNIT_LABEL,
               "Ready",
               "Press: unlock",
               "Hold 0.8s: CAL");
  }
  telemetryDirty = true;
  publishTelemetry(true);
}

bool finishGuidedCalibrationFailure(
    const String &reason,
    bool sensorFailure,
    float previousFactor,
    int32_t previousZeroOffset,
    bool previousCalibrationAvailable,
    bool previousZeroOffsetAvailable) {
  restoreCalibrationRuntime(previousFactor,
                            previousZeroOffset,
                            previousCalibrationAvailable,
                            previousZeroOffsetAvailable);
  latestWeightValid = false;

  Serial.print("Guided calibration failed: ");
  Serial.println(reason);
  showStatus("CAL FAILED!",
             reason,
             "Previous values",
             "kept");
  telemetryDirty = true;
  publishTelemetry(true);
  localOperationDelay(GUIDED_CALIBRATION_RESULT_MS);
  returnToIdleAfterLocalOperation(sensorFailure);
  return false;
}

void showGuidedCalibrationCountdown(const String &stage,
                                    const String &instruction) {
  // Five seconds of preparation are shown as the first five seconds of a
  // seven-second stage. The following raw capture takes roughly two seconds.
  const int preparationSeconds =
      GUIDED_CALIBRATION_PREP_MS / 1000;
  for (int elapsedSeconds = 0;
       elapsedSeconds < preparationSeconds;
       ++elapsedSeconds) {
    const int secondsRemaining =
        preparationSeconds + 2 - elapsedSeconds;
    showStatus("CALIBRATION",
               stage + ": " + String(secondsRemaining) + "s",
               instruction,
               "Keep platform still");
    localOperationDelay(1000);
  }
}

bool performGuidedButtonCalibration() {
  requestWeightUnlock("guided calibration");
  if (!scaleReady) {
    Serial.println("ERROR: Guided calibration needs a working load cell.");
    showStatus("CAL FAILED!", "Load cell missing");
    localOperationDelay(GUIDED_CALIBRATION_RESULT_MS);
    returnToIdleAfterLocalOperation(true);
    return false;
  }

  const float previousFactor = calibrationFactor;
  const int32_t previousZeroOffset = scale.getZeroOffset();
  const bool previousCalibrationAvailable = calibrationAvailable;
  const bool previousZeroOffsetAvailable = zeroOffsetAvailable;

  latestWeightValid = false;
  setSystemStatus("CALIBRATING");
  Serial.println(
      "Guided calibration started: empty scale, then exact 1 kg weight.");

  showGuidedCalibrationCountdown("No load", "Remove all weight");
  showStatus("CALIBRATION",
             "No load: 2s",
             "Measuring zero",
             "DO NOT TOUCH");

  int32_t newZeroOffset = 0;
  if (!readAverageRaw(GUIDED_CALIBRATION_SAMPLES,
                      GUIDED_CALIBRATION_READ_TIMEOUT_MS,
                      newZeroOffset)) {
    return finishGuidedCalibrationFailure(
        "Empty read error",
        true,
        previousFactor,
        previousZeroOffset,
        previousCalibrationAvailable,
        previousZeroOffsetAvailable);
  }

  showGuidedCalibrationCountdown("1kg load", "Place exact 1 kg");
  showStatus("CALIBRATION",
             "1kg load: 2s",
             "Measuring 1 kg",
             "DO NOT TOUCH");

  int32_t loadedReading = 0;
  if (!readAverageRaw(GUIDED_CALIBRATION_SAMPLES,
                      GUIDED_CALIBRATION_READ_TIMEOUT_MS,
                      loadedReading)) {
    return finishGuidedCalibrationFailure(
        "1kg read error",
        true,
        previousFactor,
        previousZeroOffset,
        previousCalibrationAvailable,
        previousZeroOffsetAvailable);
  }

  const int64_t rawDelta =
      static_cast<int64_t>(loadedReading) - newZeroOffset;
  if (rawDelta > -MIN_CALIBRATION_DELTA &&
      rawDelta < MIN_CALIBRATION_DELTA) {
    return finishGuidedCalibrationFailure(
        "1kg signal small",
        false,
        previousFactor,
        previousZeroOffset,
        previousCalibrationAvailable,
        previousZeroOffsetAvailable);
  }

  const float newFactor =
      static_cast<float>(rawDelta) / GUIDED_CALIBRATION_WEIGHT_KG;
  if (!validCalibrationFactor(newFactor)) {
    return finishGuidedCalibrationFailure(
        "Invalid factor",
        false,
        previousFactor,
        previousZeroOffset,
        previousCalibrationAvailable,
        previousZeroOffsetAvailable);
  }

  scale.setZeroOffset(newZeroOffset);
  zeroOffsetAvailable = true;
  calibrationFactor = newFactor;
  calibrationAvailable = true;
  scale.setCalibrationFactor(calibrationFactor);

  if (!saveCalibrationData()) {
    return finishGuidedCalibrationFailure(
        "Save failed",
        false,
        previousFactor,
        previousZeroOffset,
        previousCalibrationAvailable,
        previousZeroOffsetAvailable);
  }

  Serial.print("Guided calibration complete. Zero offset: ");
  Serial.print(newZeroOffset);
  Serial.print(", factor: ");
  Serial.println(calibrationFactor, 6);

  setSystemStatus("READY");
  showStatus("CAL COMPLETE!",
             "1.00 kg reference",
             "Zero + factor",
             "saved to flash");
  telemetryDirty = true;
  publishTelemetry(true);
  localOperationDelay(GUIDED_CALIBRATION_RESULT_MS);
  return true;
}

void performButtonUnlockRequest() {
  requestWeightUnlock("GPIO4 short press");
  showStatus("WEIGHT UNLOCKED",
             "Lock re-armed",
             "Hold >3 kg still",
             "Locks after 2.5s");
  localOperationDelay(UNLOCK_NOTICE_MS);
}

bool servicePendingPhysicalButtonAction() {
  if (pendingPhysicalButtonAction == PhysicalButtonAction::None ||
      physicalButtonOperationActive) {
    return false;
  }

  const PhysicalButtonAction action = pendingPhysicalButtonAction;
  pendingPhysicalButtonAction = PhysicalButtonAction::None;
  physicalButtonOperationActive = true;

  if (action == PhysicalButtonAction::GuidedCalibration) {
    performGuidedButtonCalibration();
  } else {
    performButtonUnlockRequest();
  }

  physicalButtonOperationActive = false;
  physicalButtonPressArmed = false;
  return true;
}

void processCommand(String command, const char *source) {
  command.trim();
  if (command.isEmpty()) {
    return;
  }

  Serial.print("Command from ");
  Serial.print(source);
  Serial.print(": ");
  Serial.println(command);

  String upperCommand = command;
  upperCommand.toUpperCase();

  if (upperCommand == "PING") {
    Serial.println("PONG");
    return;
  }

  if (upperCommand.startsWith("PING,")) {
    String token = command.substring(5);
    token.trim();
    bool validToken = !token.isEmpty() && token.length() <= 10;
    for (size_t i = 0; validToken && i < token.length(); ++i) {
      validToken = isDigit(token.charAt(i));
    }

    if (!validToken) {
      Serial.println("ERROR: Invalid PING token.");
      return;
    }

    Serial.print("PONG,");
    Serial.println(token);
    return;
  }

  if (upperCommand == "STATUS") {
    Serial.print("Current status: ");
    Serial.println(currentStatus);
    telemetryDirty = true;
    publishTelemetry(true);
    return;
  }

  if (upperCommand == "TARE") {
    performTare();
    return;
  }

  if (upperCommand == "SHUTDOWN,REQUEST") {
    shutdownRequestArmed = true;
    shutdownRequestArmedAtMs = millis();
    Serial.println("SHUTDOWN,READY");
    Serial.println("Shutdown request armed; waiting for confirmation.");
    return;
  }

  if (upperCommand == "SHUTDOWN,CONFIRM") {
    if (!shutdownRequestArmed ||
        millis() - shutdownRequestArmedAtMs > SHUTDOWN_CONFIRM_TIMEOUT_MS) {
      shutdownRequestArmed = false;
      Serial.println("ERROR: Shutdown confirmation rejected or expired.");
      return;
    }

    shutdownRequestArmed = false;
    Serial.println("SHUTDOWN,COMMIT");
    enterSenderDeepSleep();
  }

  if (upperCommand.startsWith("CAL,") || upperCommand.startsWith("CAL ")) {
    performCalibration(command.substring(4).toFloat());
    return;
  }

  const char firstCharacter = command.charAt(0);
  if ((firstCharacter >= '0' && firstCharacter <= '9') ||
      firstCharacter == '.') {
    performCalibration(command.toFloat());
    return;
  }

  Serial.println("ERROR: Unknown command.");
  Serial.println("Supported commands: TARE, CAL,<kg>, STATUS, PING[,token], SHUTDOWN,REQUEST/CONFIRM");
}

bool commandBelongsToActiveConnection(const QueuedBleCommand &command) {
  return gattClientConnected &&
         command.connectionSession == activeBleConnectionSession &&
         command.connectionHandle == activeBleConnectionHandle;
}

void executeBleCommand(const QueuedBleCommand &queuedCommand) {
  if (!commandBelongsToActiveConnection(queuedCommand)) {
    return;
  }

  const DualScaleBle::CommandPacket &packet = queuedCommand.packet;
  if (packet.magic != DualScaleBle::COMMAND_MAGIC ||
      packet.version != DualScaleBle::PROTOCOL_VERSION) {
    return;
  }

  if (packet.targetUnit != DualScaleBle::ALL_UNITS &&
      packet.targetUnit != LOAD_CELL_UNIT) {
    return;
  }

  const auto command =
      static_cast<DualScaleBle::CommandType>(packet.command);

  if (physicalButtonOperationActive &&
      (command == DualScaleBle::CommandType::Tare ||
       command == DualScaleBle::CommandType::Calibrate)) {
    setCommandAcknowledgement(
        packet.token,
        DualScaleBle::CommandResult::Busy,
        queuedCommand.connectionSession);
    return;
  }

  switch (command) {
  case DualScaleBle::CommandType::Status:
    setCommandAcknowledgement(
        packet.token,
        DualScaleBle::CommandResult::Success,
        queuedCommand.connectionSession);
    return;

  case DualScaleBle::CommandType::Tare: {
    setCommandAcknowledgement(
        packet.token,
        DualScaleBle::CommandResult::Accepted,
        queuedCommand.connectionSession);
    const bool success = performTare();
    setCommandAcknowledgement(
        packet.token,
        success ? DualScaleBle::CommandResult::Success
                : DualScaleBle::CommandResult::Failed,
        queuedCommand.connectionSession);
    return;
  }

  case DualScaleBle::CommandType::Calibrate: {
    if (packet.valueGrams <= 0 || packet.valueGrams > 100000) {
      setCommandAcknowledgement(
          packet.token,
          DualScaleBle::CommandResult::Invalid,
          queuedCommand.connectionSession);
      return;
    }

    setCommandAcknowledgement(
        packet.token,
        DualScaleBle::CommandResult::Accepted,
        queuedCommand.connectionSession);
    const bool success =
        performCalibration(static_cast<float>(packet.valueGrams) / 1000.0f);
    setCommandAcknowledgement(
        packet.token,
        success ? DualScaleBle::CommandResult::Success
                : DualScaleBle::CommandResult::Failed,
        queuedCommand.connectionSession);
    return;
  }

  case DualScaleBle::CommandType::PeerStatus:
    if (packet.valueGrams != 0 && packet.valueGrams != 1) {
      setCommandAcknowledgement(
          packet.token,
          DualScaleBle::CommandResult::Invalid,
          queuedCommand.connectionSession);
      return;
    }

    peerStatusKnown = true;
    peerScaleConnected = packet.valueGrams == 1;
    Serial.print("Other load-cell ESP32: ");
    Serial.println(peerScaleConnected ? "CONNECTED" : "DISCONNECTED");
    updateStatusLeds();
    setCommandAcknowledgement(
        packet.token,
        DualScaleBle::CommandResult::Success,
        queuedCommand.connectionSession);
    return;

  case DualScaleBle::CommandType::SetWeightLock:
    if (packet.valueGrams < -100000 ||
        packet.valueGrams > 100000) {
      setCommandAcknowledgement(
          packet.token,
          DualScaleBle::CommandResult::Invalid,
          queuedCommand.connectionSession);
      return;
    }

    setCommandAcknowledgement(
        packet.token,
        applyWeightLock(packet.valueGrams,
                        packet.unlockRequestGeneration)
            ? DualScaleBle::CommandResult::Success
            : DualScaleBle::CommandResult::Busy,
        queuedCommand.connectionSession);
    return;

  case DualScaleBle::CommandType::ClearWeightLock:
    setCommandAcknowledgement(
        packet.token,
        clearWeightLockRequest(packet.unlockRequestGeneration)
            ? DualScaleBle::CommandResult::Success
            : DualScaleBle::CommandResult::Busy,
        queuedCommand.connectionSession);
    return;
  }

  setCommandAcknowledgement(
      packet.token,
      DualScaleBle::CommandResult::Invalid,
      queuedCommand.connectionSession);
}

void serviceBleCommands() {
  if (bleCommandQueue == nullptr) {
    return;
  }

  QueuedBleCommand rejectedCommand{};
  if (bleRejectedCommandQueue != nullptr &&
      xQueueReceive(bleRejectedCommandQueue,
                    &rejectedCommand,
                    0) == pdTRUE &&
      commandBelongsToActiveConnection(rejectedCommand)) {
    setCommandAcknowledgement(
        rejectedCommand.packet.token,
        DualScaleBle::CommandResult::Busy,
        rejectedCommand.connectionSession);
  }

  QueuedBleCommand command{};
  while (xQueueReceive(bleCommandQueue, &command, 0) == pdTRUE) {
    if (commandBelongsToActiveConnection(command)) {
      executeBleCommand(command);
      break;
    }
  }
}

void pollCommandStream(Stream &stream, String &buffer, const char *source) {
  while (stream.available()) {
    const char character = static_cast<char>(stream.read());
    if (character == '\n') {
      String completeCommand = buffer;
      buffer = "";
      processCommand(completeCommand, source);
    } else if (character != '\r') {
      if (buffer.length() < 80) {
        buffer += character;
      } else {
        buffer = "";
        Serial.println("ERROR: Command too long.");
      }
    }
  }
}

void pollCommunication() {
  servicePhysicalButton();
  updateBleConnection();
  pollCommandStream(Serial, serialInput, "Serial");
  serviceBleCommands();
  updateBleConnection();
  publishTelemetry();
  updateStatusLeds();
}

void responsiveDelay(uint32_t durationMs) {
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    pollCommunication();
    delay(10);
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  serialInput.reserve(96);
  beginStatusLeds();
  beginPhysicalButton();

  Serial.println();
  Serial.println("ESP32 NAU7802 load cell + BLE display link");
  Serial.print("Load-cell unit: ");
  Serial.println(LOAD_CELL_UNIT_LABEL);
  Serial.println("I2C SDA=GPIO23, SCL=GPIO22");
  Serial.println("LEDs: green=GPIO16, red=GPIO33; button=GPIO4");
  Serial.println("GPIO4 short release: unlock and re-arm weight locking.");
  Serial.println("GPIO4 hold >=800 ms, then release: guided 1 kg calibration.");
  Serial.println("The physical button never resets the ESP32.");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);
  beginOled();
  beginBle();

  preferencesReady = preferences.begin(PREFERENCES_NAMESPACE, false);
  if (!preferencesReady) {
    Serial.println("Warning: could not open calibration storage.");
  }

  showStatus("STARTING", "Finding NAU7802...");
  if (!scale.begin(Wire, true)) {
    Serial.println("NAU7802 not found. Check power and I2C wiring.");
    setSystemStatus("NO_SENSOR");
    showStatus("LOAD CELL ERROR",
               "NAU7802 not found",
               "Check 3.3V, GND,",
               "SDA and SCL");
    while (true) {
      pollCommunication();
      delay(20);
    }
  }

  scaleReady = true;
  Serial.println("NAU7802 found.");
  scale.setGain(NAU7802_GAIN_128);
  scale.setSampleRate(NAU7802_SPS_80);
  scale.setLDO(NAU7802_LDO_3V3);
  delay(100);

  calibrationAvailable = loadCalibrationData();
  scale.setCalibrationFactor(calibrationFactor);
  if (zeroOffsetAvailable) {
    setSystemStatus(calibrationAvailable ? "READY"
                                         : "NEEDS_CALIBRATION");
    showStatus(LOAD_CELL_UNIT_LABEL,
               "Stored calibration",
               "loaded",
               "Press: unlock",
               "Hold 0.8s: CAL");
  } else {
    performTare();
  }

  Serial.println("Serial commands:");
  Serial.println("  TARE");
  Serial.println("  CAL,1.000");
  Serial.println("  STATUS");
  Serial.println("  SHUTDOWN,REQUEST");
  Serial.println("Touch the touchscreen to wake it from idle standby.");
}

void loop() {
  pollCommunication();
  if (servicePendingPhysicalButtonAction()) {
    responsiveDelay(50);
    return;
  }

  float measuredWeightKg = 0.0f;
  if (!getStableWeightKg(measuredWeightKg)) {
    reportScaleReadFailure("weight read");
    responsiveDelay(500);
    return;
  }
  recoverFromScaleReadFailure();
  pollCommunication();

  if (!zeroOffsetAvailable) {
    latestWeightValid = false;
    Serial.println("Reading available, but the scale must be tared.");
    showStatus("TARE REQUIRED",
               "Remove all weight",
               "then press TARE",
               "on the display");
  } else if (calibrationAvailable) {
    latestWeightKg = measuredWeightKg;
    latestWeightValid = true;
    ++telemetrySequence;
    serviceStandaloneWeightLock();

    Serial.print("Weight: ");
    Serial.print(latestWeightKg, 2);
    Serial.println(" kg");
    showWeight(latestWeightKg);
    telemetryDirty = true;
    publishTelemetry(true);
  } else {
    latestWeightValid = false;
    Serial.print("Uncalibrated reading: ");
    Serial.println(measuredWeightKg, 3);
    showCalibrationRequired();
  }

  responsiveDelay(500);
}
