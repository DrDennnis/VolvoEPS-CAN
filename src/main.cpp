#include <Arduino.h>
#include "driver/twai.h"
#include <AiEsp32RotaryEncoder.h>
#include <Preferences.h>
#include <OneButton.h>

#define CAN_TX_PIN SCL
#define CAN_RX_PIN SDA

#define ROTARY_ENCODER_A_PIN A0
#define ROTARY_ENCODER_B_PIN A1
#define ROTARY_ENCODER_BUTTON_PIN A2
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

#define RELAY_OUTPUT A5

unsigned long previousMillis2000ms = 0;
unsigned long previousMillis72ms = 0;
unsigned long encoderChangedMillis = 0;

const int interval2000ms = 2000;
const int interval72ms = 72;
const int encoderSaveDelayTime = 2000;

byte keepAliveBits[4] = {0x00, 0x40, 0x80, 0xC0};
uint8_t keepAliveCounter = 0;
uint8_t lastEncoderPos = 0;

static bool driver_installed = false;
static bool isEncoderSaved = true;
static bool isPowered = LOW;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
Preferences preferences;

OneButton btn = OneButton(
  ROTARY_ENCODER_BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

void sendCANMessage(uint32_t identifier, uint8_t data[], uint8_t data_length_code = TWAI_FRAME_MAX_DLC) {
  // configure message to transmit
  twai_message_t message = {
    .identifier = identifier,
    .data_length_code = data_length_code,
  };
  message.extd = 1;

  memcpy(message.data, data, data_length_code);

  // Queue message for transmission
  esp_err_t result = twai_transmit(&message, portMAX_DELAY);
  if (result != ESP_OK) {
    printf("\n%s: Failed to queue the message for transmission.\n", esp_err_to_name(result));
  }
}

void IRAM_ATTR readEncoderISR() {
	rotaryEncoder.readEncoder_ISR();
}

static void handleClick() {
  isPowered = !isPowered;
  digitalWrite(RELAY_OUTPUT, isPowered);

  Serial.print("BTN clicked: ");
  Serial.println(isPowered);
}

void setup() {
  Serial.begin(115200);

  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
 
  pinMode(RELAY_OUTPUT, OUTPUT);

  rotaryEncoder.areEncoderPinsPulldownforEsp32 = false;
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 20, false);
  rotaryEncoder.disableAcceleration();

  btn.attachClick(handleClick);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver started");
  } else {
    Serial.println("Failed to start TWAI driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("TWAI Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure TWAI alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;

  // Load settings
  preferences.begin("encoder", false);
  lastEncoderPos = preferences.getUInt("position", 0);
  rotaryEncoder.setEncoderValue(lastEncoderPos);

  Serial.println("Setup complete...");
}

void loop() {
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }

  unsigned long currentMillis = millis();
  btn.tick();

  // Set the data in order to save the encoder position
  if (rotaryEncoder.encoderChanged()) {
    lastEncoderPos = rotaryEncoder.readEncoder();
    encoderChangedMillis = currentMillis;
    isEncoderSaved = false;
    Serial.println(lastEncoderPos);
	}

  // Save encoder position
  if (!isEncoderSaved && (currentMillis >= encoderChangedMillis + encoderSaveDelayTime)) {
    isEncoderSaved = true;
    preferences.putUInt("position", lastEncoderPos);
    Serial.println("new position saved..");
  }

  if (currentMillis - previousMillis2000ms >= interval2000ms) {
    keepAliveCounter = (keepAliveCounter + 1) & 3;

    uint8_t dataKeepAlive[] = {
      keepAliveBits[keepAliveCounter], 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00
    };
    sendCANMessage(0x1ae0092c, dataKeepAlive, sizeof(dataKeepAlive));

    // Custom DD-CAN
    uint8_t ddCan[] = {
      lastEncoderPos
    };
    sendCANMessage(0x190, ddCan, sizeof(ddCan));

    previousMillis2000ms += interval2000ms;
  }

  if (currentMillis - previousMillis72ms >= interval72ms) {
    // 1-6000, 1 = fast, 6000 = slow
    uint16_t speed = map(lastEncoderPos, 0, 20, 6000, 1);
    uint8_t speed0 = (speed >> 8) & 0xFF;
    uint8_t speed1 = speed & 0xFF;

    uint8_t dataSpeed[] = {
      0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, speed0, speed1
    };
    sendCANMessage(0x02104136, dataSpeed, sizeof(dataSpeed));

    previousMillis72ms += interval72ms;
  }
}
