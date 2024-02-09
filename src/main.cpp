#include <Arduino.h>
#include "driver/twai.h"
#include <AiEsp32RotaryEncoder.h>

#define CAN_TX_PIN SCL
#define CAN_RX_PIN SDA

#define ROTARY_ENCODER_A_PIN A1
#define ROTARY_ENCODER_B_PIN A0
#define ROTARY_ENCODER_BUTTON_PIN 16
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

unsigned long previousMillis2000ms = 0;
unsigned long previousMillis72ms = 0;

const int interval2000ms = 2000;
const int interval72ms = 72;
byte keepAliveBits[4] = {0x00, 0x40, 0x80, 0xC0};
uint8_t keepAliveCounter = 0;

static bool driver_installed = false;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
int lastEncoderPos = 0;

void sendCANMessage(uint32_t identifier, uint8_t data[], uint8_t data_length_code = TWAI_FRAME_MAX_DLC) {
  // configure message to transmit
  twai_message_t message = {
    .identifier = identifier,
    .data_length_code = data_length_code,
  };
  message.extd = 1;

  memcpy(message.data, data, data_length_code);

#ifdef DEBUG
  printf("ID: %s ", String(message.identifier, HEX));
  printf("DL: %s ", String(message.data_length_code, DEC));
  for (int i = 0; i < message.data_length_code; i ++) {
    printf("%02x", message.data[i]);

    if (i < message.data_length_code) {
      printf(" ");
    }
  }
  printf("\n");
#endif

  // Queue message for transmission
  esp_err_t result = twai_transmit(&message, portMAX_DELAY);
  if (result != ESP_OK) {
    printf("\n%s: Failed to queue the message for transmission.\n", esp_err_to_name(result));
  }
}

void IRAM_ATTR readEncoderISR() {
	rotaryEncoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);

  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);

  rotaryEncoder.areEncoderPinsPulldownforEsp32 = false;
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 20, false);
  rotaryEncoder.disableAcceleration();

  // Initialize configuration structures using macro initializers
#ifdef DEBUG
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK);
#else
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
#endif
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

  Serial.println("Setup complete...");
}

void loop() {
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }

  unsigned long currentMillis = millis();

  lastEncoderPos = rotaryEncoder.readEncoder();
#ifdef DEBUG
  if (rotaryEncoder.encoderChanged())	{
    Serial.print("EncoderValue: ");
    Serial.println(lastEncoderPos);
  }
#endif

  if (currentMillis - previousMillis2000ms >= interval2000ms) {
    keepAliveCounter = (keepAliveCounter + 1) & 3;

    uint8_t dataKeepAlive[] = {
      keepAliveBits[keepAliveCounter], 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00
    };
    sendCANMessage(0x1ae0092c, dataKeepAlive, sizeof(dataKeepAlive));

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
