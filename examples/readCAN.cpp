/*
 * requieres the SpiManager from project OpenDTU-onBattery
 * for use with ESP32 or ESP32S3 chips
 *
 * The example is not working without the SpiManager library. 
 * The example is only for explaining the usage of the mcp2515_can library in the project OpenDTU-onBattery
 * The example illustrates how to read the HUAWEI Charger via the CAN bus
 *
 * The SpiManager library will be included from the module mcp2515_spi
 */

#include <SpiManager.h>

MCP2515Class *CAN;

// GPIO pins used as an example
static const uint8_t pin_miso = 7;
static const uint8_t pin_mosi = 7;
static const uint8_t pin_clk = 5;
static const uint8_t pin_cs = 15;
static const uint8_t pin_irq = 4;

setup()
{
  uint8_t;

  CAN = new MCP2515Class(pin_miso, pin_mosi, pin_clk, pin_cs);

  if ((rc = CAN->initMCP2515(MCP_ANY, CAN_125KBPS, MCP_8MHZ)) != CAN_OK) {
      Serial.printf("MCP2515 failed to initialize. Error code: %d\r\n", rc);
      while(1);
  }

  pinMode(pin_irq, INPUT_PULLUP);

  CAN->setFilterMask(0, 1, 0xFFFFFFFF);  // Look at all incoming bits and...
  CAN->setFilter(0, 1, 0x1081407F);  // filter for this message only
  CAN->setFilterMask(1, 1, myMask);

	// Change to normal mode to allow messages to be transmitted
	if ((rc = CAN->setMode(MCP_NORMAL)) != CAN_OK) {
      Serial.printf("MCP2515 failed to set mode to NORMAL. Error code: %d\r\n", rc);
      while(1);
  }
}

loop()
{
  can_message_t rx_message = {};

  if (digitalRead(pin_irq)) return;  // nothing received from CAN Bus

  // If CAN_INT pin is low, read receive buffer
  uint8_t rc;
  if ((rc = CAN->readMsgBuf((can_message_t*)&rx_message)) != CAN_OK) {
      Serial.printf("MCP2515 failed to read CAN message: Error code %d\r\n", rc);
      return;
  }

  Serial.printf("received CAN id : %ld\r\n", rx_message.identifier);
  Serial.printf("received msg len: %u\r\n", rx_message.data_length_code);
  Serial.printf("received msg    : ");
  for (uint8_t i=0; i<rx_message.data_length_code; i++) {
    Serial.printf("%02X ", rx_message.data[i]);
  }
  Serial.println();
}
