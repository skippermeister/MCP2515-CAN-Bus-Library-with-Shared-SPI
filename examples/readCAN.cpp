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
#include <mcp2515_can.h>

MCP2515Class *CAN;

// GPIO pins used as an example
static const uint8_t pin_miso = 7;
static const uint8_t pin_mosi = 7;
static const uint8_t pin_clk = 5;
static const uint8_t pin_cs = 15;
static const uint8_t pin_irq = 4;

setup()
{
  uint8_t rc;

  auto bus_config = std::make_shared<SpiBusConfig>(
      static_cast<gpio_num_t>(pin_mosi),
      static_cast<gpio_num_t>(pin_miso),
      static_cast<gpio_num_t>(pin_clk)
  );

  spi_device_interface_config_t device_config {
      .command_bits = 0, // set by transactions individually
      .address_bits = 0, // set by transactions individually
      .dummy_bits = 0,
      .mode = 0, // SPI mode 0
      .duty_cycle_pos = 0,
      .cs_ena_pretrans = 2, // only 1 pre and post cycle would be required for register access
      .cs_ena_posttrans = static_cast<uint8_t>(2 * spi_speed / 1000000), // >2 us
      .clock_speed_hz = spi_speed, // 10000000, // 10mhz
      .input_delay_ns = 0,
      .spics_io_num = pin_cs,
      .flags = SPI_DEVICE_HALFDUPLEX,
      .queue_size = 1,
      .pre_cb = nullptr,
      .post_cb = nullptr,
  };

  spi_device_handle_t spi;
  spi = SpiManagerInst.alloc_device("", bus_config, device_config);
  if (!spi) {
    Serial.println("failed to alloc a shared SPI bus");
    while (1);
  }

  CAN = new MCP2515Class(spi, pin_irq);

  if ((rc = CAN->begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ)) != CAN_OK) {
      Serial.printf("MCP2515 failed to initialize. Error code: %d\r\n", rc);
      while(1);
  }

  CAN->setFilterMask(MASK0, 1, 0xFFFFFFFF);  // Look at all incoming bits and...
  CAN->setFilter(RXF0,  1, 0x1081407F);  // filter for this message only
  CAN->setFilterMask(MASK1, 1, myMask);

  // Change to normal mode to allow messages to be transmitted
  if ((rc = CAN->setMode(MCP_NORMAL)) != CAN_OK) {
      Serial.printf("MCP2515 failed to set mode to NORMAL. Error code: %d\r\n", rc);
      while(1);
  }
}

loop()
{
  can_message_t rx_message = {};

  if (!CAN->isInterrupt()) return;  // no interrupt means nothing received from CAN Bus

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
