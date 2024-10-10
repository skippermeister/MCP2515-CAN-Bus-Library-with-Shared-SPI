// SPDX-License-Identifier: GPL-2.0-or-later

#include "mcp2515_spi.h"

#define SPI_PARAM_LOCK() \
    do {                 \
    } while (xSemaphoreTake(paramLock, portMAX_DELAY) != pdPASS)
#define SPI_PARAM_UNLOCK() xSemaphoreGive(paramLock)

void MCP2515SPIClass::spi_init()
{
    if (paramLock == NULL) paramLock = xSemaphoreCreateMutex();
    if (paramLock == NULL) {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    if (!connection_check_interrupt(static_cast<gpio_num_t>(pin_irq)))
        ESP_ERROR_CHECK(ESP_FAIL);

    // Return to default state once again after connection check
    ESP_ERROR_CHECK(gpio_reset_pin(_pin_irq));
    ESP_ERROR_CHECK(gpio_set_direction(_pin_irq, GPIO_MODE_INPUT));
}

bool MCP2515SPIClass::connection_check_interrupt(gpio_num_t pin_irq)
{
    gpio_set_direction(pin_irq, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin_irq, GPIO_PULLDOWN_ONLY);
    int level = gpio_get_level(pin_irq);

    // Interrupt line must be high
    return level == 1;
}

void MCP2515SPIClass::spi_deinit(void)
{
    spi_bus_remove_device(_spi);
}

void MCP2515SPIClass::spi_reset(void)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 8, // 1 bits = 1 bytes
        .rxlength = 0,
        .user = 0,
        .tx_data = {INSTRUCTION_RESET,0,0,0},
        .rx_data = {0,0,0,0}
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();
}

uint8_t MCP2515SPIClass::spi_readRegister(const REGISTER_t reg)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = (3 * 8), // 24 bits = 3 bytes
        .rxlength = 0, // default to length 24 bits = 3 bytes
        .user = 0,
        .tx_data = {INSTRUCTION_READ, reg, 0x00},
        .rx_data = {0,0,0,0}
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();

    return trans.rx_data[2];
}

void MCP2515SPIClass::spi_readRegisters(const REGISTER_t reg, uint8_t values[], const uint8_t n)
{
    uint8_t tx_data[n + 2] = { INSTRUCTION_READ, reg };
    uint8_t rx_data[n + 2];

    spi_transaction_t trans = {
        .flags = 0,  // use tx_buffer and rx_buffer
        .cmd = 0,
        .addr = 0,
        .length = ((2 + ((size_t)n)) * 8),
        .rxlength = 0,
        .user = 0,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();
    for (uint8_t i = 0; i < n; i++) {
        values[i] = rx_data[i+2];
    }
}

void MCP2515SPIClass::spi_setRegister(const REGISTER_t reg, const uint8_t value)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = (3 * 8),
        .rxlength = 0,
        .user = 0,
        .tx_data = { INSTRUCTION_WRITE, reg, value },
        .rx_data = {0,0,0,0}
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();
}

void MCP2515SPIClass::spi_setRegisters(const REGISTER_t reg, const uint8_t values[], const uint8_t n)
{
    uint8_t data[n + 2];

    data[0] = INSTRUCTION_WRITE;
    data[1] = reg;

    for (uint8_t i=0; i<n; i++) {
        data[i+2] = values[i];
    }

    spi_transaction_t trans = {
        .flags = 0, // use tx_buffer and rx_buffer (point to nullptr)
        .cmd = 0,
        .addr = 0,
        .length = ((2 + ((size_t)n)) * 8),
        .rxlength = 0,
        .user = 0,
        .tx_buffer = data,
        .rx_buffer = nullptr
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();
}

void MCP2515SPIClass::spi_modifyRegister(const REGISTER_t reg, const uint8_t mask, const uint8_t data)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = (4 * 8),
        .rxlength = 0,
        .user = 0,
        .tx_data = { INSTRUCTION_BITMOD, reg, mask, data },
        .rx_data = {0,0,0,0}
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();
}

uint8_t MCP2515SPIClass::spi_getStatus(void)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = (2 * 8),
        .rxlength = 0,
        .user = 0,
        .tx_data = { INSTRUCTION_READ_STATUS, 0x00 },
        .rx_data = {0,0,0,0}
    };

    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    SPI_PARAM_UNLOCK();

    return trans.rx_data[1];
}
