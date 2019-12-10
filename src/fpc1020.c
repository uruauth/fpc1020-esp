#include <string.h>

#include <esp_err.h>
#include <esp_log.h>

#include <driver/spi_master.h>

#include "fpc1020.h"

#define LOG_TAG "FPC1020"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 14
#define PIN_NUM_CS 15

#define PIN_NUM_IRQ 4

static spi_device_handle_t fpc1020_spi;

esp_err_t fpc1020_init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 192 * 192};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS, //CS pin
        .queue_size = 7,            //We want to be able to queue 7 transactions at a time
        // .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to init SPI bus: %d", ret);
        return ret;
    }

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &fpc1020_spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to add SPI device: %d", ret);
        return ret;
    }

    return ret;
}

esp_err_t fpc1020_get_hwid(uint16_t *hwid)
{
    static uint8_t cmd[2] = {0xFC, 0x00};

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 3;
    t.tx_buffer = cmd;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void *)0;

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve hardware id: %d", ret);
        return ret;
    }

    *hwid = *(uint16_t *)t.rx_data;

    return ret;
}