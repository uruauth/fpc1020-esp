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
        .command_bits = 8,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .mode = 0,                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS, //CS pin
        .queue_size = 1,            //We want to be able to queue 7 transactions at a time
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
    spi_transaction_t t = {
        .cmd = 0xFC,
        .length = 8 * 2,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve hardware id: %d", ret);
        return ret;
    }

    *hwid = t.rx_data[0] << 8 | t.rx_data[1];

    return ret;
}

esp_err_t fpc1020_read_interrupt(fpc1020_interrupt_t *interrupt, uint8_t clear)
{
    spi_transaction_t t = {
        .cmd = clear ? 0x1C : 0x18,
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve interrupt status: %d", ret);
        return ret;
    }

    ESP_LOGI(LOG_TAG, "Received interrupt flags 0x%X", t.rx_data[0]);

    interrupt->command_done = (t.rx_data[0] & (1 << 7)) != 0;
    interrupt->image_available = (t.rx_data[0] & (1 << 5)) != 0;
    interrupt->error = (t.rx_data[0] & (1 << 2)) != 0;
    interrupt->finger_down = (t.rx_data[0] & (1 << 0)) != 0;

    return ret;
}

esp_err_t fpc1020_finger_present_query()
{
    spi_transaction_t t = {
        .cmd = 0x20};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to query finger present: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t fpc1020_get_finger_present_status(uint16_t *status)
{
    spi_transaction_t t = {
        .cmd = 0xD4,
        .length = 8 * 2,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve hardware id: %d", ret);
        return ret;
    }

    *status = t.rx_data[0] << 8 | t.rx_data[1];

    return ret;
}