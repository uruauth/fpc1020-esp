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

#define CHECK_RET(f, message)           \
    {                                   \
        esp_err_t ret = (f);            \
        if (ret != ESP_OK)              \
        {                               \
            ESP_LOGE(LOG_TAG, message); \
            return ret;                 \
        }                               \
    }

static spi_device_handle_t fpc1020_spi;

/**
 * @brief Init FPC1020 device SPI
 *
 * @return esp_err_t
 */
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

/**
 * @brief Send command to FPC1020 device
 *
 * @param cmd
 * @return esp_err_t
 */
static esp_err_t fpc1020_command(uint8_t cmd)
{
    spi_transaction_t t = {
        .cmd = cmd};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to write command: %d", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief Transfer one byte to FPC1020 device
 *
 * @param cmd
 * @param val
 * @return esp_err_t
 */
static esp_err_t fpc1020_transmit_uint8(uint8_t cmd, uint8_t *val)
{
    spi_transaction_t t = {
        .cmd = cmd,
        .length = 8,
        .tx_buffer = val,
        .rx_buffer = val};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to transmit data: %d", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief Transfer 2 bytes to FPC1020 device
 *
 * @param cmd
 * @param val
 * @return esp_err_t
 */
static esp_err_t fpc1020_transmit_uint16(uint8_t cmd, uint16_t *val)
{
    spi_transaction_t t = {
        .cmd = cmd,
        .length = 8 * 2,
        .tx_buffer = val,
        .rx_buffer = val};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to transmit data: %d", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief Transfer 4 bytes to FPC1020 device
 *
 * @param cmd
 * @param val
 * @return esp_err_t
 */
static esp_err_t fpc1020_transmit_uint32(uint8_t cmd, uint32_t *val)
{
    spi_transaction_t t = {
        .cmd = cmd,
        .length = 8 * 4,
        .tx_buffer = val,
        .rx_buffer = val};

    esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to transmit data: %d", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief Get Hardware ID
 *
 * @param hwid
 * @return esp_err_t
 */
esp_err_t fpc1020_get_hwid(uint16_t *hwid)
{
    CHECK_RET(fpc1020_transmit_uint16(0xFC, hwid), "Failed to retrieve hardware id");

    return ESP_OK;
}

esp_err_t fpc1020_read_interrupt(fpc1020_interrupt_t *interrupt, uint8_t clear)
{
    uint8_t idata = 0;

    esp_err_t ret = fpc1020_transmit_uint8(clear ? 0x1C : 0x18, &idata);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve interrupt status: %d", ret);
        return ret;
    }

    ESP_LOGI(LOG_TAG, "Received interrupt flags 0x%X, %s", idata, clear ? "cleared" : "not cleared");

    interrupt->command_done = (idata & (1 << 7)) != 0;
    interrupt->image_available = (idata & (1 << 5)) != 0;
    interrupt->error = (idata & (1 << 2)) != 0;
    interrupt->finger_down = (idata & (1 << 0)) != 0;

    return ret;
}

esp_err_t fpc1020_finger_present_query()
{
    ESP_LOGI(LOG_TAG, "Finger present query");

    CHECK_RET(fpc1020_command(0x20), "Failed to query finger present");

    return ESP_OK;
}

esp_err_t fpc1020_get_finger_present_status(uint16_t *status)
{
    CHECK_RET(fpc1020_transmit_uint16(0xD4, status), "");

    return ESP_OK;
}

esp_err_t fpc1020_get_error(uint8_t *error)
{
    CHECK_RET(fpc1020_transmit_uint8(0x38, error), "Failed to retrieve error");

    if (*error != 0)
    {
        ESP_LOGW(LOG_TAG, "Error: %d", *error);
    }

    return ESP_OK;
}

esp_err_t fpc1020_get_finger_drive_conf(fpc1020_finger_drive_conf_t *conf)
{
    uint8_t dt = 0;

    dt |= conf->fngrDrvVdBstEn ? (1 << 5) : 0;
    dt |= conf->fngrDrvVdIntEn ? (1 << 4) : 0;
    dt |= conf->fngrDrvExtInv ? (1 << 3) : 0;
    dt |= conf->fngrDrvTst ? (1 << 2) : 0;
    dt |= conf->fngrDrvExt ? (1 << 1) : 0;

    esp_err_t ret = fpc1020_transmit_uint8(0x8C, &dt);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to retrieve finger drive conf: %d", ret);
        return ret;
    }

    ESP_LOGI(LOG_TAG, "Finger Drive Conf: 0x%x", dt);

    conf->fngrDrvVdBstEn = (dt & (1 << 5)) != 0;
    conf->fngrDrvVdIntEn = (dt & (1 << 4)) != 0;
    conf->fngrDrvExtInv = (dt & (1 << 3)) != 0;
    conf->fngrDrvTst = (dt & (1 << 2)) != 0;
    conf->fngrDrvExt = (dt & (1 << 1)) != 0;

    return ret;
}

esp_err_t fpc1020_get_image_capture_size(uint32_t *size)
{
    CHECK_RET(fpc1020_transmit_uint32(0x8C, size), "Failed to retrieve capture size");

    return ESP_OK;
}

esp_err_t fpc1020_get_test_pattern(uint16_t *testPattern)
{
    CHECK_RET(fpc1020_transmit_uint16(0x78, testPattern), "Failed to retrieve test pattern");

    return ESP_OK;
}