#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

#include <driver/spi_master.h>

#include "fpc1020.h"

#include "sdkconfig.h"

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
        .miso_io_num = CONFIG_FPC1020A_PIN_MISO,
        .mosi_io_num = CONFIG_FPC1020A_PIN_MISO,
        .sclk_io_num = CONFIG_FPC1020A_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 192 * 192 + 1};

    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .mode = 0,                  //SPI mode 0
        .spics_io_num = CONFIG_FPC1020A_PIN_CS, //CS pin
        .queue_size = 1,            //We want to be able to queue 7 transactions at a time
        // .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 2);
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

    ESP_LOGI(LOG_TAG, "FPC1020A SPI Init done");

    return ret;
}

/**
 * @brief Send command to FPC1020 device
 *
 * @param cmd
 * @return esp_err_t
 */
static esp_err_t fpc1020_command(fpc1020_reg_t cmd)
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
static esp_err_t fpc1020_transmit_uint8(fpc1020_reg_t cmd, uint8_t *val)
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
static esp_err_t fpc1020_transmit_uint16(fpc1020_reg_t cmd, uint16_t *val)
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
static esp_err_t fpc1020_transmit_uint32(fpc1020_reg_t cmd, uint32_t *val)
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
 * @brief
 *
 * @param cmd
 * @param val
 * @return esp_err_t
 */
static esp_err_t fpc1020_transmit_uint64(fpc1020_reg_t cmd, uint64_t *val)
{
    spi_transaction_t t = {
        .cmd = cmd,
        .length = 8 * 8,
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
 * @brief Setup FPC1020 device
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_setup()
{
    uint8_t val8;
    uint16_t val16;
    uint32_t val32;
    uint64_t val64;

    // revision 3
    val64 = 0x1717171723232323;
    CHECK_RET(fpc1020_transmit_uint64(FPC102X_REG_SAMPLE_PX_DLY, &val64), "");

    val8 = 0x0f;
    CHECK_RET(fpc1020_transmit_uint8(FPC102X_REG_PXL_RST_DLY, &val8), "");

    val8 = 0x18;
    CHECK_RET(fpc1020_transmit_uint8(FPC102X_REG_FINGER_DRIVE_DLY, &val8), "");

    val8 = 0x02;
    CHECK_RET(fpc1020_transmit_uint8(FPC102X_REG_FINGER_DRIVE_CONF, &val8), "");

    val16 = 0x1002;
    CHECK_RET(fpc1020_transmit_uint16(FPC102X_REG_ADC_SHIFT_GAIN, &val16), "");

    val16 = 0x000a | 0x0F00;
    CHECK_RET(fpc1020_transmit_uint16(FPC102X_REG_PXL_CTRL, &val16), "");

    val8 = 0x03 | 0x08;
    CHECK_RET(fpc1020_transmit_uint8(FPC102X_REG_IMAGE_SETUP, &val8), "");

    val8 = 0x50;
    CHECK_RET(fpc1020_transmit_uint8(FPC1020_REG_FNGR_DET_THRES, &val8), "");

    val16 = 0x00FF;
    CHECK_RET(fpc1020_transmit_uint16(FPC1020_REG_FNGR_DET_CNTR, &val16), "");
    return ESP_OK;
}

/**
 * @brief Read interrupt with no clear / Read interrupt with clear
 *
 * Read interrupt register. Two byte access, command and interrupt data.
 *
 * @param interrupt
 * @param clear
 * @return esp_err_t
 */
esp_err_t fpc1020_read_interrupt(uint8_t *interrupt, uint8_t clear)
{
    CHECK_RET(fpc1020_transmit_uint8(clear ? 0x1C : 0x18, interrupt), "Failed to retrieve interrupt status");

    return ESP_OK;
}

/**
 * @brief Finger present query
 *
 * Checks if a finger is present. One byte access, only the command is transmitted.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_finger_present_query()
{
    CHECK_RET(fpc1020_command(0x20), "Failed to query finger present");

    return ESP_OK;
}

/**
 * @brief Wait for finger present
 *
 * Continue to check for a finger until a finger is present. One byte access, only the command is transmitted.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_wait_for_finger()
{
    CHECK_RET(fpc1020_command(0x24), "Failed to wait for finger");

    return ESP_OK;
}

/**
 * @brief Activate sleep mode
 *
 * Go to Sleep Mode. One byte access, only the command is transmitted.
 *
 * @param mode
 * @return esp_err_t
 */
esp_err_t fpc1020_sleep(fpc1020_sleep_mode_t mode)
{
    switch (mode)
    {
    case SLEEP:
        CHECK_RET(fpc1020_command(0x28), "Failed to enter sleep mode");
        break;
    case DEEP_SLEEP:
        CHECK_RET(fpc1020_command(0x2C), "Failed to enter deep sleep mode");
        break;
    case IDLE:
        CHECK_RET(fpc1020_command(0x34), "Failed to enter idle mode");
        break;
    default:
        ESP_LOGE(LOG_TAG, "Invalid sleep mode %d", mode);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Errors
 *
 * The name of the error register is fpcError. The fpcError register has access to two bytes: one address byte and one read byte.
 *
 * @param error
 * @return esp_err_t
 */
esp_err_t fpc1020_get_error(uint8_t *error)
{
    CHECK_RET(fpc1020_transmit_uint8(0x38, error), "Failed to retrieve error");

    return ESP_OK;
}

/**
 * @brief ClkBIST
 *
 * The command is used to measure the two internal oscillators’ frequency in relation to a known SPICLK frequency.
 * The command is done with one byte access plus a number of SPICLK cycles which is decided by the frequency of the OscLo oscillator.
 * The measurement needs to run between two rising edges of the OscLo oscillator. Sending more SPICLK pulses will not affect the measurement.
 * The result is read in the clkBistResult register.
 * For more information on calculating clock frequencies, see section 5.7.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_get_clkbist()
{
    return ESP_OK;
}

/**
 * @brief Image Capture Size
 *
 * @param size
 * @return esp_err_t
 */
esp_err_t fpc1020_get_image_capture_size(uint32_t *size)
{
    CHECK_RET(fpc1020_transmit_uint32(0x8C, size), "Failed to retrieve capture size");

    return ESP_OK;
}

/**
 * @brief Test Pattern
 *
 * @param testPattern
 * @return esp_err_t
 */
esp_err_t fpc1020_get_test_pattern(uint16_t *testPattern)
{
    CHECK_RET(fpc1020_transmit_uint16(0x78, testPattern), "Failed to retrieve test pattern");

    return ESP_OK;
}

/**
 * @brief Oscillator Frequency Calculation
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_get_clkbist_result()
{
    return ESP_OK;
}

/**
 * @brief Finger Drive
 *
 * @param conf
 * @return esp_err_t
 */
esp_err_t fpc1020_get_finger_drive_conf(uint8_t *conf)
{
    CHECK_RET(fpc1020_transmit_uint8(0x8C, conf), "Failed to retrieve finger drive conf");

    return ESP_OK;
}

/**
 * @brief Oscillator Calibration
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_get_osc_trim()
{
    return ESP_OK;
}

/**
 * @brief Shift Gain
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_get_adc_shift_gain()
{
    return ESP_OK;
}

static uint8_t image[192 * 192 + 1];
const size_t imageBufSize = 192 * 192 + 1;

/**
 * @brief Capture image
 *
 * Capture new image. One byte access. Only the command is transmitted.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_capture_image()
{
    // setup?

    //
    CHECK_RET(fpc1020_command(0xC0), "Failed to start capturing image");

    ESP_LOGI(LOG_TAG, "Waiting for the interrupt...");
    // wait for the interrupt
    for (int i = 0; i < 20; i++)
    {
        uint8_t intr = 0;
        ESP_ERROR_CHECK(fpc1020_read_interrupt(&intr, 1));

        if (intr == (1 << 5))
        {
            ESP_LOGI(LOG_TAG, "Image available");

            memset(image, 0, imageBufSize);

            spi_transaction_t t = {
                .cmd = 0xC4,
                .length = imageBufSize * 8,
                .tx_buffer = image,
                .rx_buffer = image};

            esp_err_t ret = spi_device_polling_transmit(fpc1020_spi, &t);

            ESP_LOGI(LOG_TAG, "Image received");

            for(int i = 0; i < 10; i++) {
                ESP_LOG_BUFFER_HEX(LOG_TAG, image, 192 * i + 1);
            }

            break;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    return ESP_OK;
}

/**
 * @brief Read image data
 *
 * Valid data is first received following a command with a dummy byte. The read continues until csN is de-asserted.
 * It is possible to split the reading of an image into several requests.
 * In this case, new commands (all but the first) should be issued without the dummy byte.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_read_image(uint8_t *buf, size_t bufSize)
{
    return ESP_OK;
}

/**
 * @brief Finger Present Status
 *
 * The name of the register for finger present status is fngrPresentStatus.
 * The fngrPresentStatus register has access to three bytes: one address byte, and two data bytes.
 * Current register content is read when data is written to the register.
 *
 * @param status
 * @return esp_err_t
 */
esp_err_t fpc1020_get_finger_present_status(uint16_t *status)
{
    CHECK_RET(fpc1020_transmit_uint16(0xD4, status), "Failed to get finger present status");

    return ESP_OK;
}

/**
 * @brief Finger Detection Threshold
 *
 * The name of the finger detection threshold register is fngrDetThresh.
 * The fngrDetThresh register has access to two bytes: one address byte and one read byte.
 * Current register content is read when data is written to the register
 *
 * @param threshold
 * @return esp_err_t
 */
esp_err_t fpc1020_get_finger_det_thrs(uint8_t *threshold)
{
    CHECK_RET(fpc1020_transmit_uint8(0xD8, threshold), "Failed to read detection threshold")

    return ESP_OK;
}

/**
 * @brief Finger Detection Queries
 *
 * The name of the finger detection query control register is fngrDetCntr.
 * The fngrDetCntr register has access to three bytes: one address byte and two read bytes.
 * Current register content is read when data is written to the register.
 *
 * @param query
 * @return esp_err_t
 */
esp_err_t fpc1020_get_finger_det_cntr(uint16_t *query)
{
    CHECK_RET(fpc1020_transmit_uint16(0xDC, query), "Failed to retrieve finger detection queries");

    return ESP_OK;
}

/**
 * @brief Soft reset
 *
 * Performs a software controlled reset of the chip. One byte access, only the command is transmitted.
 *
 * @return esp_err_t
 */
esp_err_t fpc1020_soft_reset()
{
    CHECK_RET(fpc1020_command(0xF8), "Failed to soft reset");

    return ESP_OK;
}

/**
 * @brief Hardware ID
 *
 * @param hwid
 * @return esp_err_t
 */
esp_err_t fpc1020_get_hwid(uint16_t *hwid)
{
    CHECK_RET(fpc1020_transmit_uint16(0xFC, hwid), "Failed to retrieve hardware id");

    return ESP_OK;
}
