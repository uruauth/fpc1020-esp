#pragma once

#include <esp_system.h>

typedef enum
{
    SLEEP,
    DEEP_SLEEP,
    IDLE
} fpc1020_sleep_mode_t;

// init the defice
esp_err_t fpc1020_init();

esp_err_t fpc1020_read_interrupt(uint8_t *interrupt, uint8_t clear);

esp_err_t fpc1020_finger_present_query();

esp_err_t fpc1020_wait_for_finger();

esp_err_t fpc1020_sleep(fpc1020_sleep_mode_t mode);

esp_err_t fpc1020_get_error(uint8_t *error);

esp_err_t fpc1020_get_clkbist();

esp_err_t fpc1020_get_image_capture_size(uint32_t* size);

esp_err_t fpc1020_get_test_pattern(uint16_t* testPattern);

esp_err_t fpc1020_get_clkbist_result();

esp_err_t fpc1020_get_finger_drive_conf(uint8_t *conf);

esp_err_t fpc1020_get_osc_trim();

esp_err_t fpc1020_get_adc_shift_gain();

esp_err_t fpc1020_capture_image();

esp_err_t fpc1020_read_image();

esp_err_t fpc1020_get_finger_present_status(uint16_t *status);

esp_err_t fpc1020_get_finger_det_thrs(uint8_t *threshold);

esp_err_t fpc1020_get_finger_det_cntr(uint16_t *query);

esp_err_t fpc1020_soft_reset();

esp_err_t fpc1020_get_hwid(uint16_t *hwid);
