#pragma once

typedef struct
{
    uint8_t command_done : 1;
    uint8_t image_available : 1;
    uint8_t error : 1;
    uint8_t finger_down : 1;
} fpc1020_interrupt_t;

esp_err_t fpc1020_init();

esp_err_t fpc1020_get_hwid(uint16_t *hwid);

esp_err_t fpc1020_capture_image();

esp_err_t fpc1020_read_image();

esp_err_t fpc1020_read_interrupt(fpc1020_interrupt_t *interrupt, uint8_t clear);

esp_err_t fpc1020_finger_present_query();

esp_err_t fpc1020_wait_for_finger();

esp_err_t fpc1020_sleep(uint8_t mode);

esp_err_t fpc1020_soft_reset();

esp_err_t fpc1020_clkbist();

esp_err_t fpc1020_irq_status(uint8_t* status);