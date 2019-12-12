#pragma once

#include <esp_system.h>

typedef struct
{
    uint8_t command_done : 1;
    uint8_t image_available : 1;
    uint8_t error : 1;
    uint8_t finger_down : 1;
} fpc1020_interrupt_t;

typedef enum
{
    SLEEP,
    DEEP_SLEEP,
    IDLE
} fpc1020_sleep_mode_t;

typedef struct
{
    uint16_t wait_for_finger : 7;
    uint16_t sleep : 7;
} fpc1020_detect_t;

typedef struct
{
    uint8_t fngrDrvVdBstEn : 1;
    uint8_t fngrDrvVdIntEn : 1;
    uint8_t fngrDrvExtInv : 1;
    uint8_t fngrDrvTst : 1;
    uint8_t fngrDrvExt : 1;
} fpc1020_finger_drive_conf_t;

// init the defice
esp_err_t fpc1020_init();

// Read interrupt status
// 18, 1C
esp_err_t fpc1020_read_interrupt(fpc1020_interrupt_t *interrupt, uint8_t clear);

// 20
esp_err_t fpc1020_finger_present_query();

// 24
esp_err_t fpc1020_wait_for_finger();

// 28, 2C, 34
esp_err_t fpc1020_sleep(fpc1020_sleep_mode_t mode);

// 38
esp_err_t fpc1020_get_error(uint8_t *error);

// 3C
esp_err_t fpc1020_get_clkbist();

// 54
esp_err_t fpc1020_get_image_capture_size(uint8_t *startRow, uint8_t *rowLength, uint8_t *startCol, uint8_t *colLength);

// 78
esp_err_t fpc1020_get_test_pattern();

// 7C
esp_err_t fpc1020_get_clkbist_result();

// 8C
esp_err_t fpc1020_get_finger_drive_conf(fpc1020_finger_drive_conf_t *conf);

// 94
esp_err_t fpc1020_get_ost_trim();

// A0
esp_err_t fpc1020_get_adc_shift_gain();

// C0
esp_err_t fpc1020_capture_image();

// C4
esp_err_t fpc1020_read_image();

// D4
esp_err_t fpc1020_get_finger_present_status(uint16_t *status);

// D8
esp_err_t fpc1020_get_finger_det_thrs(uint8_t *threshold);

// DC
esp_err_t fpc1020_get_finger_det_cntr(fpc1020_detect_t *query);

// F8
esp_err_t fpc1020_soft_reset();

// FC
esp_err_t fpc1020_get_hwid(uint16_t *hwid);
