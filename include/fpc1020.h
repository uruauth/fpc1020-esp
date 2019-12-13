#pragma once

#include <esp_system.h>

typedef enum
{
    /* --- Common registers --- */
    FPC102X_REG_FPC_STATUS = 20,                /* RO, 1 bytes  */
    FPC102X_REG_READ_INTERRUPT = 24,            /* RO, 1 byte   */
    FPC102X_REG_READ_INTERRUPT_WITH_CLEAR = 28, /* RO, 1 byte   */
    FPC102X_REG_READ_ERROR_WITH_CLEAR = 56,     /* RO, 1 byte   */
    FPC102X_REG_MISO_EDGE_RIS_EN = 64,          /* WO, 1 byte   */
    FPC102X_REG_FPC_CONFIG = 68,                /* RW, 1 byte   */
    FPC102X_REG_IMG_SMPL_SETUP = 76,            /* RW, 3 bytes  */
    FPC102X_REG_CLOCK_CONFIG = 80,              /* RW, 1 byte   */
    FPC102X_REG_IMG_CAPT_SIZE = 84,             /* RW, 4 bytes  */
    FPC102X_REG_IMAGE_SETUP = 92,               /* RW, 1 byte   */
    FPC102X_REG_ADC_TEST_CTRL = 96,             /* RW, 1 byte   */
    FPC102X_REG_IMG_RD = 100,                   /* RW, 1 byte   */
    FPC102X_REG_SAMPLE_PX_DLY = 104,            /* RW, 8 bytes  */
    FPC102X_REG_PXL_RST_DLY = 108,              /* RW, 1 byte   */
    FPC102X_REG_TST_COL_PATTERN_EN = 120,       /* RW, 2 bytes  */
    FPC102X_REG_CLK_BIST_RESULT = 124,          /* RW, 4 bytes  */
    FPC102X_REG_ADC_WEIGHT_SETUP = 132,         /* RW, 1 byte   */
    FPC102X_REG_ANA_TEST_MUX = 136,             /* RW, 4 bytes  */
    FPC102X_REG_FINGER_DRIVE_CONF = 140,        /* RW, 1 byte   */
    FPC102X_REG_FINGER_DRIVE_DLY = 144,         /* RW, 1 byte   */
    FPC102X_REG_OSC_TRIM = 148,                 /* RW, 2 bytes  */
    FPC102X_REG_ADC_WEIGHT_TABLE = 152,         /* RW, 10 bytes */
    FPC102X_REG_ADC_SETUP = 156,                /* RW, 5 bytes  */
    FPC102X_REG_ADC_SHIFT_GAIN = 160,           /* RW, 2 bytes  */
    FPC102X_REG_BIAS_TRIM = 164,                /* RW, 1 byte   */
    FPC102X_REG_PXL_CTRL = 168,                 /* RW, 2 bytes  */
    FPC102X_REG_FPC_DEBUG = 208,                /* RO, 1 bytes  */
    FPC102X_REG_FINGER_PRESENT_STATUS = 212,    /* RO, 2 bytes  */
    FPC102X_REG_HWID = 252,                     /* RO, 2 bytes  */
    /* --- fpc1020/21 specific --- */
    FPC1020_REG_FNGR_DET_THRES = 216, /* RW, 1 byte   */
    FPC1020_REG_FNGR_DET_CNTR = 220,  /* RW, 2 bytes  */
} fpc1020_reg_t;

typedef enum
{
    SLEEP,
    DEEP_SLEEP,
    IDLE
} fpc1020_sleep_mode_t;

// init the defice
esp_err_t fpc1020_init();

esp_err_t fpc1020_test();

esp_err_t fpc1020_read_interrupt(uint8_t *interrupt, uint8_t clear);

esp_err_t fpc1020_finger_present_query();

esp_err_t fpc1020_wait_for_finger();

esp_err_t fpc1020_sleep(fpc1020_sleep_mode_t mode);

esp_err_t fpc1020_get_error(uint8_t *error);

esp_err_t fpc1020_get_clkbist();

esp_err_t fpc1020_get_image_capture_size(uint32_t *size);

esp_err_t fpc1020_get_test_pattern(uint16_t *testPattern);

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
