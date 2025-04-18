/**
 * @file RA8875_TOUCH.h
 */

#ifndef RA8875X_TOUCH__H
#define RA8875X_TOUCH__H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include <stdint.h>
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/

#define RA8875_X_MIN       CONFIG_DISP_TOUCH_X_MIN
#define RA8875_Y_MIN       CONFIG_DISP_TOUCH_Y_MIN
#define RA8875_X_MAX       CONFIG_DISP_TOUCH_X_MAX
#define RA8875_Y_MAX       CONFIG_DISP_TOUCH_Y_MAX
#define RA8875_X_INV       CONFIG_DISP_TOUCH_INVERT_X
#define RA8875_Y_INV       CONFIG_DISP_TOUCH_INVERT_Y
#define RA8875_XY_SWAP     CONFIG_DISP_TOUCH_XY_SWAP

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ra8875_touch_init(void);
void ra8875_touch_enable(bool enable);
bool ra8875_touch_read(lv_indev_drv_t * drv, lv_indev_data_t * data);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RA8875_TOUCH__H */
