//resistive_touch_sensor.h
#ifndef _RESISTIVE_TOUCH_SENSOR_H
#define _RESISTIVE_TOUCH_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define TOUCHSCREEN_CENTER (1615)

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

// typedef int      (*high_z_pin_1_t)(void);
// typedef int      (*high_z_pin_2_t)(void);
// typedef int      (*drive_high_pin_t)(void);
// typedef int      (*drive_low_pin_t)(void);
// typedef void     (*wait_to_settle_t)(void);
// typedef uint16_t (*adc_read_t)(void);

// typedef struct {
//     high_z_pin_1_t   high_z_pin_1;
//     high_z_pin_2_t   high_z_pin_2;    
//     drive_high_pin_t drive_high_pin;
//     drive_low_pin_t  drive_low_pin;
//     wait_to_settle_t wait_to_settle;
//     adc_read_t       adc_read;
// }touch_sensor_t;

// uint16_t touch_sensor_read(touch_sensor_t *sensor);

int touch_sensor_init(void);
uint16_t touch_sensor_read_x(void);
uint16_t touch_sensor_read_y(void);

#ifdef __cplusplus
}
#endif

#endif //_RESISTIVE_TOUCH_SENSOR_H