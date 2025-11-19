//resistive_touch_sensor.h
#ifndef _RESISTIVE_TOUCH_SENSOR_H
#define _RESISTIVE_TOUCH_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

#define TOUCHSCREEN_CENTER (1615)

int touch_sensor_init(void);
uint16_t touch_sensor_read_x(void);
uint16_t touch_sensor_read_y(void);

#ifdef __cplusplus
}
#endif

#endif //_RESISTIVE_TOUCH_SENSOR_H