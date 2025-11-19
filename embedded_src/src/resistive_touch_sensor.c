#include "resistive_touch_sensor.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

//ADC's
// #define TOUCH_SENSOR_X DT_ALIAS(adc_touch_sensor_x)
// #define TOUCH_SENSOR_Y DT_ALIAS(adc_touch_sensor_y)
// static const struct device *touch_sensor = DEVICE_DT_GET(DT_ALIAS(adc_touch_sensor));
// static const struct adc_channel_cfg touch_sensor_x = ADC_CHANNEL_CFG_DT(TOUCH_SENSOR_X);
// static const struct adc_channel_cfg touch_sensor_y = ADC_CHANNEL_CFG_DT(TOUCH_SENSOR_Y);

//GPIO's 
static const struct gpio_dt_spec x_m_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(x_m_pin), gpios);
static const struct gpio_dt_spec x_p_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(x_p_pin), gpios);
static const struct gpio_dt_spec y_m_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(y_m_pin), gpios);
static const struct gpio_dt_spec y_p_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(y_p_pin), gpios);


int touch_sensor_init(void){
    int err;
    // check if gpio are ready //ToDo: is this needed?
	if (!gpio_is_ready_dt(&x_m_pin)) {
		printk("x_m_pin port is not ready.\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&x_p_pin)) {
		printk("x_p_pin port is not ready.\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&y_m_pin)) {
		printk("y_m_pin port is not ready.\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&y_p_pin)) {
		printk("y_p_pin port is not ready.\n");
		return 0;
	}

    //configure pins
	err = gpio_pin_configure_dt(&x_m_pin, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Configuring GPIO pin failed: %d\n", err);
		return 0;
	}
	err = gpio_pin_configure_dt(&x_p_pin, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Configuring GPIO pin failed: %d\n", err);
		return 0;
	}
	err = gpio_pin_configure_dt(&y_m_pin, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Configuring GPIO pin failed: %d\n", err);
		return 0;
	}
	err = gpio_pin_configure_dt(&y_p_pin, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Configuring GPIO pin failed: %d\n", err);
		return 0;
	}

    // uint16_t buf_x;
	// uint16_t val_mv_x;
	// uint32_t vref_mv_x = DT_PROP(TOUCH_SENSOR_X, zephyr_vref_mv);

	// uint16_t buf_y;
	// uint16_t val_mv_y;
	// uint32_t vref_mv_y = DT_PROP(TOUCH_SENSOR_Y, zephyr_vref_mv);

	// struct adc_sequence seq_x = {
	// 	.channels = BIT(touch_sensor_x.channel_id),
	// 	.buffer = &buf_x,
	// 	.buffer_size = sizeof(buf_x),
	// 	.resolution = DT_PROP(TOUCH_SENSOR_X, zephyr_resolution)
	// };

	// struct adc_sequence seq_y = {
	// 	.channels = BIT(touch_sensor_y.channel_id),
	// 	.buffer = &buf_y,
	// 	.buffer_size = sizeof(buf_y),
	// 	.resolution = DT_PROP(TOUCH_SENSOR_Y, zephyr_resolution)
	// };

	// if(!device_is_ready(touch_sensor)){
	// 	printk("Touch Sensor is not ready\r\n");
	// 	return 0;
	// }

	// ret = adc_channel_setup(touch_sensor, &touch_sensor_x);
	// if(ret<0){
	// 	printk("Could not set up touch sensor x\r\n");
	// 	return 0;
	// }

	// ret = adc_channel_setup(touch_sensor, &touch_sensor_y);
	// if(ret<0){
	// 	printk("Could not set up touch sensor y\r\n");
	// 	return 0;
	// }
    return 0;
}

uint16_t touch_sensor_read_x(void){
    uint16_t value = 0;
    gpio_pin_configure_dt(&x_m_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&x_p_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&y_m_pin, GPIO_INPUT);
    gpio_pin_configure_dt(&y_p_pin, GPIO_INPUT);
    
    gpio_pin_set_dt(&x_p_pin, 0);
    gpio_pin_set_dt(&x_m_pin, 1);

    k_sleep(K_MSEC(1));
//   value = analogRead(YP_PIN);
// vlaue = adc_read(touch_sensor, &seq_x);
		// if(ret<0){
		// 	printk("Could not read touch sensor x: %d\r\n", ret);
		// }
		// val_mv_x = buf_x * vref_mv_x /(1 << seq_x.resolution);
		// printk("X RAW: %u, mV: %u\r\n", buf_x, val_mv_x);
		
		// ret = adc_read(touch_sensor, &seq_y);
		// if(ret<0){
		// 	printk("Could not read touch sensor y: %d\r\n", ret);
		// }
		// val_mv_y = buf_y * vref_mv_y /(1 << seq_y.resolution);
		// printk("Y RAW: %u, mV: %u\r\n", buf_y, val_mv_y);

    gpio_pin_set_dt(&x_m_pin, 0);
    gpio_pin_configure_dt(&x_m_pin, GPIO_INPUT);
    gpio_pin_configure_dt(&x_p_pin, GPIO_INPUT);

  return(value);
    
}

uint16_t touch_sensor_read_y(void){
    uint16_t value = 0;
    gpio_pin_configure_dt(&x_m_pin, GPIO_INPUT);
    gpio_pin_configure_dt(&x_p_pin, GPIO_INPUT);
    gpio_pin_configure_dt(&y_m_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&y_p_pin, GPIO_OUTPUT);
    
    gpio_pin_set_dt(&y_p_pin, 0);
    gpio_pin_set_dt(&y_m_pin, 1);

    k_sleep(K_MSEC(1));
//   value = analogRead(YP_PIN);

    gpio_pin_set_dt(&y_m_pin, 0);
    gpio_pin_configure_dt(&y_m_pin, GPIO_INPUT);
    gpio_pin_configure_dt(&y_p_pin, GPIO_INPUT);

    return(value);
}
