#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#define STEP PWM_USEC(100)
enum direction {
	DOWN,
	UP,
};

//Servos
#define SERVO_MID_US (1439)
#define SERVO_STEP (SERVO_MID_US - 10)

static const struct pwm_dt_spec servo_left = PWM_DT_SPEC_GET(DT_NODELABEL(servo_left));
static const struct pwm_dt_spec servo_right = PWM_DT_SPEC_GET(DT_NODELABEL(servo_right));
//Note - min and max are the same for both servos
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo_left), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo_left), max_pulse);

//ADC's
// #define TOUCHSCREEN_CENTER (1615)
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

//private prototypes:
uint16_t readX(void);
uint16_t ready(void);


//public functions

int main(void)
{
	printk("Amazing Balancing Ball!!!\r\n");

	enum direction dir = UP;
	int ret;
	int err;

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

	printk("Initializing pin with inactive level.\n");

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

    uint32_t pulse_width_left = min_pulse;
    uint32_t pulse_width_right = max_pulse;

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

	if (!pwm_is_ready_dt(&servo_left)) {
		printk("Error: PWM device %s is not ready\n",
		       servo_left.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&servo_right)) {
		printk("Error: PWM device %s is not ready\n",
		       servo_right.dev->name);
		return 0;
	}

    while (1) {

		// ret = adc_read(touch_sensor, &seq_x);
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

		err = gpio_pin_toggle_dt(&x_m_pin);
		if (err != 0) {
			printk("Setting x_m_pin level failed: %d\n", err);
		}
		err = gpio_pin_toggle_dt(&x_p_pin);
		if (err != 0) {
			printk("Setting x_p_pin level failed: %d\n", err);
		}
		err = gpio_pin_toggle_dt(&y_m_pin);
		if (err != 0) {
			printk("Setting y_m_pin level failed: %d\n", err);
		}
		err = gpio_pin_toggle_dt(&y_p_pin);
		if (err != 0) {
			printk("Setting y_p_pin level failed: %d\n", err);
		}

        // ret = pwm_set_pulse_dt(&servo_left, pulse_width_left);
        // if (ret < 0) {
		// 	printk("Error %d: failed to set pulse width left\n", ret);
		// 	return 0;
		// }

        // ret = pwm_set_pulse_dt(&servo_right, pulse_width_right);
		// if (ret < 0) {
		// 	printk("Error %d: failed to set pulse width right\n", ret);
		// 	return 0;
		// }

		if (dir == DOWN) {
			if (pulse_width_left <= min_pulse) {
				dir = UP;
				pulse_width_left = min_pulse;
			} else {
				pulse_width_left -= STEP;
			}

            pulse_width_right += STEP;

			if (pulse_width_right >= max_pulse) {
				dir = DOWN;
				pulse_width_right = max_pulse;
			}
		} else {
			pulse_width_left += STEP;

			if (pulse_width_left >= max_pulse) {
				dir = DOWN;
				pulse_width_left = max_pulse;
			}

            if (pulse_width_right <= min_pulse) {
				dir = UP;
				pulse_width_right = min_pulse;
			} else {
				pulse_width_right -= STEP;
			}
		}

        k_sleep(K_MSEC(1000));
    }
}


/***
 *  private implementations
 */
// uint16_t readX() {
//   uint16_t value = 0;
  
//   pinMode(YP_PIN, INPUT);
//   pinMode(YM_PIN, INPUT);
//   pinMode(XM_PIN, OUTPUT);
//   pinMode(XP_PIN, OUTPUT);
  
//   digitalWrite(XP_PIN, LOW);
//   digitalWrite(XM_PIN, HIGH);
  
// //  delay_us(200);
//   delay(1);
//   value = analogRead(YP_PIN);

//   digitalWrite(XM_PIN, LOW);
//   pinMode(XM_PIN, INPUT);
//   pinMode(XP_PIN, INPUT);
// //  return(value-1024);
//   return(value);
// }


// uint16_t readY() {
//   uint16_t value = 0;
   
//   pinMode(XM_PIN, INPUT);
//   pinMode(XP_PIN, INPUT);
//   pinMode(YP_PIN, OUTPUT);
//   pinMode(YM_PIN, OUTPUT);
 
//   digitalWrite(YP_PIN, LOW);
//   digitalWrite(YM_PIN, HIGH);
// //  delay_us(200);
//   delay(1);
//   value = analogRead(XM_PIN);

//   digitalWrite(YM_PIN, LOW);
//   pinMode(YP_PIN, INPUT);
//   pinMode(YM_PIN, INPUT);

// //  return (value-1024);
//   return(value);
// }

