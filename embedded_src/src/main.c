#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/pwm.h>
#include "resistive_touch_sensor.h"

#define STEP PWM_USEC(100)
enum direction {
	DOWN,
	UP,
};

//Servos
#define SERVO_MID_US (PWM_USEC(1439))
#define SERVO_STEP (SERVO_MID_US - 10)

static const struct pwm_dt_spec servo_left = PWM_DT_SPEC_GET(DT_NODELABEL(servo_left));
static const struct pwm_dt_spec servo_right = PWM_DT_SPEC_GET(DT_NODELABEL(servo_right));
//Note - min and max are the same for both servos
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo_left), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo_left), max_pulse);


//public functions

int main(void)
{
	printk("Amazing Balancing Ball!!!\r\n");

	int ret;
	touch_sensor_init();

    uint32_t pulse_width_left = SERVO_MID_US; //max_pulse;
    uint32_t pulse_width_right = SERVO_MID_US; //min_pulse;

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

		touch_sensor_read_x();
		touch_sensor_read_y();

	// 	if(!isBallDetected(sensorX,sensorY))
	// 	{
	// 		printf("no ball detected\r\n");
	// 		servoX_val = SERVO_MID_US;
	// 		servoY_val = SERVO_MID_US;
	// 		errorLastX = 0;
	// 		errorLastY = 0;
	// 		PID_i_x = 0;
	// 		PID_i_y = 0;
	// 	}
	// 	else
	// 	{
	// 		if(run)
	// 		{
	// 			servoX_val = PidControl(TOUCHSCREEN_CENTER,sensorX, &errorLastX, &PID_i_x);
	// 			servoY_val = PidControl(sensorY, TOUCHSCREEN_CENTER, &errorLastY, &PID_i_y);
			
	// //            printf("%f,%f\r\n", errorLastX, errorLastY);
	// 			printf("%u,%u\r\n", sensorX, sensorY);
	// //            printf("%lu,%lu\r\n", servoX_val, servoY_val);
	// 		}
	// 	}

        ret = pwm_set_pulse_dt(&servo_left, pulse_width_left);
        if (ret < 0) {
			printk("Error %d: failed to set pulse width left\n", ret);
			return 0;
		}

        ret = pwm_set_pulse_dt(&servo_right, pulse_width_right);
		if (ret < 0) {
			printk("Error %d: failed to set pulse width right\n", ret);
			return 0;
		}

        k_sleep(K_MSEC(1000));
    }
}

