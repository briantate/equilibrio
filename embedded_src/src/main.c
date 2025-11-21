#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/pwm.h>
#include "pid.h"
#include "resistive_touch_sensor.h"

#define STEP PWM_USEC(100)
#define SAMPLE_RATE (20) //ms

//Servos
#define SERVO_MID_US (PWM_USEC(1439))
#define SERVO_STEP (SERVO_MID_US - PWM_USEC(10))

uint32_t pid_x_output = SERVO_MID_US;
uint32_t pid_y_output = SERVO_MID_US;

static const struct pwm_dt_spec servo_left = PWM_DT_SPEC_GET(DT_NODELABEL(servo_left));
static const struct pwm_dt_spec servo_right = PWM_DT_SPEC_GET(DT_NODELABEL(servo_right));
//Note - min and max are the same for both servos
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo_left), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo_left), max_pulse);

uint16_t sensorX_val = 0;
uint16_t sensorY_val = 0;
bool last_ball_state = true;

// PID
pid_t pid_x = {
	.kp = 4,
	.ki = 0.2,
	.kd = 3100,
	.error_last = 0,
	.PID_i = 0,
	.sampling_period = SAMPLE_RATE
};

pid_t pid_y = {
	.kp = 4,
	.ki = 0.2,
	.kd = 3100,
	.error_last = 0,
	.PID_i = 0,
	.sampling_period = SAMPLE_RATE
};

// private prototypes:
static bool isBallDetected(uint16_t x, uint16_t y);
static long map(long x, long in_min, long in_max, long out_min, long out_max);

//public functions

int main(void)
{
	printk("Amazing Balancing Ball!!!\r\n");

	int err;

	// check if gpio are ready //ToDo: is this needed?
	printk("check if gpio are ready\r\n");
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
	printk("configure gpio\r\n");
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

	//configure adc
	printk("configure adc\r\n");
	if(!device_is_ready(touch_sensor_adc)){
		printk("Touch Sensor is not ready\r\n");
		return 0;
	}

	err = adc_channel_setup(touch_sensor_adc, &touch_sensor_adc_x);
	if(err<0){
		printk("Could not set up touch sensor x\r\n");
		return 0;
	}

	err = adc_channel_setup(touch_sensor_adc, &touch_sensor_adc_y);
	if(err<0){
		printk("Could not set up touch sensor y\r\n");
		return 0;
	}


    uint32_t pulse_width_left = SERVO_MID_US;
    uint32_t pulse_width_right = SERVO_MID_US;

	//
	printk("check if pwms are ready\r\n");
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

	//level the table
	pwm_set_pulse_dt(&servo_left, pulse_width_left);
	pwm_set_pulse_dt(&servo_right, pulse_width_right);

    while (1) {

		sensorX_val = touch_sensor_read_x();
		sensorY_val = touch_sensor_read_y();
		// sensorX_val = touch_sensor_read(&sensor_x);
		// sensorY_val = touch_sensor_read(&sensor_y);
		printk("x: %u ", sensorX_val);
		printk("y: %u\r\n", sensorY_val);
		
		bool is_ball_detected = isBallDetected(sensorX_val,sensorY_val);
		if(!is_ball_detected){
			if(last_ball_state)
				printk("no ball detected\r\n");

			pid_x_output = SERVO_MID_US;
			pid_y_output = SERVO_MID_US;
			pid_clear_values(&pid_x);
			pid_clear_values(&pid_y);
		}
		else{ 
			if(!last_ball_state)
				printk("ball detected\r\n");

			pid_x_output = pid_control(&pid_x, TOUCHSCREEN_CENTER, sensorX_val);
			pulse_width_left = map(pid_x_output, -16000, 16000, min_pulse, max_pulse);
			pid_y_output = pid_control(&pid_y, sensorY_val, TOUCHSCREEN_CENTER);
			pulse_width_right = map(pid_y_output, -16000, 16000, min_pulse, max_pulse);
		}
		last_ball_state = is_ball_detected;

        err = pwm_set_pulse_dt(&servo_left, pulse_width_left);
        if (err < 0) {
			printk("Error %d: failed to set pulse width left\n", err);
			return 0;
		}

        err = pwm_set_pulse_dt(&servo_right, pulse_width_right);
		if (err < 0) {
			printk("Error %d: failed to set pulse width right\n", err);
			return 0;
		}

        k_sleep(K_MSEC(SAMPLE_RATE));
    }
}

// private functions:

static bool isBallDetected(uint16_t x, uint16_t y)
{
    return((x > 300) && (y > 300));
}

// helper function to map a value from one range to another
static long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}