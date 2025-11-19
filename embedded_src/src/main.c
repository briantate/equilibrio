#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/pwm.h>
#include "pid.h"
#include "resistive_touch_sensor.h"

#define STEP PWM_USEC(100)

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

uint16_t sensorX = 0;
uint16_t sensorY = 0;
bool last_ball_state = true;

// PID
pid_t pid_x = {
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.error_last = 0,
	.PID_i = 0
};

pid_t pid_y = {
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.error_last = 0,
	.PID_i = 0
};

// private prototypes:
static bool isBallDetected(uint16_t x, uint16_t y);
static void SetServoXOnTime(uint32_t onTimeUs);
static void SetServoYOnTime(uint32_t onTimeUs);
static long map(long x, long in_min, long in_max, long out_min, long out_max);

//public functions

int main(void)
{
	printk("Amazing Balancing Ball!!!\r\n");

	int ret;

	touch_sensor_init();

    uint32_t pulse_width_left = SERVO_MID_US;
    uint32_t pulse_width_right = SERVO_MID_US;

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

		sensorX = touch_sensor_read_x();
		sensorY = touch_sensor_read_y();
		
		bool is_ball_detected = isBallDetected(sensorX,sensorY);
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
				printk("ball detected");

			pid_x_output = pid_control(&pid_x, TOUCHSCREEN_CENTER, sensorX);
			pulse_width_left = map(pid_x_output, -16000, 16000, min_pulse, max_pulse);
			pid_y_output = pid_control(&pid_y, sensorY, TOUCHSCREEN_CENTER);
			pulse_width_right = map(pid_y_output, -16000, 16000, min_pulse, max_pulse);
			
	// //            printf("%f,%f\r\n", errorLastX, errorLastY);
	// 			printf("%u,%u\r\n", sensorX, sensorY);
	// //            printf("%lu,%lu\r\n", pid_x_output, pid_y_output);
		}
		last_ball_state = is_ball_detected;

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

        k_sleep(K_MSEC(20)); //run the PID every 20ms
    }
}

// private functions:

static bool isBallDetected(uint16_t x, uint16_t y)
{
    return((x > 300) && (y > 300));
}

static void SetServoXOnTime(uint32_t onTimeUs)
{
    // TCC0_PWM24bitDutySet(TCC0_CHANNEL0, onTimeUs);
}


static void SetServoYOnTime(uint32_t onTimeUs)
{
    // TCC0_PWM24bitDutySet(TCC0_CHANNEL2, onTimeUs);
}


// helper function to map a value from one range to another
static long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}