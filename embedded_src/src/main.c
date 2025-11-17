#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

#define STEP PWM_USEC(100)

#define TOUCHSCREEN_CENTER (1615)

#define SERVO_MID_US (1439)
#define SERVO_STEP (SERVO_MID_US - 10)

#define ADC_NODE DT_NODELABEL(adc0)

enum direction {
	DOWN,
	UP,
};

//Servos
static const struct pwm_dt_spec servo_left = PWM_DT_SPEC_GET(DT_NODELABEL(servo_left));
static const struct pwm_dt_spec servo_right = PWM_DT_SPEC_GET(DT_NODELABEL(servo_right));
//Note - min and max are the same for both servos
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo_left), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo_left), max_pulse);

//ADC's
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

int main(void)
{
    uint32_t pulse_width_left = min_pulse;
    uint32_t pulse_width_right = max_pulse;
	enum direction dir = UP;
	int ret;

	printk("Amazing Balancing Ball!!!\r\n");

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

