#include <stdbool.h>
#include "notifications.h"


#define ON_STEPS_NB      5
#define ON_TICKS_DEFAULT 3


static neopixel_strip_t     strip;
static neo_sb_init_params_t _params;
static bool                 leds_is_on;     /**< Flag for indicating if LEDs are on. */
static bool                 is_counting_up; /**< Flag for indicating if counter is incrementing or decrementing. */
static int32_t              pause_ticks;
static uint32_t             ratio;


static void notifications_clear() {
	neopixel_clear(&strip);
}

static void notifications_show() {
	neopixel_show(&strip);
}

static uint8_t notifications_setColor(uint8_t red, uint8_t green, uint8_t blue ) {
	return neopixel_set_color(&strip, 0, red, green, blue );
}

/**
 *
 * @param pin_num
 */
void notifications_init(uint8_t pin_num) {

	_params.max = 10;
	_params.min = 0;

	ratio = 0;
	leds_is_on  = false;
	pause_ticks = 0;

	_params.rgb[0] = 0;
	_params.rgb[1] = 0;
	_params.rgb[2] = 0;

	_params.step = _params.max / ON_STEPS_NB;
	_params.on_time_ticks = ON_TICKS_DEFAULT;

	is_counting_up = true;

	neopixel_init(&strip, pin_num, 1);

}

/**
 *
 * @param red
 * @param green
 * @param blue
 * @param on_time
 */
void notifications_setNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time) {

	notifications_setNotify(red, green, blue, on_time);
	_params.on_time_ticks = on_time;

}

/**
 *
 * @param red
 * @param green
 * @param blue
 * @param on_time
 */
void notifications_setWeakNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time) {

	if (!leds_is_on) {
		notifications_setNotify(red, green, blue, on_time);
		_params.on_time_ticks = on_time;
	}

}

/**
 *
 */
void notifications_tasks() {

	if (!leds_is_on) {
		notifications_clear();
		return;
	}

	// continue process
	if (pause_ticks <= 0)
	{
		if (is_counting_up)
		{
			if ((int)(ratio) >= (int)(_params.max - _params.step))
			{
				// start decrementing.
				is_counting_up = false;
				pause_ticks = _params.on_time_ticks;
			}
			else
			{
				ratio += _params.step;
			}
		}
		else
		{
			if ((int)(ratio) <= (int)(_params.min + _params.step))
			{
				// Min is reached, we are done
				// end process
				leds_is_on = false;
				notifications_clear();
				return;
			}
			else
			{
				ratio -= _params.step;
			}
		}
	}
	else
	{
		pause_ticks -= 1;
	}

	// update neopixel
	notifications_setColor(_params.rgb[0] * ratio * ratio / 255,
			_params.rgb[1] * ratio * ratio / 255,
			_params.rgb[2] * ratio * ratio / 255);
	notifications_show();
}



