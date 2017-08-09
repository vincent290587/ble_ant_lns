
#ifndef __ANT_C__
#define __ANT_C__

#define APP_TIMER_PRESCALER         0                                   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     5                                   /**< Size of timer operation queues. */


void ant_stack_init(void);

int ant_setup_start(void);

void ant_timers_init(void);

#endif
