
#ifndef  SMTIMER_H
#define  SMTIMER_H

/*State machine timer */

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
  SM_TIMER_STOP = 0,
  SM_TIMER_RUNNING,
  SM_TIMER_TIMEOUT
}sm_timer_status;	/* Typedef holding status of sm timer */


typedef struct{
	uint32_t timestamp;
	bool isRunning;
}sm_stamp;


void sm_timer_run(sm_stamp *event, uint32_t delayMs);

void sm_timer_deactivate(sm_stamp *event);


/** @brief Wait @ref timeMs ms, function refresh watchdog*/
void sm_timer_delay(uint32_t timeMs);


/** @brief Check the timer status
		@return SM_TIMER_ERROR, pointer error
						SM_TIMER_STOP - event is deactivated
						SM_TIMER_WORKING - event not occurred yet, timer still running
						SM_TIMER_TIMEOUT - time event occurred, timeout*/
sm_timer_status sm_timer_getState(const sm_stamp *event);

#endif /* SMTIMER_H */
