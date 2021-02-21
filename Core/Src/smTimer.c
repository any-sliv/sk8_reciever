#include "smTimer.h"
#include "stm32f1xx_hal.h"

//#define WDG_REFRESH()             IWDG_update()
#define SM_TIMER_GET_COUNTER()     HAL_GetTick()


void sm_timer_run(sm_stamp *event, uint32_t delayMs)
{
  if(NULL != event)
  {
    event->timestamp = (SM_TIMER_GET_COUNTER() + delayMs);
		event->isRunning = true;
  }
}


void sm_timer_deactivate(sm_stamp *event)
{
  if(NULL != event)
  {
    event->timestamp = 0;
		event->isRunning = false;
  }
}


void sm_timer_delay(uint32_t timeMs)
{
  sm_stamp myTimer = {0,false};
  sm_timer_run(&myTimer, timeMs);

  while(sm_timer_getState(&myTimer) == SM_TIMER_RUNNING)
  {
    //WDG_REFRESH();
  }
}


sm_timer_status sm_timer_getState(const sm_stamp *event)
{
	if (event != NULL)
	{
		if (!event->isRunning)
		{
			return SM_TIMER_STOP;
		}
		else
		{
			if(SM_TIMER_GET_COUNTER() > event->timestamp)
			{
				return SM_TIMER_TIMEOUT;
			}
			else
			{
				return SM_TIMER_RUNNING;
			}
		}		
	}
	return 0;
}
