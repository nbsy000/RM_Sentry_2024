#include "main.h"
#include "task_StateLED.h"

/**
 *  @brief  ��״̬��
 *  @param  ��
 *  @retval ��
 */
void task_StateLED(void)
{
    while(1)
    {  
        BLUE_light_toggle();
        vTaskDelay(500);         
        RED_light_toggle();
        vTaskDelay(500);
    }
}
