/**
 * @file    Dev2_FreeRTOS.c
 * @brief   Application entry point.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#define DEBUG_MSG 1

void mainTask(void*);

TaskHandle_t mainTaskHandle = NULL;

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

#ifdef DEBUG_MSG
    PRINTF("\033[32mApplication Start\033[0m\n\r");
#endif

    if(xTaskCreate(mainTask, "Main Task", configMINIMAL_STACK_SIZE + 20, NULL, 3, &mainTaskHandle) == pdFALSE)
    {
    	PRINTF("\n\r\033[31mMain Task creation failed\033[0m\n\r");
    }

    vTaskStartScheduler();

    while(1) {}

    return 0 ;
}

void mainTask(void* pvParameters)
{
#ifdef DEBUG_MSG
	PRINTF("\n\rMain Task\n\r");
#endif

	for(;;)
	{
		GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
