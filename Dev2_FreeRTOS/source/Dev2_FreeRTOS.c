/**
 * @file    Dev2_FreeRTOS.c
 * @brief   Application entry point.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

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
QueueHandle_t btQueue = NULL;

void BLUETOOTH_IRQHandler()
{
	char charReceived;

	if(kUART_RxDataRegFullFlag & UART_GetStatusFlags(BLUETOOTH_PERIPHERAL))
	{
		BaseType_t xHigherPriorityTaskWoken;
		charReceived = UART_ReadByte(BLUETOOTH_PERIPHERAL);

		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(btQueue, &charReceived, &xHigherPriorityTaskWoken);

		GPIO_PortToggle(BOARD_GREEN_LED_GPIO, 1 << BOARD_GREEN_LED_PIN);
		UART_WriteByte(BLUETOOTH_PERIPHERAL, charReceived);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    UART_EnableInterrupts(BLUETOOTH_PERIPHERAL, kUART_RxDataRegFullInterruptEnable);
    NVIC_SetPriority(UART4_RX_TX_IRQn, 10);
    NVIC_ClearPendingIRQ(UART4_RX_TX_IRQn);
    NVIC_EnableIRQ(UART4_RX_TX_IRQn);

#ifdef DEBUG_MSG
    PRINTF("\033[32mApplication Start\033[0m\n\r");
#endif

    btQueue = xQueueCreate(5, 5);

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

	char charReceived = 'a';

	for(;;)
	{
		GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);

		if(xQueueReceive(btQueue, &charReceived, 0) == pdTRUE)
			PRINTF("\n\r%c", charReceived);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
