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
void btStatusTask(void*);
void motorTask(void*);
void buzzerTask(void*);

TaskHandle_t mainTaskHandle = NULL;
QueueHandle_t btQueue = NULL;
SemaphoreHandle_t ackSemphr = NULL;

//void BLUETOOTH_IRQHandler()
//{
//	char charReceived;
//
//	if(kUART_RxDataRegFullFlag & UART_GetStatusFlags(BLUETOOTH_PERIPHERAL))
//	{
//		BaseType_t xHigherPriorityTaskWoken;
//		charReceived = UART_ReadByte(BLUETOOTH_PERIPHERAL);
//
//		xHigherPriorityTaskWoken = pdFALSE;
//		xQueueSendFromISR(btQueue, &charReceived, &xHigherPriorityTaskWoken);
//
//		GPIO_PortToggle(BOARD_GREEN_LED_GPIO, 1 << BOARD_GREEN_LED_PIN);
//		UART_WriteByte(BLUETOOTH_PERIPHERAL, charReceived);
//
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}

void PORTA_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken;
	GPIO_PortClearInterruptFlags(BOARD_ACKNOWLEDGE_GPIO, 1 << BOARD_ACKNOWLEDGE_PIN);

	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ackSemphr, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


    NVIC_SetPriority(PORTA_IRQn, 7);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
//    UART_EnableInterrupts(BLUETOOTH_PERIPHERAL, kUART_RxDataRegFullInterruptEnable);
//    NVIC_SetPriority(UART4_RX_TX_IRQn, 10);
//    NVIC_ClearPendingIRQ(UART4_RX_TX_IRQn);
//    NVIC_EnableIRQ(UART4_RX_TX_IRQn);

#ifdef DEBUG_MSG
    PRINTF("\033[32mApplication Start\033[0m\n\r");
#endif

    btQueue = xQueueCreate(5, 5);

    if(xTaskCreate(mainTask, "Main Task", configMINIMAL_STACK_SIZE + 20, NULL, 3, &mainTaskHandle) == pdFALSE)
    {
    	PRINTF("\n\r\033[31mMain Task creation failed\033[0m\n\r");
    }

    if(xTaskCreate(btStatusTask, "BT Status Task", configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL) == pdFALSE)
    {
    	PRINTF("\n\r\033[31mMain Task creation failed\033[0m\n\r");
    }

    ackSemphr = xSemaphoreCreateBinary();

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
		if(kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART4)) {
			uint8_t charReceived = UART_ReadByte(UART4);
			if(charReceived == '\001') {
				GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);
				PRINTF("\n\rAlarm");
				if(xTaskCreate(motorTask, "Motor task", configMINIMAL_STACK_SIZE, NULL, 4, NULL) == pdFALSE)
				{
					PRINTF("\n\rMotor Task creation failed");
				}
			}
			else if(charReceived == '\002') {
				PRINTF("\n\rBuzzer");
				if(xTaskCreate(buzzerTask, "Buzzer task", configMINIMAL_STACK_SIZE, NULL, 5, NULL) == pdFALSE)
				{
					PRINTF("\n\rBuzzer Task creation failed");
				}
			}
			charReceived = '0';
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void btStatusTask(void* pvParameters)
{
	for(;;)
	{
		if(GPIO_PinRead(BOARD_BT_STATE_GPIO, BOARD_BT_STATE_PIN) == 1)
			GPIO_PinWrite(BOARD_BLUE_LED_GPIO, BOARD_BLUE_LED_PIN, 0);
		else
			GPIO_PortToggle(BOARD_BLUE_LED_GPIO, 1 << BOARD_BLUE_LED_PIN);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void motorTask(void* pvParameters)
{
	for(;;)
	{
		GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);
		vTaskDelay(pdMS_TO_TICKS(200));

		if(xSemaphoreTake(ackSemphr, 0) == pdTRUE)
		{
			GPIO_PinWrite(BOARD_RED_LED_GPIO, BOARD_RED_LED_PIN, 1);
			vTaskDelete(NULL);
		}
	}
}

void buzzerTask(void* pvParameters)
{
	for(;;)
	{
		GPIO_PortToggle(BOARD_BUZZER_GPIO, 1 << BOARD_BUZZER_PIN);
		vTaskDelay(pdMS_TO_TICKS(5));

		if(xSemaphoreTake(ackSemphr, 0) == pdTRUE)
		{
			GPIO_PinWrite(BOARD_BUZZER_GPIO, BOARD_BUZZER_PIN, 1);
			vTaskDelete(NULL);
		}
	}
}
