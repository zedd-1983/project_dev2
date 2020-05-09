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
#define SCHEDULED_WAKE_UP '\001'
#define INTENSIVE_WAKE_UP '\002'
#define ACK_FROM_PHONE 'a'
#define ACK_FROM_DEVICE 'b'

void dev2MainTask(void*);
void btStatusTask(void*);
void motorTask(void*);
void buzzerTask(void*);

TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t buzzerTaskHandle = NULL;
QueueHandle_t btQueue = NULL;
SemaphoreHandle_t ackSemphr = NULL;
SemaphoreHandle_t countDownReachedSemphr = NULL;
//uint8_t timeout = 0;
bool motorTaskIsRunning = false;
bool buzzerTaskIsRunning = false;


void PIT_1_0_IRQHANDLER()
{
	static uint8_t timeout = 0;
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	timeout++;
	PRINTF("\n\r%d", timeout);

	if(timeout > 10) {
		if(motorTaskIsRunning) {
			// send semaphore to main task to start the  buzzerTask
			timeout = 0;
			PRINTF("\n\rGiving semaphore");
			xSemaphoreGiveFromISR(countDownReachedSemphr, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		timeout = 0;
	}
}
/// @brief PORTA IRQ handler (ACK button)
/// @details external interrupt handler, gives semaphore to motorTask and/or
/// buzzer task
void PORTA_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken;
	GPIO_PortClearInterruptFlags(BOARD_ACKNOWLEDGE_GPIO, 1 << BOARD_ACKNOWLEDGE_PIN);

	// temporary
	//GPIO_PortToggle(BOARD_MOTOR_GPIO, 1 << BOARD_MOTOR_PIN);

	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ackSemphr, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/// @brief Main function
/// @details initializes development board hardware and configures priority of
/// external interrupt. It proceeds to create two tasks - mainTask and btTask and
/// creates a semaphore to be used between PORTA_IRQHandler and motorTask/buzzerTask
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

    if(xTaskCreate(dev2MainTask, "Main Task", configMINIMAL_STACK_SIZE + 20, NULL, 3, &mainTaskHandle) == pdFALSE)
    {
    	PRINTF("\n\r\033[31mMain Task creation failed\033[0m\n\r");
    }

    if(xTaskCreate(btStatusTask, "BT Status Task", configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL) == pdFALSE)
    {
    	PRINTF("\n\r\033[31mMain Task creation failed\033[0m\n\r");
    }

    ackSemphr = xSemaphoreCreateBinary();
    countDownReachedSemphr = xSemaphoreCreateBinary();

    vTaskStartScheduler();

    while(1) {}

    return 0 ;
}

/// @brief Main task
/// @details first task created in main. Based on data received from Dev1 it creates
/// either a motorTask or buzzerTask (or both if alarm-time catches up)
void dev2MainTask(void* pvParameters)
{
#ifdef DEBUG_MSG
	PRINTF("\n\rMain Task\n\r");
#endif

	for(;;)
	{
		if(kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART4)) {
			uint8_t charReceived = UART_ReadByte(UART4);
			PRINTF("\n\rValue received: %d", charReceived);
			if(charReceived == SCHEDULED_WAKE_UP) {
				charReceived = 0;
				GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);
				PRINTF("\n\rAlarm");
				if(xTaskCreate(motorTask, "Motor task", configMINIMAL_STACK_SIZE + 100, NULL, 4, &motorTaskHandle) == pdFALSE)
				{
					PRINTF("\n\rMotor Task creation failed");
				}
			}
			else if((charReceived == INTENSIVE_WAKE_UP)) {
				charReceived = 0;
				PRINTF("\n\rBuzzer");
				if(xTaskCreate(buzzerTask, "Buzzer task", configMINIMAL_STACK_SIZE + 100, NULL, 5, &buzzerTaskHandle) == pdFALSE)
				{
					PRINTF("\n\rBuzzer Task creation failed");
				}
			}
			else if(charReceived == ACK_FROM_PHONE)
					//((eTaskGetState(motorTaskHandle) == eReady) || (eTaskGetState(buzzerTaskHandle) == eReady)))
			{
				charReceived = 0;
				PRINTF("\n\rACKNOWLEDGED from PHONE");
				// give semaphore to stop ongoing scheduled and/or intensive wake_up
				xSemaphoreGive(ackSemphr);

			}
//			else if(charReceived == 'X')
//			{
//				charReceived = 0;
//				FTM_StartTimer(BUZZER_FTM_PERIPHERAL, BUZZER_FTM_CLOCK_SOURCE);
//				PRINTF("\n\rBuzzer");
//				if(xTaskCreate(buzzerTask, "Buzzer task", configMINIMAL_STACK_SIZE, NULL, 5, NULL) == pdFALSE)
//				{
//					PRINTF("\n\rBuzzer Task creation failed");
//				}
//			}

			//charReceived = '0';
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/// @brief Bluetooth status task
/// @details checks connectivity status of the bluetooth module by monitoring
/// its state pin (connected to PTC16), if connected Blue LED is turned on constantly,
/// otherwise it just flashes
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

/// @brief Vibration motor task
/// @details if created, controls a vibration motor (at the moments simulated by
/// toggling a RED LED)
/// This method will need to be adjusted to take advantage of PWM in the future
void motorTask(void* pvParameters)
{
	motorTaskIsRunning = true;
	PRINTF("\n\rMotor (scheduled wake-up) task created");
	GPIO_PinWrite(BOARD_MOTOR_GPIO, BOARD_MOTOR_PIN, 1);
	// start PIT to enable countdown to buzzerTask
	PIT_StartTimer(PIT, kPIT_Chnl_0);

	for(;;)
	{
		GPIO_PortToggle(BOARD_RED_LED_GPIO, 1 << BOARD_RED_LED_PIN);
		if(xSemaphoreTake(countDownReachedSemphr, 0) == pdTRUE) {
			// start the buzzer task
			PRINTF("\n\rCountDown semphr received in motorTask");
			PRINTF("\n\rBuzzer");
			if(xTaskCreate(buzzerTask, "Buzzer task", configMINIMAL_STACK_SIZE + 100, NULL, 5, &buzzerTaskHandle) == pdFALSE)
			{
				PRINTF("\n\rBuzzer Task creation failed");
			}
			GPIO_PinWrite(BOARD_RED_LED_GPIO, BOARD_RED_LED_PIN, 1);
			GPIO_PinWrite(BOARD_MOTOR_GPIO, BOARD_MOTOR_PIN, 0);
			// stop countdown when acknowledged
			PIT_StopTimer(PIT, kPIT_Chnl_0);
			motorTaskIsRunning = false;
			PRINTF("\n\rMotor task deleted after PIT semaphore");
			vTaskDelete(NULL);
		}

		if(xSemaphoreTake(ackSemphr, 0) == pdTRUE)
		{
			GPIO_PinWrite(BOARD_RED_LED_GPIO, BOARD_RED_LED_PIN, 1);
			GPIO_PinWrite(BOARD_MOTOR_GPIO, BOARD_MOTOR_PIN, 0);
			// stop countdown when acknowledged
			PIT_StopTimer(PIT, kPIT_Chnl_0);
			motorTaskIsRunning = false;
			PRINTF("\n\rMotor task deleted after acknowledgement");
			vTaskDelete(NULL);
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

/// @brief Buzzer task
/// @details if created, controls a buzzer connected to PTC17
/// if an acknowledgment is received, stops the FTM (buzzer)
void buzzerTask(void* pvParameters)
{
	buzzerTaskIsRunning = true;
	PRINTF("\n\rBuzzer (intensive wake-up) task created");
	FTM_StartTimer(BUZZER_FTM_PERIPHERAL, BUZZER_FTM_CLOCK_SOURCE);
	// stop timer when buzzer task starts
	PIT_StopTimer(PIT, kPIT_Chnl_0);

	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));

		if(xSemaphoreTake(ackSemphr, 0) == pdTRUE)
		{
			FTM_StopTimer(BUZZER_FTM_PERIPHERAL);
			PRINTF("\n\rBuzzer task deleted");
			buzzerTaskIsRunning = false;
			vTaskDelete(NULL);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
