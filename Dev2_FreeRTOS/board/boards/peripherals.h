/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_uart.h"
#include "fsl_clock.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define BLUETOOTH_PERIPHERAL UART4
/* Definition of the clock source frequency */
#define BLUETOOTH_CLOCK_SOURCE CLOCK_GetFreq(UART4_CLK_SRC)
/* BLUETOOTH interrupt vector ID (number). */
#define BLUETOOTH_SERIAL_RX_TX_IRQN UART4_RX_TX_IRQn
/* BLUETOOTH interrupt handler identifier. */
#define BLUETOOTH_IRQHandler UART4_RX_TX_IRQHandler
/* BLUETOOTH interrupt vector ID (number). */
#define BLUETOOTH_SERIAL_ERROR_IRQN UART4_ERR_IRQn
/* BLUETOOTH interrupt handler identifier. */
#define BLUETOOTH_SERIAL_ERROR_IRQHANDLER UART4_ERR_IRQHandler

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const uart_config_t BLUETOOTH_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
