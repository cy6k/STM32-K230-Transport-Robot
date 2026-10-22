#ifndef __MOTOR_TD_H
#define __MOTOR_TD_H

#include "stm32f4xx.h"

/* _Td blocks in main until reached or error. TIM6/GetTick and IRQs must run.
 * Drivers: Emm V5 custom UART protocol, fixed check byte 0x6B.
 * A query reply is required from EACH addressed driver (PA3 RX).
 * Return MOTOR_TD_OK before executing the next operation.
 */
#define MOTOR_TD_TIMEOUT_MS         20000U
#define MOTOR_TD_START_TIMEOUT_MS     500U
#define MOTOR_TD_REPLY_TIMEOUT_MS      30U
#define MOTOR_TD_QUERY_RETRIES          3U
#define MOTOR_TD_POLL_MS               10U
#define MOTOR_TD_CONFIRM_COUNT          2U

typedef enum {
    MOTOR_TD_OK = 0,
    MOTOR_TD_COMM_ERROR,
    MOTOR_TD_TIMEOUT,
    MOTOR_TD_FAULT,             /* Disabled, stalled or stall protection. */
    MOTOR_TD_START_FAILED,      /* Nonzero command did not change target. */
    MOTOR_TD_ARGUMENT_ERROR
} Motor_Td_Result_t;

/* Shared implementation for Motor.c/Arm.c; call from main only, not ISR.
 * Bit 0..6 corresponds to driver 1..7. changedMask marks nonzero moves.
 * Prepare snapshots the old targets; Wait requires new targets and fresh
 * reached replies. Errors stop only mask's motors and retain their enable.
 */
Motor_Td_Result_t Motor_Td_Prepare(uint8_t mask, uint8_t changedMask);
Motor_Td_Result_t Motor_Td_Wait(void);

/* USART2 ISR hooks: queue bytes only; no query, delay or printing in ISR. */
void Motor_Td_RxByte(uint8_t byte);
void Motor_Td_RxError(void);

#endif
