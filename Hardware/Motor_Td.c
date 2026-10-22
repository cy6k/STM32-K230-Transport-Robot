#include "Motor_Td.h"
#include "Emm_V5.h"
#include "Timer.h"
#include "Delay.h"
#include <string.h>

#define TD_RX_SIZE 128U
#define TD_ENABLED 0x01U
#define TD_REACHED 0x02U
#define TD_FAULT   0x0CU

/* Dedicated queue: legacy rxCmd may be overwritten by command ACKs. */
static volatile uint8_t tdRx[TD_RX_SIZE];
static volatile uint8_t tdHead, tdTail, tdLost, tdReceiving;
static uint8_t tdMask, tdChangedMask;
static uint8_t tdOldTarget[8][5];

void Motor_Td_RxByte(uint8_t byte)
{
    uint8_t next;
    if (!tdReceiving) return;
    next = (uint8_t)((tdHead + 1U) % TD_RX_SIZE);
    if (next == tdTail)
    {
        tdLost = 1;
        return;
    }
    tdRx[tdHead] = byte;
    tdHead = next;
}

void Motor_Td_RxError(void)
{
    if (tdReceiving) tdLost = 1;
}

static void Td_ClearRx(void)
{
    /* Only USART2 is briefly masked; all waits keep interrupts enabled. */
    NVIC_DisableIRQ(USART2_IRQn);
    tdTail = tdHead;
    tdLost = 0;
    NVIC_EnableIRQ(USART2_IRQn);
}

static int16_t Td_ReadByte(void)
{
    uint8_t byte;
    if (tdTail == tdHead) return -1;
    byte = tdRx[tdTail];
    tdTail = (uint8_t)((tdTail + 1U) % TD_RX_SIZE);
    return byte;
}

/* Exactly one outstanding query. Match address, function, fixed length and
 * check byte. ACKs/arrival notifications/noise cannot overwrite its reply.
 * The five target bytes are sign + unsigned magnitude (0x33 response).
 */
static Motor_Td_Result_t Td_Query(uint8_t addr, uint8_t function, uint8_t *data)
{
    uint8_t frame[8], used, length, retry;
    uint32_t start;
    int16_t byte;

    length = (function == 0x33U) ? 8U : 4U;
    for (retry = 0; retry < MOTOR_TD_QUERY_RETRIES; retry++)
    {
        /* Let preceding control ACKs leave the bus before querying. */
        Td_ClearRx();
        Delay_ms(2);
        Td_ClearRx();
        used = 0;
        Emm_V5_Read_Sys_Params(addr, function == 0x33U ? S_TPOS : S_FLAG);
        start = GetTick();
        while ((uint32_t)(GetTick() - start) < MOTOR_TD_REPLY_TIMEOUT_MS)
        {
            if (tdLost) break; /* Entire attempt invalid after any lost byte. */
            while ((byte = Td_ReadByte()) >= 0)
            {
                if (tdLost) break;
                if (used == sizeof(frame))
                {
                    memmove(frame, frame + 1, sizeof(frame) - 1U);
                    used--;
                }
                frame[used++] = (uint8_t)byte;
                if (used >= length && frame[used - length] == addr &&
                    frame[used - length + 1U] == function &&
                    frame[used - 1U] == 0x6BU)
                {
                    if (function == 0x33U)
                    {
                        if (frame[used - length + 2U] > 1U) continue;
                        memcpy(data, frame + used - length + 2U, 5);
                        /* Normalize negative zero before target comparison. */
                        if (!(data[1] | data[2] | data[3] | data[4])) data[0] = 0;
                    }
                    else
                    {
                        /* V5.1.5+: bit 7 is the power-cycle flag, not a fault. */
                        data[0] = frame[used - length + 2U];
                    }
                    if (tdLost) break;
                    return MOTOR_TD_OK;
                }
            }
            Delay_ms(1);
        }
    }
    return MOTOR_TD_COMM_ERROR;
}

static Motor_Td_Result_t Td_Finish(Motor_Td_Result_t result)
{
    uint8_t id;
    tdReceiving = 0;
    if (result != MOTOR_TD_OK)
    {
        /* Never broadcast stop: that would also stop unrelated arm axes. */
        for (id = 1; id <= 7; id++)
        {
            if (tdMask & (1U << (id - 1U)))
            {
                Emm_V5_Stop_Now(id, false);
                Delay_ms(2);
            }
        }
    }
    tdMask = 0;
    return result;
}

Motor_Td_Result_t Motor_Td_Prepare(uint8_t mask, uint8_t changedMask)
{
    Motor_Td_Result_t result;
    uint8_t addr, bit, flags;

    /* Blocking waits require main context, enabled IRQs and Timer_Init(). */
    if (!mask || (mask & 0x80U) || (changedMask & (uint8_t)~mask) ||
        __get_IPSR() || __get_PRIMASK() || tdReceiving)
        return MOTOR_TD_ARGUMENT_ERROR;

    tdMask = mask;
    tdChangedMask = changedMask;
    tdReceiving = 1;
    for (addr = 1; addr <= 7; addr++)
    {
        bit = (uint8_t)(1U << (addr - 1U));
        if (!(mask & bit)) continue;
        result = Td_Query(addr, 0x3A, &flags);
        if (result != MOTOR_TD_OK) return Td_Finish(result);
        if (!(flags & TD_ENABLED) || (flags & TD_FAULT))
            return Td_Finish(MOTOR_TD_FAULT);
        if (changedMask & bit)
        {
            result = Td_Query(addr, 0x33, tdOldTarget[addr]);
            if (result != MOTOR_TD_OK) return Td_Finish(result);
        }
    }
    return MOTOR_TD_OK;
}

Motor_Td_Result_t Motor_Td_Wait(void)
{
    Motor_Td_Result_t result;
    uint32_t start = GetTick();
    uint8_t addr, bit, flags, allReached, confirms = 0;
    uint8_t pending = tdChangedMask;
    uint8_t target[5];

    if (!tdReceiving || !tdMask) return MOTOR_TD_ARGUMENT_ERROR;
    while (1)
    {
        allReached = 1;
        for (addr = 1; addr <= 7; addr++)
        {
            bit = (uint8_t)(1U << (addr - 1U));
            if (!(tdMask & bit)) continue;
            if ((uint32_t)(GetTick() - start) >= MOTOR_TD_TIMEOUT_MS)
                return Td_Finish(MOTOR_TD_TIMEOUT);

            /* New target is required even when a short move finishes before
             * the first status poll. Do not require observing a busy edge. */
            if (pending & bit)
            {
                result = Td_Query(addr, 0x33, target);
                if (result != MOTOR_TD_OK) return Td_Finish(result);
                if (memcmp(target, tdOldTarget[addr], sizeof(target)) != 0)
                    pending &= (uint8_t)~bit;
                else if ((uint32_t)(GetTick() - start) >= MOTOR_TD_START_TIMEOUT_MS)
                    return Td_Finish(MOTOR_TD_START_FAILED);
            }

            result = Td_Query(addr, 0x3A, &flags);
            if (result != MOTOR_TD_OK) return Td_Finish(result);
            if (!(flags & TD_ENABLED) || (flags & TD_FAULT))
                return Td_Finish(MOTOR_TD_FAULT);
            if ((pending & bit) || !(flags & TD_REACHED)) allReached = 0;
        }
        if ((uint32_t)(GetTick() - start) >= MOTOR_TD_TIMEOUT_MS)
            return Td_Finish(MOTOR_TD_TIMEOUT);
        /* Require two complete fresh sweeps; reset if any motor is busy. */
        if (allReached)
        {
            if (++confirms >= MOTOR_TD_CONFIRM_COUNT)
                return Td_Finish(MOTOR_TD_OK);
        }
        else confirms = 0;
        Delay_ms(MOTOR_TD_POLL_MS);
    }
}
