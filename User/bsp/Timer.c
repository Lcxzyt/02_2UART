#include "Timer.h"
#include "ti_msp_dl_config.h"
#include "CmdDispatch.h"
#include "Encoder.h"
#include "Motor.h"

#define SPEED_FILTER_SHIFT 2U
#define MAIN_HEARTBEAT_TIMEOUT_TICKS 10U

static int32_t speed_filter_l;
static int32_t speed_filter_r;
static uint8_t speed_filter_ready;
static volatile uint8_t main_heartbeat_ticks;
static volatile uint8_t safety_stop_latched;

static int16_t Speed_FilterToInt(int32_t value)
{
    if (value >= 0) {
        return (int16_t)((value + (1 << (SPEED_FILTER_SHIFT - 1U))) >> SPEED_FILTER_SHIFT);
    }
    return (int16_t)(-((-value + (1 << (SPEED_FILTER_SHIFT - 1U))) >> SPEED_FILTER_SHIFT));
}

static int16_t Speed_FilterUpdate(int32_t *state, int16_t raw)
{
    int32_t target = (int32_t)raw << SPEED_FILTER_SHIFT;

    *state += (target - *state) >> SPEED_FILTER_SHIFT;
    return Speed_FilterToInt(*state);
}

void Timer_ResetSpeedFilter(void)
{
    speed_filter_l = 0;
    speed_filter_r = 0;
    speed_filter_ready = 0U;
    SpeedFiltL = 0;
    SpeedFiltR = 0;
}

void Timer_NotifyMainAlive(void)
{
    main_heartbeat_ticks = 0U;
}

uint8_t Timer_WasSafetyStop(void)
{
    return safety_stop_latched;
}

void Timer_Init(void)
{
    main_heartbeat_ticks = 0U;
    safety_stop_latched = 0U;
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            /* 控制周期里始终读编码器，停机时也能手拨轮子排查测速信号。 */
            SpeedL = Encoder_Get_L();
            SpeedR = Encoder_Get_R();
            if (!speed_filter_ready) {
                speed_filter_l = (int32_t)SpeedL << SPEED_FILTER_SHIFT;
                speed_filter_r = (int32_t)SpeedR << SPEED_FILTER_SHIFT;
                speed_filter_ready = 1U;
            }
            SpeedFiltL = Speed_FilterUpdate(&speed_filter_l, SpeedL);
            SpeedFiltR = Speed_FilterUpdate(&speed_filter_r, SpeedR);
            if (g_Run && (main_heartbeat_ticks >= MAIN_HEARTBEAT_TIMEOUT_TICKS)) {
                g_Run = 0U;
                safety_stop_latched = 1U;
                Motor_SetTarget_L(0);
                Motor_SetTarget_R(0);
                Motor_Control_Stop();
            } else if (g_Run) {
                main_heartbeat_ticks++;
                Motor_Control_Update(SpeedL, SpeedR);
            } else {
                main_heartbeat_ticks = 0U;
                Motor_Control_Stop();
            }
            if (g_SampleTicks < 10U) {
                g_SampleTicks++;
            }
            g_SampleReady = 1U;
            break;
        default:
            break;
    }
}
