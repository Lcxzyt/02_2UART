#include "Timer.h"
#include "ti_msp_dl_config.h"
#include "CmdDispatch.h"
#include "Encoder.h"
#include "Motor.h"

void Timer_Init(void)
{
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
            if (g_Run) {
                Motor_Control_Update(SpeedL, SpeedR);
            } else {
                Motor_Control_Stop();
            }
            g_SampleReady = 1U;
            break;
        default:
            break;
    }
}