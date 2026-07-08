#include "delay.h"
#include "ti_msp_dl_config.h"

static void Delay_Cycles(uint64_t cycles)
{
    while (cycles > 0U) {
        uint32_t chunk = (cycles > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t) cycles;
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

void Delay_us(uint32_t us)
{
    uint64_t cycles;

    if (us == 0U) return;

    cycles = ((uint64_t) CPUCLK_FREQ * (uint64_t) us) / 1000000ULL;
    if (cycles == 0U) cycles = 1U;
    Delay_Cycles(cycles);
}

void Delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        Delay_us(1000U);
    }
}

void delay_ms(int ms)
{
    if (ms > 0) {
        Delay_ms((uint32_t) ms);
    }
}
