#include "system_cm4.h"
#include "tremo_rcc.h"
#include "tremo_delay.h"
#include "tremo_regs.h"
#include "tremo_pwr.h"
#include "rtc-board.h"
#include "ASR_Arduino.h"
#include "uart.h"

void boardInitMcu( void )
{
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_AFEC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);

    //pinMode(1,ANALOG);
    //pinMode(6,ANALOG);
    //pinMode(7,ANALOG);
	
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,HIGH);
    pinMode(VBAT_ADC_CTL, OUTPUT);
    digitalWrite(VBAT_ADC_CTL,HIGH);
    systime = 0;
}


void nvic_init()
{
	NVIC_SetPriorityGrouping(0x00000003U);

    //NVIC_SetPriority(PendSV_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    
    //for(int i=0; i<=IWDG_IRQn; i++)
        //NVIC_SetPriority(i, configLIBRARY_NORMAL_INTERRUPT_PRIORITY);
}

void system_init(void)
{
    rcc_set_sys_clk_source(RCC_SYS_CLK_SOURCE_RCO48M);
    // FPU enable
#if (__FPU_PRESENT == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif

    // enable afec clk
    TREMO_REG_EN(RCC->CGR0, RCC_CGR0_AFEC_CLK_EN_MASK, true);

    // set flash read number to 1
    EFC->TIMING_CFG  =  (EFC->TIMING_CFG & (~0xF0000)) | (1<<16);
    while(!(EFC->SR&0x2));

    nvic_init();
    delay_init();
}

int boardIrqIsDisabled = false;
bool BoardDisableIrq( void )
{
   if(boardIrqIsDisabled==false)
   {
       boardIrqIsDisabled = true;
       return true;
   }
   else
      return false;
}

void BoardEnableIrq( bool disabledhere)
{
    if(disabledhere)
    {
       boardIrqIsDisabled = false;
    }
}

uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t GetTickCount(void)
{
	uint32_t m = systime;
	return m;
}

unsigned long micros( void )
{
    uint32_t ticks, ticks2;
    uint32_t pend, pend2;
    uint32_t count, count2;

    ticks2  = SysTick->VAL;
    pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
    count2  = GetTickCount();

    do {
        ticks=ticks2;
        pend=pend2;
        count=count2;
        ticks2  = SysTick->VAL;
        pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
        count2  = GetTickCount();
    } while ((pend != pend2) || (count != count2) || (ticks < ticks2));

    return ((count+pend) * 1000) + (((SysTick->LOAD  - ticks)*(1048576/(F_CPU/1000000)))>>20) ; 
    // this is an optimization to turn a runtime division into two compile-time divisions and 
    // a runtime multiplication and shift, saving a few cycles
}
