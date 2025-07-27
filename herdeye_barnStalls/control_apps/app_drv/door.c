#include "door.h"


void boor_init(){
	TIM_Init_Type tim_init;
    tim_init.ClockFreqHz = CLOCK_SYS_FREQ;
    tim_init.StepFreqHz = 1000000;
    tim_init.Period = 20000 - 1; /* the counter would return to the base on next step. */
    tim_init.EnablePreloadPeriod = true; /* no need preload, load period value immediately. */
    tim_init.PeriodMode = TIM_PeriodMode_Continuous;
    tim_init.CountMode = TIM_CountMode_Increasing;
    TIM_Init(DOOR_TIM_PORT, &tim_init);

    /* Setup the PWM output channel. */
    TIM_OutputCompareConf_Type tim_out_conf;
    tim_out_conf.ChannelValue = 0u;
    tim_out_conf.EnableFastOutput = false;
    tim_out_conf.EnablePreLoadChannelValue = true; /* disable preload, load channel value immediately. */
    tim_out_conf.RefOutMode = TIM_OutputCompareRefOut_FallingEdgeOnMatch;
    tim_out_conf.ClearRefOutOnExtTrigger = false;
    tim_out_conf.PinPolarity = TIM_PinPolarity_Rising;
	 
    TIM_EnableOutputCompare(DOOR_TIM_PORT, DOOR_TIM_CHANNEL, &tim_out_conf);

    /* Start the output compare, only available for the TIM peripheral with this feature. */
    TIM_EnableOutputCompareSwitch(DOOR_TIM_PORT, true);

    /* Start the counter. */
    TIM_Start(DOOR_TIM_PORT);
}

void door_Set_Angle(float Angle){
	// 计算脉冲宽度 (500~2500对应0.5ms~2.5ms)
    uint32_t pulseWidth = (uint32_t)(500 + (Angle * 2000 / 180));
    TIM_PutChannelValue(DOOR_TIM_PORT, DOOR_TIM_CHANNEL, pulseWidth);
}

void set_boor_flag(uint8_t flag){
	float  value = flag ? 0 : 180;
	door_Set_Angle(value);
}
