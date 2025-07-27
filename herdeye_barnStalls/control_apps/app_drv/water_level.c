#include "water_level.h"


static  ADC_Type    *adc_index[2]       = {ADC1, ADC2};                         // 模块索引数组
static  uint8_t       adc_resolution[2]   = {ADC_12BIT, ADC_12BIT};               // 精度数据备份

// 定义5秒定时器参数
#define TIMER_INTERVAL_MS   5000  // 5秒
#define SYSTEM_CLOCK_FREQ   CLOCK_SYS_FREQ  // 系统时钟频率
#define TIMER_PRESCALER     4800  // 分频系数

volatile bool send_water_data = false; // 发送标志

// 计算定时器周期值
#define TIMER_PERIOD ((SYSTEM_CLOCK_FREQ / TIMER_PRESCALER) * (TIMER_INTERVAL_MS / 1000) - 1)

void water_level_init(){
//	ADC_Init_Type adc_init;
//    adc_init.Resolution = ADC_Resolution_Alt0;
//    adc_init.ConvMode = ADC_ConvMode_SingleSlot;
//    adc_init.Align = ADC_Align_Right;
//    ADC_Init(ADC1, &adc_init);

//    ADC_Enable(ADC1, true); /* power on the converter. */
//	ADC_DoAutoCalib(ADC1);
	
	
	TIM_Init_Type tim_init;
    tim_init.ClockFreqHz = CLOCK_SYS_FREQ;
    tim_init.StepFreqHz = SYSTEM_CLOCK_FREQ / TIMER_PRESCALER,  // 分频后频率
    tim_init.Period = TIMER_PERIOD,  // 计算得到的周期值
    tim_init.EnablePreloadPeriod = false;
    tim_init.PeriodMode = TIM_PeriodMode_Continuous;
    tim_init.CountMode = TIM_CountMode_Increasing;
    TIM_Init((TIM_Type *)TIM8, &tim_init);

    /* Enable interrupt. */
	NVIC_SetPriority(TIM8_UP_IRQn, 1); 
	NVIC_EnableIRQ(TIM8_UP_IRQn);
    TIM_EnableInterrupts((TIM_Type *)TIM8, TIM_STATUS_UPDATE_PERIOD, true);

    /* Start the counter. */
    TIM_Start((TIM_Type *)TIM8);
	
	
	printf("start\r\n");
	
	
}

uint16_t adc_convert (adc_channel_enum ch)
{
    uint32_t flag = ((ch & 0xFF0) >> 4);                                 
    uint32_t data = 0;                                                    

    ADC_EnableSeqSlot(adc_index[(ch & 0x0F)], 0, flag);
    ADC_DoSwTrigger(adc_index[(ch & 0x0F)], true);                           
    while((ADC_STATUS_CONV_SLOT_DONE | ADC_STATUS_CONV_SAMPLE_DONE) !=
        (ADC_GetStatus(adc_index[(ch & 0x0F)]) & (ADC_STATUS_CONV_SLOT_DONE | ADC_STATUS_CONV_SAMPLE_DONE)));
    ADC_ClearStatus(adc_index[(ch & 0x0F)], (ADC_STATUS_CONV_SLOT_DONE | ADC_STATUS_CONV_SAMPLE_DONE));
    data = ADC_GetSlotConvResult(adc_index[(ch & 0x0F)], 0, &flag);        
    return (data >> adc_resolution[(ch & 0x0F)]);                             
}

uint16_t water_level_data(){
//	return adc_convert(ADC1_CH4_A4);
	return GPIO_ReadInDataBit(GPIOA, GPIO_PIN_4);
}

void send_sensors_data(){
	if(send_water_data) {
        send_water_data = false;
        
        uint16_t level = water_level_data();
        float weight_val = get_current_weight();
        
        char json_buffer[64];
        int len = snprintf(json_buffer, sizeof(json_buffer),
                          "{\"water_level\":%u,\"weight\":%.1f}", 
                          level, weight_val);
        
        if(len > 0 && (size_t)len < sizeof(json_buffer)) {
            lora_uart_putstr((uint8_t *)json_buffer);
            lora_uart_putstr((uint8_t *)"\r\n");
            printf("LoRa Sent: %s\r\n", json_buffer);
        }
    }
}

void TIM8_UP_IRQHandler(){
	uint32_t flags = TIM_GetInterruptStatus((TIM_Type *)TIM8);
    if ( 0u != ( flags & TIM_STATUS_UPDATE_PERIOD ) ) /* Channel capture event. */
    {
		send_water_data = true;
		
    }
	TIM_ClearInterruptStatus((TIM_Type *)TIM8, TIM_STATUS_UPDATE_PERIOD);
}