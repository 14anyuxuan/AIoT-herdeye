#include "brighetness.h"


/* setup the adc converter, and prepare a regular conversion sequence with only one channel in. */
void brighetness_init(void)
{
    /* pins and clock are already in the pin_init.c and clock_init.c. */

    /* setup the converter. */
    ADC_Init_Type adc_init;
    adc_init.Resolution = ADC_Resolution_Alt0;
    adc_init.ConvMode = ADC_ConvMode_SingleSlot;
    adc_init.Align = ADC_Align_Right;
    adc_init.SingleDiffMode = ADC_SingleDiffConvMode_SingleEnd; /* single-ended channel conversion mode. */
    adc_init.SingleVolt = ADC_SingleConvVref_Internal;  /* internal reference voltage as the reference voltage for single-ended conversion. */
    ADC_Init(BRIGHETNESS_ADC_PORT, &adc_init);

    ADC_Enable(BRIGHETNESS_ADC_PORT, true); /* power on the converter. */

    /* setup the sequence, a fixed channel 0 conversion. */
    ADC_EnableSeqSlotFixed(BRIGHETNESS_ADC_PORT, BRIGHETNESS_ADC_SLOT_MASK, ADC_SeqFixedDirection_LowFirst);

    /* set channel sample time. */
    ADC_SetChnSampleTime(BRIGHETNESS_ADC_PORT, BRIGHETNESS_ADC_CHN_NUM, ADC_SampleTime_Alt7);
}

/* software tirgger the adc converter and start the sequence conversion. */
uint32_t brighetness_adc_run_conv(void)
{
    uint32_t data;
    uint32_t flags;
    uint32_t adc_channel; /* keep the actual hardware conversion channel number. */

    /* software tirgger the conversion. */
    ADC_DoSwTrigger(BRIGHETNESS_ADC_PORT, true);

    /* wait while the conversion is ongoing. */
    while( 0u == (ADC_GetStatus(BRIGHETNESS_ADC_PORT) & ADC_STATUS_CONV_SEQ_DONE) )
    {}

    ADC_ClearStatus(BRIGHETNESS_ADC_PORT, ADC_STATUS_CONV_SEQ_DONE);

    data = ADC_GetConvResult(BRIGHETNESS_ADC_PORT, &adc_channel, &flags);

    if (0u == (flags & ADC_CONV_RESULT_FLAG_VALID) )
    {
        data = 0u; /* the data is unavailable when the VALID flag is not on. */
    }

    return data;
}

uint8_t brighetness_data(){
	
}



