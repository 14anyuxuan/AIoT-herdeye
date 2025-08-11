#include "ws2812.h"

volatile uint8_t ws2812b_disp_bit_buff[WS2812B_DISP_BUFF_LEN]; /* bit stream of colors, to be transferred by hardware. */
volatile uint8_t ws2812b_disp_byte_buff[WS2812B_LIST_LED_NUM * 3u]; /* byte stream of colors, to represent the colors. */
volatile bool ws2812b_spi_tx_dma_done = true;


void ws2812b_init(void)
{
    /* pins are ready in pin_init.c */

    /* dma. */
    DMA_Channel_Init_Type dma_channel_init;
    dma_channel_init.XferMode = DMA_XferMode_MemoryToPeriph;
    dma_channel_init.ReloadMode = DMA_ReloadMode_AutoReload; /* DMA_AutoReloadMode_Circular */
    dma_channel_init.PeriphAddrIncMode = DMA_AddrIncMode_StayAfterXfer;
    dma_channel_init.MemAddrIncMode = DMA_AddrIncMode_IncAfterXfer;
    dma_channel_init.XferWidth = DMA_XferWidth_8b;
    dma_channel_init.Priority = DMA_Priority_Low;
    dma_channel_init.XferCount = WS2812B_DISP_BUFF_LEN;
    dma_channel_init.MemAddr = (uint32_t)ws2812b_disp_bit_buff;
    dma_channel_init.PeriphAddr = SPI_GetTxDataRegAddr(WS2812B_SPI_TX_PORT); /* use tx data register here. */
    DMA_InitChannel(WS2812B_SPI_TX_DMA_PORT, WS2812B_SPI_TX_DMA_CHANNEL, &dma_channel_init);

    /* enable interrupt for DMA. */
    NVIC_EnableIRQ(WS2812B_SPI_TX_DMA_IRQ);
    DMA_EnableChannelInterrupts(WS2812B_SPI_TX_DMA_PORT, WS2812B_SPI_TX_DMA_CHANNEL, DMA_CHN_INT_XFER_DONE, true);

    /* spi3. */
    SPI_Master_Init_Type spi_init;
    spi_init.ClockFreqHz = WS2812B_SPI_TX_FREQ;
    spi_init.BaudRate = WS2812B_SPI_TX_BAUDRATE;
    spi_init.XferMode = SPI_XferMode_TxRx;
    spi_init.PolPha = SPI_PolPha_Alt0;
    spi_init.DataWidth = SPI_DataWidth_8b;
    spi_init.LSB = false;
    spi_init.AutoCS = true;
    SPI_InitMaster(WS2812B_SPI_TX_PORT, &spi_init);
    SPI_EnableDMA(WS2812B_SPI_TX_PORT, true); /* Events would trigger the DMA instead of interrupt. */
    SPI_Enable(WS2812B_SPI_TX_PORT, true);

    /* clear the led matrix buff. */
    uint8_t *p_buff = (uint8_t *)ws2812b_disp_bit_buff;
    for (uint32_t i = 0u; i < WS2812B_DISP_PIXEL_LEN * WS2812B_LIST_LED_NUM; i++, p_buff++)
    {
        *p_buff = WS2812B_BIT0_CODE;
    }

    ws2812b_spi_tx_dma_done = true;
}

/* from led_disp_byte_buff to led_disp_bit_buff */
void ws2812b_display_buff_update_all(void)
{
    uint8_t *p_buff1 = (uint8_t *)ws2812b_disp_byte_buff;
    uint8_t *p_buff2 = (uint8_t *)ws2812b_disp_bit_buff;

    for (uint32_t i1 = 0u; i1 < 3u * WS2812B_LIST_LED_NUM; i1++, p_buff1++)
    {
        for (uint32_t i2 = 0u; i2 < 8u; i2++, p_buff2++)
        {
            if ( 0u == (*p_buff1 & (1u << (7u - i2))) )
            {
                *p_buff2 = WS2812B_BIT0_CODE;
            }
            else
            {
                *p_buff2 = WS2812B_BIT1_CODE;
            }
        }
    }
}

void ws2812b_display_set_color(uint32_t color, uint32_t index)
{
    uint8_t *p_buff = (uint8_t *)ws2812b_disp_byte_buff + index * 3u;

    /* mark the brightness of each leds. */
    *p_buff++  = (uint8_t)((color & 0xFF0000) >> 16u);
    *p_buff++  = (uint8_t)((color & 0x00FF00) >> 8u );
    *p_buff    = (uint8_t)((color & 0x0000FF)       );

    /* update to the bit buff. */
    //ws2812b_display_buff_update_all();
}

void ws2812b_display_clear_color(void)
{
    for (uint32_t i = 0u; i < WS2812B_LIST_LED_NUM; i++)
    {
        ws2812b_display_set_color(0u, i);
    }
}

/* ISP for SPI_TX DMA done. */
void WS2812B_SPI_TX_DMA_IRQHandler(void)
{
    if (0u != (DMA_CHN_INT_XFER_DONE & DMA_GetChannelInterruptStatus(WS2812B_SPI_TX_DMA_PORT, WS2812B_SPI_TX_DMA_CHANNEL)) )
    {
        ws2812b_spi_tx_dma_done = true;
        DMA_ClearChannelInterruptStatus(WS2812B_SPI_TX_DMA_PORT, WS2812B_SPI_TX_DMA_CHANNEL, DMA_CHN_INT_XFER_DONE);

        /* The DMA channel would be disbaled automatically when the xfer is done. */
        //DMA_EnableChannel(BOARD_DMA_UART_TX_PORT, BOARD_DMA_UART_TX_CHANNEL, true); /* Enable the DMA channel for the next xfer. */
    }
}

void ws2812b_display_start(void)
{
    ws2812b_display_waitdone();
    ws2812b_spi_tx_dma_done = false;
    DMA_EnableChannel(WS2812B_SPI_TX_DMA_PORT, WS2812B_SPI_TX_DMA_CHANNEL, true);
}

void ws2812b_display_waitdone(void)
{
    while (!ws2812b_spi_tx_dma_done) /* wait while the dma is busy. */
    {}
}

/*============================================================================================================================================================*/

uint8_t brightness = BRIGHTNESS_MAX;  // 默认亮度值

// 设置亮度（0-255）
void set_brightness(uint8_t br)
{
    if (br > BRIGHTNESS_MAX)
    {
        br = BRIGHTNESS_MAX;
    }
    brightness = br;
}

// 根据亮度调整RGB值
void adjust_rgb_for_brightness(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = (*r * brightness) / BRIGHTNESS_MAX;
    *g = (*g * brightness) / BRIGHTNESS_MAX;
    *b = (*b * brightness) / BRIGHTNESS_MAX;
}

// 使用RGB值设置颜色（需要实现ws2812b_display_set_rgb函数）
void ws2812b_display_set_rgb(uint8_t r, uint8_t g, uint8_t b, uint32_t index)
{
    // 调整亮度
    adjust_rgb_for_brightness(&r, &g, &b);
    
    // 假设ws2812b_display_set_color函数接受一个颜色值
    // 需要将RGB转换为对应的颜色格式
    uint32_t color = (g << 16) | (r << 8) | b;
    ws2812b_display_set_color(color, index);
}

// 工作模式（静态）
void set_work_effect_nonblocking() {
    // 直接设置颜色即可，无需循环
    ws2812b_display_buff_update_all();
    ws2812b_display_start();
}

// 学习模式（状态机实现呼吸）
static uint8_t breathe_brightness = 0;
static bool breathe_direction = true;
void set_study_effect_nonblocking(uint8_t target_brightness) {
    // 更新亮度
    if(breathe_direction){
        if(++breathe_brightness >= target_brightness) breathe_direction = false;
    }else{
        if(--breathe_brightness <= (target_brightness/2)) breathe_direction = true;
    }
    
    set_brightness(breathe_brightness);
    ws2812b_display_buff_update_all();
    ws2812b_display_start();
}

// 睡眠模式（状态机实现深度呼吸）
static uint8_t sleep_brightness = 0;
static bool sleep_direction = false;
static uint8_t sleep_amplitude = BRIGHTNESS_MAX;
void set_sleep_effect_nonblocking() {
    // 更新亮度
    if(sleep_direction){
        if(++sleep_brightness >= sleep_amplitude){
            sleep_direction = false;
            sleep_amplitude *= 0.9;
            if(sleep_amplitude < 20) sleep_amplitude = 20;
        }
    }else{
        if(--sleep_brightness <= 10) sleep_direction = true;
    }
    
    set_brightness(sleep_brightness);
    ws2812b_display_buff_update_all();
    ws2812b_display_start();
}

void app_delay_ms(uint32_t t)
{
    for (uint32_t i = 0u; i < t; i++)
    {
        for (uint32_t j = 0u; j < 10000U; j++)
        {
            __NOP();
        }
    }
}




