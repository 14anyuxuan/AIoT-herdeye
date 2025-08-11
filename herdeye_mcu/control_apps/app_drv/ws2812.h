#ifndef __WS2812_H__
#define __WS2812_H__

#include "board_init.h"


#define WS2812B_LIST_LED_NUM   9u
#define WS2812B_DISP_PIXEL_LEN 24u /* 24bit.  */
#define WS2812B_DISP_BUFF_LEN  (WS2812B_DISP_PIXEL_LEN * WS2812B_LIST_LED_NUM)

#define WS2812B_SPI_TX_PORT            SPI3
#define WS2812B_SPI_TX_FREQ            CLOCK_APB1_FREQ
#define WS2812B_SPI_TX_BAUDRATE        6400000u  /* 6.4Mbps */

#define WS2812B_SPI_TX_DMA_PORT        DMA2
#define WS2812B_SPI_TX_DMA_CHANNEL     DMA_REQ_DMA2_SPI3_TX
#define WS2812B_SPI_TX_DMA_IRQ         DMA2_CH2_IRQn
#define WS2812B_SPI_TX_DMA_IRQHandler  DMA2_CH2_IRQHandler

#define WS2812B_BIT0_CODE              0xC0
#define WS2812B_BIT1_CODE              0xF0

#define WS2812B_COLOR_GREEN   0xFF0000
#define WS2812B_COLOR_RED     0xFF00
#define WS2812B_COLOR_BLUE    0xFF

#define BRIGHTNESS_MAX 255  // 最大亮度值
extern uint8_t brightness;  // 默认亮度值
extern volatile bool ws2812b_spi_tx_dma_done;




void ws2812b_init(void);
void ws2812b_display_set_color(uint32_t color, uint32_t index);
void ws2812b_display_clear_color(void);
void ws2812b_display_buff_update_all(void);
void ws2812b_display_start(void);
void ws2812b_display_waitdone(void);


void set_brightness(uint8_t br);
void adjust_rgb_for_brightness(uint8_t *r, uint8_t *g, uint8_t *b);
void ws2812b_display_set_rgb(uint8_t r, uint8_t g, uint8_t b, uint32_t index);
void app_delay_ms(uint32_t t);

// 工作模式 - 静态高亮效果
void set_work_effect_nonblocking();

// 学习模式 - 渐变呼吸效果（柔和过渡）
void set_study_effect_nonblocking(uint8_t target_brightness);

// 睡眠模式 - 深度呼吸效果（缓慢熄灭）
void set_sleep_effect_nonblocking();


// 工作模式 - 冷白光全亮
//set_work_effect(255, 255, 255, 200);

// 学习模式 - 暖黄光渐变
//set_study_effect(255, 180, 100, 150);

// 睡眠模式 - 暗红色呼吸
//set_sleep_effect(80, 0, 0, 50);

#endif
