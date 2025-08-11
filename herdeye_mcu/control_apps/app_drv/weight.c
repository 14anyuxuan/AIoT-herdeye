#include "weight.h"

int value;
float weight;

static int32_t reset;
uint8_t buff[30];

float Weights = 100.0f;  // 100g
int32_t Weights_100 = 8493860;  // 100g calibration value

unsigned long HX711_GetData(void)
{
	unsigned long Count;
	unsigned char i;
	HX711_SCK_L;
	delay_us(1);
	Count=0;
	while(HX711_DT);
	for (i=0;i<24;i++)
	{
		HX711_SCK_H;
		delay_us(1);
		Count=Count<<1;
		HX711_SCK_L;
		delay_us(1);
		if(HX711_DT) Count++;
	}
	HX711_SCK_H;
	delay_us(1);
	Count=Count^0x800000;//最高位取反，其他位不变
	                      //在HX71芯片中，count是一个32位的有符号整数，用于存储称重传感器的读数。
	                      //当count的最高位为1时，表示读数为负数，而HX711芯片不支持负数的读数。
	                      //因此，为了将负数转换为正数，需要将count的最高位取反，即将count与0x800000进行异或操作。
                          //具体来说，0x800000的二进制表示为100000000000000000000000，与count进行异或操作后，
	                      //可以将count的最高位从1变为0，从而得到对应的正数读数。
	HX711_SCK_L;
	delay_us(1);
		
	return(Count);
}


void weight_reset_init(){
	reset = HX711_GetData();//初始化重量校准数据
}

// 获取当前重量的函数
float get_current_weight(void)
{
    int32_t raw_value = HX711_GetData(); // 读取原始数据
    // 计算重量：(当前值 - 零点值) * 标准重量 / (标准重量点 - 零点值)
    return (float)(raw_value - reset) * Weights / (float)(Weights_100 - reset);
}
