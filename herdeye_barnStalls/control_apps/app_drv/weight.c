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
	Count=Count^0x800000;//���λȡ��������λ����
	                      //��HX71оƬ�У�count��һ��32λ���з������������ڴ洢���ش������Ķ�����
	                      //��count�����λΪ1ʱ����ʾ����Ϊ��������HX711оƬ��֧�ָ����Ķ�����
	                      //��ˣ�Ϊ�˽�����ת��Ϊ��������Ҫ��count�����λȡ��������count��0x800000������������
                          //������˵��0x800000�Ķ����Ʊ�ʾΪ100000000000000000000000����count������������
	                      //���Խ�count�����λ��1��Ϊ0���Ӷ��õ���Ӧ������������
	HX711_SCK_L;
	delay_us(1);
		
	return(Count);
}


void weight_reset_init(){
	reset = HX711_GetData();//��ʼ������У׼����
}

// ��ȡ��ǰ�����ĺ���
float get_current_weight(void)
{
    int32_t raw_value = HX711_GetData(); // ��ȡԭʼ����
    // ����������(��ǰֵ - ���ֵ) * ��׼���� / (��׼������ - ���ֵ)
    return (float)(raw_value - reset) * Weights / (float)(Weights_100 - reset);
}
