#include "stm32f10x.h"                  // Device header
#include "stm32f10x_can.h"              // Keil::Device:StdPeriph Drivers:CAN
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
static uint8_t Data[8]= {0};
static void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}
static void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
static void CAN_Config(){
	CAN_InitTypeDef CAN_init;
	CAN_init.CAN_ABOM = ENABLE;
	CAN_init.CAN_AWUM = ENABLE;
	CAN_init.CAN_NART = DISABLE;
	CAN_init.CAN_TTCM = DISABLE;
	CAN_init.CAN_TXFP = ENABLE;
	CAN_init.CAN_RFLM = DISABLE;
	
	CAN_init.CAN_Mode = CAN_Mode_Normal;
	CAN_init.CAN_Prescaler = 6;
	CAN_init.CAN_SJW = CAN_SJW_1tq;
	CAN_init.CAN_BS1 = CAN_BS1_6tq;
	CAN_init.CAN_BS2 = CAN_BS2_8tq;
	
	CAN_Init(CAN1, &CAN_init);
}
static void CAN_FilterConfig(){
	CAN_FilterInitTypeDef CAN_Fil;
	
	CAN_Fil.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_Fil.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_Fil.CAN_FilterNumber = 1;
	CAN_Fil.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_Fil.CAN_FilterIdHigh = 0x123<<5;
	CAN_Fil.CAN_FilterIdLow = 0;
	CAN_Fil.CAN_FilterMaskIdHigh = 0xFFE0;
	CAN_Fil.CAN_FilterMaskIdLow = 0;
	CAN_Fil.CAN_FilterActivation = ENABLE;
	
	CAN_FilterInit(&CAN_Fil);
}
void CAN_Receive_Data(uint8_t *data){
	while(CAN_MessagePending(CAN1,CAN_FIFO0)<1);
	CanRxMsg CANRx;
	
	CAN_Receive(CAN1, CAN_FIFO0, &CANRx);
	for(int i =0; i < CANRx.DLC; i++){
		data[i] = CANRx.Data[i];
	}
		CAN_FIFORelease(CAN1, CAN_FIFO0);

}
int main (){
	RCC_Config();
	GPIO_Config();
	CAN_Config();
	CAN_FilterConfig();
while(1)
{
	CAN_Receive_Data(Data);
}
}