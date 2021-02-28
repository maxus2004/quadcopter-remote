#include <stdbool.h>

#include "radio.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "string.h"

struct Telemetry
{
	float yaw, pitch, roll, thrust, altitude;
};
struct Telemetry telemetry;

bool altHold = false;
bool prevAltHoldBtn = true;

volatile bool gotResponse = false;

int packetLoss = 100;
int packetsLost = 0;
int packetsSent = 0;
int packetsPerSecond = 0;

uint32_t prevPacketLossCalculation = 0;

bool ADC_DONE = false;
uint16_t ADC_Data[4];

volatile uint32_t time = 0;


void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	}
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	//1000 ticks per second (every millisecond)
	LL_InitTick(72000000, 1000);
	LL_SYSTICK_EnableIT();
	LL_SetSystemCoreClock(72000000);
}

void SysTick_Handler(void)
{
	time++;
}

void controls_GPIO_init(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	//leds
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//digital inputs
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void init_ADC(void)
{
	//ADC
	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = { 0 };
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_13CYCLES_5);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_13CYCLES_5);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_13CYCLES_5);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_3);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_13CYCLES_5);

	LL_ADC_Enable(ADC1);
	LL_mDelay(10);

	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {}

	//DMA
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_ConfigTransfer(DMA1,
		LL_DMA_CHANNEL_1,
		LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
		LL_DMA_MODE_CIRCULAR |
		LL_DMA_PERIPH_NOINCREMENT |
		LL_DMA_MEMORY_INCREMENT |
		LL_DMA_PDATAALIGN_HALFWORD |
		LL_DMA_MDATAALIGN_HALFWORD |
		LL_DMA_PRIORITY_HIGH);
	LL_DMA_ConfigAddresses(DMA1,
		LL_DMA_CHANNEL_1,
		LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		(uint32_t)&ADC_Data,
		LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void DMA1_Channel1_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
	{
		ADC_DONE = true;
		LL_DMA_ClearFlag_TC1(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
	{
		LL_DMA_ClearFlag_TE1(DMA1);
	}
}


void processData(uint8_t* data, uint8_t length)
{
	if (data[0] != 77)return;

	gotResponse = true;
	memcpy(&telemetry, data + 1, length - 1);

}

void sendData(float* joysticks, bool altitudeHold)
{
	uint8_t data[18] = { 0 };
	data[0] = 66;
	memcpy(data + 1, joysticks, 16);
	data[17] = altitudeHold;
	radio_send(data, 18);
	gotResponse = false;
}

void EXTI9_5_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
	{
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
		//read radio interrupt register
		uint8_t status = radio_readReg(0x03);
		radio_readReg(0x04);

		//packet sent
		if (status & 1 << 2)
		{
			radio_startReceiving();
		}
		//packet received
		if (status & 1 << 1)
		{
			uint8_t buffer[64];
			uint8_t length = radio_receive(buffer);
			processData(buffer, length);
		}
		//crc error
		if (status & 1 << 0)
		{
			radio_writeReg(0x08, 0x02);
			radio_writeReg(0x08, 0x00);
			radio_startReceiving();
		}
	}
}

int main(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
	NVIC_SetPriorityGrouping(5);
	LL_GPIO_AF_Remap_SWJ_NOJTAG();
	SystemClock_Config();

	LL_mDelay(100);

	controls_GPIO_init();
	init_ADC();

	radio_setup();

	for (;;)
	{
		//calculate packet loss
		if (!gotResponse)
		{
			packetsLost++;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
		}else
		{
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
		}

		if (packetsSent > 0 && time - prevPacketLossCalculation > 1000)
		{
			prevPacketLossCalculation = time;
			packetLoss = packetsLost * 100 / packetsSent;
			packetsPerSecond = packetsSent;
			packetsLost = 0;
			packetsSent = 0;
		}


		LL_ADC_REG_StartConversionSWStart(ADC1);
		while (!ADC_DONE) {}
		ADC_DONE = false;
		float joysticks[4];
		for (int i = 0; i < 4; i++)
		{
			joysticks[i] = (float)ADC_Data[i] / 2048.0f - 1;
		}

		bool altHoldBtn = LL_GPIO_ReadInputPort(GPIOA) & 1 << 4;
		if (!altHoldBtn && prevAltHoldBtn)
		{
			altHold = !altHold;
			if (altHold)
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
			else
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
		}
		prevAltHoldBtn = altHoldBtn;

		sendData(joysticks, altHold);

		packetsSent++;
		LL_mDelay(10);
	}
}
