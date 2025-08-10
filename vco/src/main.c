#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define ADC_BUFFERSIZE 2

ADC_HandleTypeDef adcHandle;
TIM_HandleTypeDef timHandle;
TIM_HandleTypeDef timAdcHandle;

uint16_t adcValue[ADC_BUFFERSIZE];

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void ADC_Config(void);
void TIM_Config(void);
void Error_Handler(void);


// Quick and dirty delay
static void delay (unsigned int time) {
    for (unsigned int i = 0; i < time; i++)
        for (volatile unsigned int j = 0; j < 2000; j++);
}

/**
  * @brief  System clock configuration:
  *             System clock source = PLL (HSE)
  *             SYSCLK(Hz)          = 72000000
  *             HCLK(Hz)            = 72000000
  *             AHB prescaler       = 1
  *             APB1 prescaler      = 2
  *             APB2 prescaler      = 1
  *             HSE frequency(Hz)   = 8000000
  *             HSE PREDIV1         = 1
  *             PLLMUL              = 9
  *             Flash latency(WS)   = 2
  * @param  None
  * @retval None
  */
void RCC_SystemClock_Config(void)
{
	RCC_ClkInitTypeDef rccClkInit;
	RCC_OscInitTypeDef rccOscInit;

	/*## STEP 1: Configure HSE and PLL #######################################*/
	rccOscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rccOscInit.HSEState       = RCC_HSE_ON;
	rccOscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	rccOscInit.PLL.PLLState   = RCC_PLL_ON;
	rccOscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	rccOscInit.PLL.PLLMUL     = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&rccOscInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Configure SYSCLK, HCLK, PCLK1, and PCLK2 ####################*/
	rccClkInit.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	rccClkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	rccClkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	rccClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	rccClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&rccClkInit, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  GPIO configuration:
  *             GPIO  = GPIOB
  *             Pin   = PB6, PB7, PB8, PB9
  *             Mode  = Output push-pull
  *             Speed = Low
  * @param  None
  * @retval None
  */
void GPIO_Output_Config(void)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	gpioInit.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpioInit);
}

/**
  * @brief  ADC configuration:
  *             ADC                   = ADC1
  *             Data align            = Align right
  *             Scan mode             = Enable
  *             Continuous conversion = Enable
  *             Number of conversion  = 2
  *             Discontinuous mode    = Disable
  *             External trigger      = Software start
  *             Channel               = Channel 0, channel 1
  *             Channel rank          = 1, 2
  *             Channel sampling time = 239.5 cycle, 239.5 cycle
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
	ADC_ChannelConfTypeDef ADC_ChannelConfStruct;

	/*## STEP 1: Configure ADC ###############################################*/
	adcHandle.Instance                   = ADC1;
	adcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
	adcHandle.Init.ContinuousConvMode    = DISABLE;
	adcHandle.Init.NbrOfConversion       = 2;
	// adcHandle.Init.DiscontinuousConvMode = DISABLE;
	adcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
	if (HAL_ADC_Init(&adcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure ADC channel */
	ADC_ChannelConfStruct.Channel      = ADC_CHANNEL_0;
	ADC_ChannelConfStruct.Rank         = ADC_REGULAR_RANK_1;
	ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct) != HAL_OK)
	{
		Error_Handler();
	}
	ADC_ChannelConfStruct.Channel      = ADC_CHANNEL_1;
	ADC_ChannelConfStruct.Rank         = ADC_REGULAR_RANK_2;
	ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start ADC ###################################################*/
	if (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcValue, ADC_BUFFERSIZE)
			!= HAL_OK)
	{
		Error_Handler();
	}
}


/**
  * @brief  TIM configuration:
  *             TIM                = TIM4
  *             Prescaler          = 18
  *             Counter mode       = Up
  *             Period             = 1024
  *             Clock division     = 0
  *             Repetition counter = 0
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
	TIM_OC_InitTypeDef timOcInitStruct;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timHandle.Instance               = TIM4;
	/* TIM4CLK = CK_INT = 72 MHz
	 * Prescaler = 18
	 * CK_PSC = CK_CNT = clock counter = 72 MHz/18 = 4 MHz (0.25us) */
	timHandle.Init.Prescaler         = 18 - 1;
	timHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	/* ARR = counter overflow = period = 1024 count
	 * PWM signal period = 0.25us * 1024 = 256us (3906.25 Hz) */
	timHandle.Init.Period            = 1024 - 1;
	timHandle.Init.ClockDivision     = 0;
	timHandle.Init.RepetitionCounter = 0;
	timHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&timHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM PWM */
	timOcInitStruct.OCMode     = TIM_OCMODE_PWM1;
	timOcInitStruct.OCPolarity = TIM_OCPOLARITY_HIGH;
	timOcInitStruct.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_OC_ConfigChannel(&timHandle, &timOcInitStruct, TIM_CHANNEL_3)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if (HAL_TIM_PWM_Start(&timHandle, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}


	TIM_MasterConfigTypeDef timMasterConfig;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timAdcHandle.Instance               = TIM3;
	/* TIM1CLK = CK_INT = 72 MHz
	 * Prescaler = 18
	 * CK_PSC = CK_CNT = clock counter = 72 MHz/18 = 4 MHz (0.25us) */
	timAdcHandle.Init.Prescaler         = 18 - 1;
	timAdcHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	/* ARR = counter overflow = period = 1000 count
	 * TIM interrupt period = 0.25us * 1000 = 0.25ms */
	timAdcHandle.Init.Period            = 1000 - 1;
	timAdcHandle.Init.ClockDivision     = 0;
	timAdcHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&timAdcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM master TRGO */
	timMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	timMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&timAdcHandle, &timMasterConfig)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if (HAL_TIM_Base_Start(&timAdcHandle) != HAL_OK)
	{
		Error_Handler();
	}

}

/**
 * @brief  ADC MSP configuration callback.
 * @param  None
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef rccPeriphCLKInit;
	GPIO_InitTypeDef gpioInit;
	static DMA_HandleTypeDef dmaHandle;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	/* Configure ADC clock prescaler */
	__HAL_RCC_ADC1_CLK_ENABLE();
	rccPeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	rccPeriphCLKInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
	HAL_RCCEx_PeriphCLKConfig(&rccPeriphCLKInit);
	/* Configure RCC for GPIO */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* Configure RCC for DMA */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA0 and PA1 for ADC input */
	gpioInit.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3: Configure DMA ###############################################*/
	dmaHandle.Instance                 = DMA1_Channel1;
	dmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	dmaHandle.Init.Mode                = DMA_CIRCULAR;
	dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandle);
	__HAL_LINKDMA(hadc, DMA_Handle, dmaHandle);

	/*## STEP 4: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


/**
  * @brief  TIM MSP configuration callback.
  * @param  None
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PB8 for TIM4 CH3 output */
	gpioInit.Pin   = GPIO_PIN_8;
	gpioInit.Mode  = GPIO_MODE_AF_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioInit);
}

/**
  * @brief  Conversion complete callback in non blocking mode.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (adcValue[0] > 2047)
	{
		/* Turn yellow LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else
	{
		/* Turn yellow LED off */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	/*
	// Test to check ADC Timer trigger frequency, the GPIOB_9 should have a frequency of
	// (ADC Timer)/2 -> 4kHz/2 = 2kHz
	GPIO_PinState gpio_9_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

	if (gpio_9_state == GPIO_PIN_SET)
	{
		// Turn green LED on 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else
	{
		// Turn green LED off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	}

	*/
	
	if (adcValue[0] > 0 && adcValue[0] <= 1023)
	{
		__HAL_TIM_SET_AUTORELOAD(&timHandle, 1024);
		__HAL_TIM_SET_COMPARE(&timHandle, TIM_CHANNEL_3, 1024/2);
	}
	else if (adcValue[0] > 1023 && adcValue[0] <= 2047)
	{
		__HAL_TIM_SET_AUTORELOAD(&timHandle, 2048);
		__HAL_TIM_SET_COMPARE(&timHandle, TIM_CHANNEL_3, 2048/2);
	}
	else if (adcValue[0] > 2047 && adcValue[0] <= 3071)
	{
		__HAL_TIM_SET_AUTORELOAD(&timHandle, 3072);
		__HAL_TIM_SET_COMPARE(&timHandle, TIM_CHANNEL_3, 3072/2);
	}
	else
	{
		__HAL_TIM_SET_AUTORELOAD(&timHandle, 4096);
		__HAL_TIM_SET_COMPARE(&timHandle, TIM_CHANNEL_3, 4096/2);
	}
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* Turn red LED on */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	while (1);
}

int main (void) {

    /*## HAL initialization ##################################################*/
	HAL_Init();

	/*## System clocks initialization ########################################*/
	/* Set the SYSCLK at maximum frequency (72 MHz) */
	RCC_SystemClock_Config();

	/*## GPIO initialization #################################################*/
	GPIO_Output_Config();

	/*## TIM initialization #################################################*/
	TIM_Config();

	/*## ADC initialization ##################################################*/
	ADC_Config();

	/*## Main loop ###########################################################*/
	while (1);

    // Turn on the GPIOC peripheral
    //RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // Put pin 13 in general purpose push-pull mode
    //GPIOB->CRL &= ~(GPIO_CRL_CNF2);
    // Set the output mode to max. 2MHz
    //GPIOB->CRL |= GPIO_CRL_MODE2_1;

    //while (1) {
        // Reset the state of pin 13 to output low
    //    GPIOB->BSRR = GPIO_BSRR_BR2;

    //    delay(100);

        // Set the state of pin 13 to output high
    //    GPIOB->BSRR = GPIO_BSRR_BS2;

    //    delay(100);
    //}

    // Return 0 to satisfy compiler
    //return 0;
}
