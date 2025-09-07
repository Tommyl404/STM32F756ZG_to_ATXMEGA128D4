/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct int64x2_t{
	uint64_t val[2];
} int64x2_t;

typedef struct {
	int64x2_t data;
}block_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//TODO: add a counter for every "PRG" inside to use it.
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN uint32_t pKeyCRYP[4] __ALIGN_END = {
                            0x00000000,0x00000000,0x00000000,0x00000000};

HASH_HandleTypeDef hhash;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
block_t receivedBlock; // Buffer to hold the received data
uint8_t txBuffer[] = "DONE"; // Buffer to hold the transmitted data
uint8_t tmpBoolArrayZeroToSeven[8];
uint8_t tmpBoolArrayEightToRest[8];

int keys_used = 8;
uint64_t gid = 0;
uint32_t cipher[4];
//uint8_t buffer[16];

//uint8_t stage_2_buffer;
int counter = 0;
int prg_counter[128];

uint8_t ciphertext[16];
block_t arrayOfAesKeys[128]; //stage 0
block_t arrayOfBlocks[16*128]; //stage 1
block_t t[16*128];
block_t blockOfBoolInBeforeAlgorithm;
bool arrayOfBools[128]; // stage 2
block_t tmpBlockInIKNP; //stage 3
bool receivedBool;
uint64_t local_block_size;
int stages = 0;
int counter_for_sending_data = 0;
int length = 1; //TODO: CHANGE THIS IF NEEDED.


block_t data[1];
block_t delta;
block_t S;
block_t keysForAes[8];


// FOR TINY AES!!!!!!
struct AES_ctx ctx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRYP_Init(void);
static void MX_HASH_Init(void);
static void MX_RNG_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void uint64_to_uint8_array(uint64_t num, uint8_t arr[8]);
void block_to_uint8_array(block_t num, uint8_t arr[16]);
int64x2_t pack_128_bits_into_int64x2_t(uint8_t* data);
void random_block_tommyVer(block_t * data, uint64_t nblocks, int prg_serial_num);
void random_data_tommyVer(block_t * data, uint64_t nbytes, int prg_serial_num);
uint64_t Convert_to_double_word(uint8_t array_of_bytes[]);
void uint8_to_bool_array(uint8_t byte, bool* bool_array);
void buffer_to_bool_array(const uint8_t* buffer, size_t numValues, bool* bool_array);
void xorBlocks_arr(block_t* res, const block_t* x, const block_t* y, int nblocks);
void mitccrh_rk();
void mitccrh_hash(block_t * blks);
void uint64_to_uint32_array(uint64_t num, uint32_t arr[2]);
uint64_t uint32_array_to_uint64(uint32_t arr[2]);
uint64_t reverse_bytes(uint32_t val);
uint32_t reverse_bytes_for_key(uint32_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRYP_Init();
  MX_HASH_Init();
  MX_RNG_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  //TIM2->PSC = 0;
  //TIM2->CR1 |= TIM_CR1_CEN;

  //a.data.val[0] = 1;
  //a.data.val[1] = 0;

  //b.data.val[0] = 2;
  //b.data.val[1] = 0;

  block_t pad[16];
  //xorBlocks_arr(&a, &a , &b, 1);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(stages == 3)
	  {
		  stages++;
		  //HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_SET);//LOGIC 1
		  for(int i = 0; i < length; i = i + 8)
		  {
			  //part 1
			  if(length < (i+8))
			  {
				  for(int j = i; j < length; ++j)
				  {
					  pad[2*(j-i)].data.val[0] = data[j].data.val[0];
					  pad[2*(j-i)].data.val[1] = data[j].data.val[1];
					  xorBlocks_arr(&pad[2*(j-i) + 1], &data[j], &delta, 1);
				  }
			  }
			  else
			  {
				  for(int j = i; j < (i+8); ++j)
				  {
					  pad[2*(j-i)].data.val[0] = data[j].data.val[0];
					  pad[2*(j-i)].data.val[1] = data[j].data.val[1];
					  xorBlocks_arr(&pad[2*(j-i) + 1], &data[j], &delta, 1);
				  }
			  }
			  //part 2
			  //TODO: HASH FUNCTION TODAY!
			  //HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_SET);//LOGIC 1
			  //for(int bingos = 0; bingos < 40; bingos++)
			  //{
				  //serve as nope
			  //}
			  mitccrh_hash(pad);
			  //HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);//LOGIC 0
		  }
	  }
	  if(stages == 4)
	  {
		  stages = 0;
		  gid = 0;
		  HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
	  }
	  /*
	  //this is when the stages of the inputs of the data are over
	  //we need to make the stm32 intake the bools from IKNP here.
	  if(stages == 4)
	  {
		  stages++;
		  uint64_to_uint8_array(blockOfBoolInBeforeAlgorithm.data.val[0],tmpBoolArrayZeroToSeven);
		  uint64_to_uint8_array(blockOfBoolInBeforeAlgorithm.data.val[1], tmpBoolArrayEightToRest);
		  buffer_to_bool_array(tmpBoolArrayZeroToSeven, 64, arrayOfBools);
		  buffer_to_bool_array(tmpBoolArrayEightToRest, 64, arrayOfBools+64);
		  //xorBlocks_arr(arrayOfBlocks, x, y, nblocks)
	  }

	  //TODO: HERE IS THE PART THAT THE STM32 TAKES DO THE ARITMETIC CALCS.
	  if(stages == 5)
	  {

		  stages++;
		  counter = 0;
		  //start by saying that prg is new and init it to 0
		  for(int i = 0 ; i < 128; i++)
		  {
			  prg_counter[i] = 0;
		  }

		  for(int i = 0; i < 128 ; ++i)
		  {
			  // Step 1: Stop the current CRYP process
			  if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
			  {
				  // Handle error
				  while(1)
				  {

				  }
			  }
			  // Step 2: Update the key by converting the 64-bit parts to 32-bit parts
			  pKeyCRYP[0] = (uint32_t)(arrayOfAesKeys[i].data.val[0] & 0xFFFFFFFF);
			  pKeyCRYP[1] = (uint32_t)(arrayOfAesKeys[i].data.val[0] >> 32);
			  pKeyCRYP[2] = (uint32_t)(arrayOfAesKeys[i].data.val[1] & 0xFFFFFFFF);
			  pKeyCRYP[3] = (uint32_t)(arrayOfAesKeys[i].data.val[1] >> 32);

			  // Step 3: Reinitialize the CRYP module with the new key
			  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
			  {
				  // Handle error
				  while(1)
				  {

				  }
			  }

			  //int32_t test_time_before = TIM2->CNT;
			  //TIM2->CNT = 0;
			  //TODO: change 128/8 -> local_block_size/8
			  HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_SET);//LOGIC 1
			  for(int j = 0; j < 20; j++)
			  			  {}
			  random_data_tommyVer(t+(i*2048/128)/16, 128/8, i);
			  if(arrayOfBools[i])
			  {
				  xorBlocks_arr(t+(i*2048/128)/16, t+(i*2048/128)/16, arrayOfBlocks+(i*128/128)/16, 2);
			  }
			  HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);//LOGIC 0
			  //uint32_t test_time_after = TIM2->CNT;
			  for(int j = 0; j < 50; j++)
			  {}
			  //HAL_UART_Transmit_IT(&huart2, (uint8_t*)&t[0], sizeof(receivedBlock));

		  }
	 }
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRYP Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRYP_Init(void)
{

  /* USER CODE BEGIN CRYP_Init 0 */

  /* USER CODE END CRYP_Init 0 */

  /* USER CODE BEGIN CRYP_Init 1 */

  /* USER CODE END CRYP_Init 1 */
  hcryp.Instance = CRYP;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeyCRYP;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_BYTE;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRYP_Init 2 */

  /* USER CODE END CRYP_Init 2 */

}

/**
  * @brief HASH Initialization Function
  * @param None
  * @retval None
  */
static void MX_HASH_Init(void)
{

  /* USER CODE BEGIN HASH_Init 0 */

  /* USER CODE END HASH_Init 0 */

  /* USER CODE BEGIN HASH_Init 1 */

  /* USER CODE END HASH_Init 1 */
  hhash.Init.DataType = HASH_DATATYPE_32B;
  if (HAL_HASH_Init(&hhash) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HASH_Init 2 */

  /* USER CODE END HASH_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAG_Pin */
  GPIO_InitStruct.Pin = FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLAG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void buffer_to_bool_array(const uint8_t* buffer, size_t numValues, bool* bool_array) {
    for (size_t i = 0; i < numValues; ++i) {
        bool_array[i] = (buffer[i / 8] & (1 << (i % 8))) != 0;
    }
}

void uint8_to_bool_array(uint8_t byte, bool* bool_array) {
    for (size_t i = 0; i < 8; ++i) {
        bool_array[i] = (byte & (1 << i)) != 0;
    }
}


void uint64_to_uint8_array(uint64_t num, uint8_t arr[8]) {
    for (int i = 0; i < 8; ++i) {
        arr[i] = (num >> (8 * i)) & 0xFF; // Isolate each byte
    }
}

void block_to_uint8_array(block_t num, uint8_t arr[16]) {
    for (int i = 0; i < 16; ++i) {
    	if (i < 8)
    		arr[i] = (num.data.val[1] >> (8 * i)) & 0xFF; // Isolate each byte
    	else
    		arr[i] = (num.data.val[0] >> (8 * (i-8))) & 0xFF; // Isolate each byte

    }
}

void random_block_tommyVer(block_t * data, uint64_t nblocks,int prg_serial_num) // fuck, i need to hard code nblock...
{
	uint8_t buffer[16];
	block_t temp[ nblocks];

	//maker of temp blocks, it seems like that PRG is using the fact that aes-ecb on m=0 and m=1 will be totally diffrent
	for(int j = 0; j < nblocks; ++j){
		temp[j].data.val[0] = 0LL;
		temp[j].data.val[1] = prg_counter[prg_serial_num]++;
		uint64_to_uint8_array(temp[j].data.val[0], buffer);
		uint64_to_uint8_array(temp[j].data.val[1], buffer+8);
		HAL_CRYP_Encrypt(&hcryp, buffer, 16 , ciphertext, 100);
		//HAL_CRYP_Encrypt_IT(&hcryp, buffer, 16, ciphertext);
		data[j].data = pack_128_bits_into_int64x2_t(ciphertext);
	}



}

void mitccrh_rk()
{
	block_t tmp[8];
	for(int i = 0; i < 8; i++)
	{
		//REAL CODE
		//tmp[i].data.val[1] = gid++;
		//tmp[i].data.val[0] = 0;
		//xorBlocks_arr(&keysForAes[i], &tmp[i], &S, 1);

		//FOR TEST.
		keysForAes[i].data.val[0] = 0;
		keysForAes[i].data.val[1] = 0;
	}
	keys_used = 0;
	//delete this!
	//keysForAes[0].data.val[0] = 305419896; //aaaaaaaa
	//keysForAes[0].data.val[1] = 0;
	//keysForAes[0].data.val[1] = 0; //low (in memory first 64 bits)
	//keysForAes[0].data.val[0] = 0; // high (in memory last 64 bits)

}

void mitccrh_hash(block_t * blks)
{
	if(keys_used == 8)
	{
		mitccrh_rk();
	}

	block_t tmp[16];
	for(int i = 0; i < 16; ++i)
	{
		tmp[i].data.val[0] = blks[i].data.val[0];
		tmp[i].data.val[1] = blks[i].data.val[1];
	}
	//tmp[0].data.val[0] = 0;
	//tmp[0].data.val[1] = 0;
	//MY PART OF THE NEW ALGO.
	uint8_t buffer[16];
	uint32_t buffer_32t[4];
	//ALL OF THIS IS ParaEnc()
	for(int g = 0; g < 16; g = g + 2) //maybe needed to make g + 2, i have run over g+1 every time. TODO:
	{
		if (HAL_CRYP_DeInit(&hcryp) != HAL_OK)
		{
			// Handle error
			while(1)
			{

			}
		}
		uint8_t array_for_key[16];
		block_to_uint8_array(keysForAes[g], array_for_key);


		// Step 2: Update the key by converting the 64-bit parts to 32-bit parts
		pKeyCRYP[3] = (uint32_t)(keysForAes[g].data.val[0] >> 32);
		//pKeyCRYP[3] = (uint32_t)(keysForAes[g].data.val[0] & 0xFFFFFFFF);
		pKeyCRYP[2] = (uint32_t)(keysForAes[g].data.val[0] & 0xFFFFFFFF);
		//pKeyCRYP[2] = (uint32_t)(keysForAes[g].data.val[0] >> 32);
		pKeyCRYP[1] = (uint32_t)(keysForAes[g].data.val[1] >> 32);
		//pKeyCRYP[1] = (uint32_t)(keysForAes[g].data.val[1] & 0xFFFFFFFF);
		pKeyCRYP[0] = (uint32_t)(keysForAes[g].data.val[1] & 0xFFFFFFFF);
		//pKeyCRYP[0] = (uint32_t)(keysForAes[g].data.val[1] >> 32);

		// Step 3: Reinitialize the CRYP module with the new key
		if (HAL_CRYP_Init(&hcryp) != HAL_OK)
		{
		// Handle error
			while(1)
			{

			}
		}
		//Step 4: encrypt tmp[g] and tmp[g+1] with the same key.

		uint64_to_uint8_array(tmp[g].data.val[0], buffer);
		uint64_to_uint8_array(tmp[g].data.val[1], buffer+8);
		if(g == 0)
		{
			//for(int i = 0; i < 4; i++)
			//{
			//	buffer_32t[i] = 0;
			//}
			//for(int bingos = 0; bingos < 400; bingos++)
			//{
							  //serve as nope
			//}
			//TODO: DO NOT LEAVE IT LIKE THIS! THIS IS ONLY A CHECK!
			// WHAT WE NEED TO DO IS TO JUST CHECK IF IT WORKS AND MAKE
			// SURE THIS IS THE KEY WE NEED!
			for (int i = 0; i < 16; i++)
			{
				array_for_key[i] = 0;
			}
			array_for_key[15] = 170;
			AES_init_ctx(&ctx, array_for_key);
			HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_SET);//LOGIC 1

			//for(int bingos = 0; bingos < 400; bingos++)
			//{
							  //serve as nope
			//}
			AES_ECB_encrypt(&ctx, buffer);
			//HAL_CRYP_Encrypt(&hcryp, buffer_32t, 16 , cipher, 600);
			HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);//LOGIC 0
			//CRYP_ConfigTypeDef pConf;
			//HAL_CRYP_GetConfig(&hcryp, &pConf);
			//HAL_CRYP_SetConfig(&hcryp, &pConf);
			//HAL_CRYP_Encrypt_IT(&hcryp, buffer_32t, 16, cipher);
			//for(int bingos = 0; bingos < 400; bingos++)
			//{
				//serve as nope
			//}
			//HAL_GPIO_WritePin(FLAG_GPIO_Port, FLAG_Pin, GPIO_PIN_RESET);//LOGIC 0
		}
		//HAL_CRYP_Encrypt(&hcryp, buffer_32t, 16 , cipher, 100);
		tmp[g].data.val[0] = uint32_array_to_uint64(cipher);
		tmp[g].data.val[1] = uint32_array_to_uint64(cipher+2);
		//tmp[g].data = pack_128_bits_into_int64x2_t(ciphertext);

		/*
		uint64_to_uint32_array(tmp[g+1].data.val[0], buffer_32t);
		uint64_to_uint32_array(tmp[g+1].data.val[1], buffer_32t+2);
		for(int i = 0; i < 4; i++)
		{
			buffer_32t[i] = reverse_bytes_for_key(buffer_32t[i]);
		}
		//uint64_to_uint8_array(tmp[g+1].data.val[0], buffer);
		//uint64_to_uint8_array(tmp[g+1].data.val[1], buffer+8);
		HAL_CRYP_Encrypt(&hcryp, buffer_32t, 16 , cipher, 100);
		//HAL_CRYP_Encrypt(&hcryp, buffer, 16 , ciphertext, 100);
		tmp[g+1].data.val[0] = uint32_array_to_uint64(cipher);
		tmp[g+1].data.val[1] = uint32_array_to_uint64(cipher+2);
		//tmp[g+1].data = pack_128_bits_into_int64x2_t(ciphertext);
		 *
		 */
	}

	keys_used += 8;

	//and now -> the end! xor of blks and tmp.
	for(int i = 0;i < 16; ++i)
	{
		xorBlocks_arr(&blks[i], &blks[i], &tmp[i], 1);
	}
}

uint64_t reverse_bytes(uint32_t val) {
    val = ((val & 0xFF000000) >> 24) |
          ((val & 0x00FF0000) >> 8)  |
          ((val & 0x0000FF00) << 8)  |
          ((val & 0x000000FF) << 24);
    return val;
}

uint32_t reverse_bytes_for_key(uint32_t val) {
    val = ((val & 0xFF000000) >> 24) |
          ((val & 0x00FF0000) >> 8)  |
          ((val & 0x0000FF00) << 8)  |
          ((val & 0x000000FF) << 24);
    return val;
}

void uint64_to_uint32_array(uint64_t num, uint32_t arr[2]) {
    for (int i = 0; i < 2; ++i) {
        arr[i] = (num >> (32 * i)) & 0xFFFFFFFF; // Isolate each 32-bit segment
    }
}

uint64_t uint32_array_to_uint64(uint32_t arr[2]) {
	uint64_t num = 0;
	uint32_t high = reverse_bytes(arr[1]); // Reverse bytes of arr[1] to place it in the high bits
	uint32_t low = reverse_bytes(arr[0]);  // Reverse bytes of arr[0] to place it in the low bits
	num = ((uint64_t)high << 32) | low;    // Combine the reversed parts correctly
	return num;
}


/*
 * random_data_tommyVer:
 * Input - an array of type block_t and the number of bytes that wee need as random.
 * Output - VOID
 * the main goal of this function is to get a number X of bytes that the user needs. the bytes that the user needs MUST BE PESUDO-RANDOMIC
 * so, it will calculate how many blocks does those bytes represent and make them as BLOCK with random_block_tommyVer
 * moreover, if nbytes does not give us a full block, we will still make extra block for the use of the user. he will choose what bytes he want to use (this is the logic from the main API)
 * */
void random_data_tommyVer(block_t * data, uint64_t nbytes, int prg_serial_num)
{
	random_block_tommyVer(data, nbytes/16,prg_serial_num);
	if(nbytes % 16 != 0)
	{
		random_block_tommyVer(data + (nbytes/16) + 1, 1,prg_serial_num); //data + (n-1 place in the array) + 1 = data[n]
	}
}


int64x2_t pack_128_bits_into_int64x2_t(uint8_t* data) {
    int64x2_t result;

    // Adjusting for the updated Convert_to_double_word which assumes big-endian input
    // Load the first 64 bits (MSB) into the first element of the array within the struct
    result.val[0] = Convert_to_double_word(data);

    // Load the next 64 bits into the second element of the array within the struct
    result.val[1] = Convert_to_double_word(data + 8);

    return result;
}

uint64_t Convert_to_double_word(uint8_t array_of_bytes[])
{
	uint64_t temp = 0x00;

		for (int index = 0; index < 8; index++)
			temp +=  (uint64_t)array_of_bytes[index]<<(index*8);


	return temp;
}

void xorBlocks_arr(block_t* res, const block_t* x, const block_t* y, int nblocks) {
	const block_t* dest = nblocks + x;
	for (; x != dest;) {
		res->data.val[0] = x->data.val[0] ^ y->data.val[0];
		res->data.val[1] = x->data.val[1] ^ y->data.val[1];
		res++;
		x++;
		y++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		if(stages == 0) //delta
		{
			delta.data.val[0]= receivedBlock.data.val[0];
			delta.data.val[1] = receivedBlock.data.val[1];
			counter = 0;
			stages++;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));


			/*
			//arrayOfAesKeys[counter] = receivedBlock;
			arrayOfAesKeys[counter].data.val[0] = receivedBlock.data.val[0];
			arrayOfAesKeys[counter].data.val[1] = receivedBlock.data.val[1];
			counter++;
			if(counter == 128)
			{
				counter = 0;
				stages++;
				//HAL_UART_AbortReceive(&huart2); //ask if needed or not
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			else
			{
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			*/
		}
		else if(stages == 1) //S
		{
			S.data.val[0]= receivedBlock.data.val[0];
			S.data.val[1] = receivedBlock.data.val[1];
			counter = 0;
			stages++;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));


			/*
			tmpBlockInIKNP.data.val[0] = receivedBlock.data.val[0];
			tmpBlockInIKNP.data.val[1] = receivedBlock.data.val[1];
			local_block_size = receivedBlock.data.val[0];
			counter = 0;
			stages++;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			*/


			//arrayOfBlocks[counter] = receivedBlock;
			/*
			arrayOfBlocks[counter].data.val[0] = receivedBlock.data.val[0];
			arrayOfBlocks[counter].data.val[1] = receivedBlock.data.val[1];
			counter++;
			if(counter == 16*128)
			{
				counter = 0;
				flag_for_stage_2 = 1;
				stages++;
				//stages = 3;
				//HAL_UART_AbortReceive(&huart2); //ask if needed or not
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			else
			{
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			*/
		}
		else if(stages == 2) //DATA
		{
			data[counter].data.val[0] = receivedBlock.data.val[0];
			data[counter].data.val[1] = receivedBlock.data.val[1];
			counter++;
			if(counter == length)
			{
				counter = 0;
				stages++;
				//HAL_UART_AbortReceive(&huart2); //ask if needed or not
				//HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			else
			{
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}




			/*
			arrayOfBlocks[counter].data.val[0] = receivedBlock.data.val[0];
			arrayOfBlocks[counter].data.val[1] = receivedBlock.data.val[1];
			counter++;
			if(counter == 16*128) //in the real deal TODO: counter == local_block_size
			{
				counter = 0;

				stages++;
				//stages = 3;
				//HAL_UART_AbortReceive(&huart2); //ask if needed or not
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			else
			{
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}
			*/




			//blockOfBoolInBeforeAlgorithm.data.val[0]= receivedBlock.data.val[0];
			//blockOfBoolInBeforeAlgorithm.data.val[1] = receivedBlock.data.val[1];
			//receivedBool = (stage_2_buffer  != 0);
			//arrayOfBools[counter] = receivedBool;
			// Receive the bool value as a single byte
			//uint8_t boolValue;
			//HAL_UART_Receive_IT(&huart2, &boolValue, sizeof(boolValue));
			//uint64_to_uint8_array(receivedBlock.data.val[0],tmpBoolArrayZeroToSeven);
			//uint64_to_uint8_array(receivedBlock.data.val[1], tmpBoolArrayEightToRest);
			// Interpret the received byte as a bool
			//bool receivedBool = (boolValue != 0);
			//arrayOfBools[counter] = receivedBool;
			/*for(int i = 0 ; i < 16 ; i++)
			{
				if(i < 8)
				{
					bool receivedBool = (tmpBoolArrayZeroToSeven[i] != 0);
					arrayOfBools[counter] = receivedBool;
					counter++;
				}

				else
				{
					bool receivedBool = (tmpBoolArrayEightToRest[i-8] != 0);
					arrayOfBools[counter] = receivedBool;
					counter++;
				}

			}*/

			//if(counter == 128)
			//{
				//counter = 0;
				//stages++;

				//HAL_UART_AbortReceive(&huart2); //ask if needed or not
			//}
			/*else
			{
				//HAL_UART_Receive_IT(&huart2, stage_2_buffer, sizeof(stage_2_buffer));
				HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
			}*/
				//if(stages == 3)
					//HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
		}
		/*
		else if(stages == 3)//BOOLS
		{
			blockOfBoolInBeforeAlgorithm.data.val[0]= receivedBlock.data.val[0];
			blockOfBoolInBeforeAlgorithm.data.val[1] = receivedBlock.data.val[1];
			counter = 0;
			stages++;
			//tmpBlockInIKNP.data.val[0] = receivedBlock.data.val[0];
			//tmpBlockInIKNP.data.val[1] = receivedBlock.data.val[1];
			//counter = 0;
			//stages++;

			//HAL_UART_AbortReceive(&huart2); //ask if needed or not
		}
		*/

	}


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	counter_for_sending_data++;
	if(counter_for_sending_data < 2048)
	{
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)&t[counter_for_sending_data], sizeof(receivedBlock));
	}
	else
	{
		stages = 0;
		counter = 0;
		counter_for_sending_data = 0;
		HAL_UART_Receive_IT(&huart2, (uint8_t*)&receivedBlock, sizeof(receivedBlock));
	}


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
