/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RING_SIZE 50
#define BUF_SIZE  200

#define	SSID	"H2SENSOR"
#define	PASSWD	"xxxxxxxx"
#define PROTOCOL	"TCP"
#define SEV_IP	"192.168.0.5"

#define DEBUG   0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_SIZE  24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//uint8_t slave_buffer_tx[]="This is slave tx data ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned int weather_count=0;
int overflow_flag=0;
int overflow_front_size=0;
uint8_t wh_flag=0;
uint8_t ret = 0;
uint8_t ready_flag=0;

uint32_t count=0;
uint32_t odd=0;
char buff_wh[100]={0,};
char parse[6][7]={0,};

uint32_t cnt=0;
uint32_t reset_cnt=0;
uint8_t rx_completed=0;
//uint8_t missing_count=0;

// for modbus request Address 32   {01}{03}{00}{20}{00}{02}{C5}{C1}  => {01}{03}{04}{00}{00}{7F}{80}{DB}{A3}
//uint8_t request[8]={0x01,0x03,0x00,0x20,0x00,0x02,0xc5,0xc1};   //resistor version
//uint8_t request[8]={0x01,0x03,0x00,0x1e,0x00,0x02,0xa4,0x0d};   // h2 voltage version
uint8_t request[8]={0x01,0x03,0x00,0x1c,0x00,0x02,0x05,0xcd};   // r2 voltage version
uint8_t vdata[9]={0,};

uint16_t txLen = 0;

uint8_t rcvFlag = 0;
uint8_t sndFlag = 0;


uint8_t rcvDat;
uint8_t rcvDat1;

uint8_t  ring_buffer[RING_SIZE];
uint32_t  ring_ptr_head = 0;
uint32_t  ring_ptr_tail = 0;

uint8_t  ring_buffer1[RING_SIZE];
uint32_t  ring_ptr_head1 = 0;
uint32_t  ring_ptr_tail1 = 0;

uint8_t h2_buffer[BUF_SIZE]={0,};
uint32_t value[2]={0,0};
uint8_t state=0;
uint8_t byte_count=0;
uint8_t ppm_data[20]={0,};
uint8_t data_count=0;
uint8_t emptyFlag =0;
uint8_t compare_cnt=0;

typedef union
{
	float fVal;
	unsigned long hex;
}H2VAL;

H2VAL h2_val;
uint16_t adc_val=3000;
char x_value[8]={0,};
char strADC[7]={0,};

float temp=0.0;
float humi=0.0;


void request_value(UART_HandleTypeDef *huart)
{

}

void increase_ptr(uint32_t * data_ptr)
{
    (* data_ptr)++;
    if(RING_SIZE <= (* data_ptr))
    {
        (* data_ptr) = 0;
    }
}

void Clean(void){
	for(int j=0;j< RING_SIZE; j++) ring_buffer[j]=0;
	ring_ptr_head = 0;
	ring_ptr_tail = 0;
}

void Clean1(void){
	for(int j=0;j< RING_SIZE; j++) ring_buffer1[j]=0;
	ring_ptr_head1 = 0;
	ring_ptr_tail1 = 0;
}

void EnQ(uint8_t data)
{
    ring_buffer[ring_ptr_head] = data;
    increase_ptr(&ring_ptr_head);
}

void EnQ_1(uint8_t data)
{
    ring_buffer1[ring_ptr_head1] = data;
    increase_ptr(&ring_ptr_head1);
}

uint8_t DeQ(void)
{

    uint8_t retVal = ring_buffer[ring_ptr_tail];
    increase_ptr(&ring_ptr_tail);

    return retVal;
}

uint8_t DeQ_1(void)
{
    uint8_t retVal = ring_buffer1[ring_ptr_tail1];
    increase_ptr(&ring_ptr_tail1);
    return retVal;
}

int _write(int file, unsigned char* p, int len)
{
	//HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == huart1.Instance)
	{

		EnQ_1(rcvDat1);
		HAL_UART_Receive_IT(&huart1, &rcvDat1, 1);

	}


	if(huart -> Instance == huart3.Instance)
	{
		EnQ(rcvDat);
		HAL_UART_Receive_IT(&huart3, &rcvDat, 1);
	}

}

uint8_t compare(const char * str, uint8_t len){
	uint8_t result=0;

	char dat[20]={0,};
	for(int k=0;k<20;k++) dat[k]=0;
	//HAL_IWDG_Refresh(&hiwdg);
	while(!result){

		//if(DEBUG) HAL_UART_Transmit(&huart2,(uint8_t *)"*",1,HAL_MAX_DELAY);

    	if(DeQ()=='O'){
          dat[0]='O';

		  for(int i=0;i<len-1;i++){
			  dat[i+1]=DeQ();
	       }

		  if(!strncmp(str, dat, len)) result=1;
    	}
    	else continue;
     }
	 //HAL_IWDG_Refresh(&hiwdg);
	 return 1;
}


uint8_t compare_OK(const char * str, uint8_t len){
	uint8_t result=0;
	compare_cnt=0;

	char dat[20]={0,};
	for(int k=0;k<20;k++) dat[k]=0;
	//HAL_IWDG_Refresh(&hiwdg);
	while(!result){
		compare_cnt++;
		if(compare_cnt > 42){
			result=1;
			//missing_count++;
		}
		//if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"*",1,HAL_MAX_DELAY);

    	if(DeQ()=='O'){
          dat[0]='O';

		  for(int i=0;i<len-1;i++){
			  dat[i+1]=DeQ();
	       }

		  if(!strncmp(str, dat, len)) result=1;
    	}
    	else continue;
     }
	 //HAL_IWDG_Refresh(&hiwdg);
	 return 1;
}



uint8_t parsing(){

	int len_buf=0;
	 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    if(ring_ptr_head1 < ring_ptr_tail1)
	{
    	len_buf= RING_SIZE-ring_ptr_tail1;
		for(int i=0;i<len_buf; i++)
		 {
			h2_buffer[i]=DeQ_1();
			//HAL_UART_Transmit(&huart3,&h2_buffer[i],1,HAL_MAX_DELAY);
		 }
		overflow_flag=1;
		overflow_front_size=len_buf;
		len_buf=ring_ptr_head1;
		ring_ptr_tail1=0;
	}

	else
	{

		len_buf= (int)(ring_ptr_head1 - ring_ptr_tail1)/sizeof(uint8_t);

	}

   // HAL_IWDG_Refresh(&hiwdg);

	if(len_buf >= BUF_SIZE)
	{
	  for(int h=0;h<BUF_SIZE;h++)
	  {
		  h2_buffer[h]=0;
		  DeQ_1();
	  }
	  len_buf=0;
    }

	//memset(h2_buffer, 0, BUF_SIZE);

	for(int i=0;i<len_buf; i++)
	 {
		if(overflow_flag==1){
			h2_buffer[i+overflow_front_size]=DeQ_1();
			//if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"ov\r\n",4,HAL_MAX_DELAY);
		}
		else{
			h2_buffer[i]=DeQ_1();
		}
	 }


	 // if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"gg\r\n",4,HAL_MAX_DELAY);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 count++;
     odd ^=1;
	 if(count > 500 ){
		 count=0;
		if(odd){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		  Clean1();
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
 		  HAL_UART_Transmit(&huart1,(uint8_t *)request,8,HAL_MAX_DELAY);
 		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
 		  count=0;
		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			parsing();
		}
	 }



}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  ready_flag=0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);

  //HAL_ADC_Start_DMA(&hadc1, value, 2);

  //sprintf(x_value,"%7d",adc_val);

  Clean();
  HAL_UART_Receive_IT(&huart3, &rcvDat, 1);
  HAL_UART_Receive_IT(&huart1, &rcvDat1, 1);

  Clean();
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"Initialize..\r\n",14,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  Clean();
  HAL_IWDG_Refresh(&hiwdg);
  HAL_UART_Transmit(&huart3,(uint8_t *)"AT+RST\r\n",8,HAL_MAX_DELAY);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);
  Clean();
  while(1){
  ret=0;
  Clean();
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"Sending AT..\r\n",14,HAL_MAX_DELAY);     // AT TEST
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  Clean();
  HAL_UART_Transmit(&huart3,(uint8_t *)"AT\r\n",4,HAL_MAX_DELAY);
  ret=compare("OK\r\n",4);
  if(ret) break;
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"AT OK!\r\n",8,HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(200);

  HAL_IWDG_Refresh(&hiwdg);

  Clean();
  while(1){
  ret=0;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"Setting Mode..\r\n",16,HAL_MAX_DELAY);    // Mode setting => station
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  Clean();
  HAL_UART_Transmit(&huart3,(uint8_t *)"AT+CWMODE=1\r\n",13,HAL_MAX_DELAY);
  HAL_IWDG_Refresh(&hiwdg);
  ret=compare("OK\r\n",4);
  if(ret) break;
  }
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"MODE OK!\r\n",10,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(200);

  HAL_IWDG_Refresh(&hiwdg);

  Clean();
  while(1){
  ret=0;
  char tmpData[40]={0,};
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  // HAL_UART_Transmit(&huart1,(uint8_t *)"Connecting AP..\r\n",17,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  sprintf (tmpData, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", SSID, PASSWD);    //AT+CWJAP_CUR="H2SENSOR","tsei1234"
  Clean();
  HAL_UART_Transmit(&huart3,(uint8_t *)tmpData,36,HAL_MAX_DELAY);
  ret=compare("OK\r\n",4);
  if(ret) break;
  }
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
 // HAL_UART_Transmit(&huart1,(uint8_t *)"Connect OK!\r\n",13,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(200);

  HAL_IWDG_Refresh(&hiwdg);

  Clean();
  while(1){
  ret=0;
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"Set Multiple..\r\n",16,HAL_MAX_DELAY);    // Mode setting => station
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  Clean();
  HAL_IWDG_Refresh(&hiwdg);
  HAL_UART_Transmit(&huart3,(uint8_t *)"AT+CIPMUX=0\r\n",13,HAL_MAX_DELAY);   // if multimode, link id is need when using sending commands
  ret=compare("OK\r\n",4);
  if(ret) break;
  }
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
 // HAL_UART_Transmit(&huart1,(uint8_t *)"Single OK!\r\n",12,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(100);

  HAL_IWDG_Refresh(&hiwdg);

  Clean();
  while(1){
  ret=0;
  char tmpStr[50]={0,};
  Clean();
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"SET TCP..\r\n",11,HAL_MAX_DELAY);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  sprintf (tmpStr, "AT+CIPSTART=\"%s\",\"%s\",5000\r\n", PROTOCOL, SEV_IP);  //AT+CIPSTART=0,"TCP","192.168.0.5",3306\r\n
  Clean();
  HAL_IWDG_Refresh(&hiwdg);
  HAL_UART_Transmit(&huart3,(uint8_t *)tmpStr,38,HAL_MAX_DELAY);
  ret=compare("OK\r\n",4);
  if(ret) break;
  }
  HAL_IWDG_Refresh(&hiwdg);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  //HAL_UART_Transmit(&huart1,(uint8_t *)"TCP OK!\r\n",9,HAL_MAX_DELAY);
   //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(200);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //if(missing_count>10) while(1);
      HAL_IWDG_Refresh(&hiwdg);
	  Clean();

	  h2_val.hex = (h2_buffer[5] <<24) | (h2_buffer[6] << 16) | (h2_buffer[3] << 8) | (h2_buffer[4]) ;
	  sprintf(x_value,"%6.4f\r\n",(h2_val.fVal));  //3.2345   in terms of voltage value
	 // sprintf(x_value,"%7d\r\n",(h2_val.fVal));  //152445   in terms of sensor resister value

	  while(1){
		  ret=0;

		  //if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"h1\r\n",4,HAL_MAX_DELAY);    // Mode setting => station

		  Clean();
		  HAL_UART_Transmit(&huart3,(uint8_t *)"AT+CIPSEND=6\r\n",14,HAL_MAX_DELAY);

		  //if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"h2\r\n",4,HAL_MAX_DELAY);
		  while(DeQ()!='>'){
			if (ring_ptr_tail >= ring_ptr_head) break;
		 }
		  HAL_Delay(200);
		 // if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"h3\r\n",4,HAL_MAX_DELAY);
		  HAL_IWDG_Refresh(&hiwdg);

		  Clean();
		  HAL_UART_Transmit(&huart3,(uint8_t *)x_value,8,HAL_MAX_DELAY);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

		  HAL_UART_Transmit(&huart2,(uint8_t *)x_value,8,HAL_MAX_DELAY);
		  HAL_Delay(300);
		  //if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"h5\r\n",4,HAL_MAX_DELAY);
		  ret=compare_OK("OK\r\n",4);
		  //if(DEBUG)HAL_UART_Transmit(&huart2,(uint8_t *)"h6\r\n",4,HAL_MAX_DELAY);

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		  if(ret) break;
       }

	  HAL_Delay(490);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB15 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
