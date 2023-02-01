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
#include <stdint.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//PV: PC
volatile uint8_t alarma =0;
#define SIZE_RX 15
char dato_Rx[SIZE_RX];
volatile uint8_t indice =0;
char errortime[100]="El numero ingresado no es correcto, por favor ingrese 500(120s)/1000(60s)/2000(30s) \r\n";
char helpg[100]="Nos devuelve la ubicacicion actual del gps\r\n";
char helpt[100]="Cambia el tiempo de interrupcion del timer segun 500(5s)/1000(10s)/2000(20s)\r\n ";
char helpa[100]="Activa/desactiva la alarma segun ON/OFF\r\n";
char aon[]="Alarma activada\r\n";
char aoff[]="Alarma apagada\r\n";
char start[]="Iniciando GPS TRACKER...\r\n";
char helpini[]="ingrese :helpt/:helpgubicacion/:helpaon/ :helpaoff para ver la ayuda de comandos\r\n";

//PV: GPS
uint8_t cadena[3];
uint8_t data[3];
#define MAX 100
char buffer[MAX];
char disableRMC[29] = "$PUBX,40,RMC,0,0,0,0,0,0\r\n";
char disableGLL[29] = "$PUBX,40,GLL,0,0,0,0,0,0\r\n";
char disableGSV[29] = "$PUBX,40,GSV,0,0,0,0,0,0\r\n";
char disableGSA[29] = "$PUBX,40,GSA,0,0,0,0,0,0\r\n";
char disableGGA[29] = "$PUBX,40,GGA,0,0,0,0,0,0\r\n";
char disableVTG[29] = "$PUBX,40,VTG,0,0,0,0,0,0\r\n";
char disableZDA[29] = "$PUBX,40,ZDA,0,0,0,0,0,0\r\n";
char poll[17] = "$PUBX,00*33\r\n";
char hora[] = "\r\n La hora en formato UTC es: ";
char buff[114];
uint8_t cnt=0;
char tiempo[20];
char latitud[20];
char longitud[20];
char lat[10];
char lon[10];
// The Equator has a latitude of 0°,
//the North Pole has a latitude of 90° North (written 90° N or +90°),
//and the South Pole has a latitude of 90° South (written 90° S or −90°)
char *latRaw;
char latDg[2];
char latMS[7];
char *hemNS;
// longitude in degrees (0° at the Prime Meridian to +180° eastward and −180° westward)
// that is why 3
char *lonRaw;
char lonDg[3];
char lonMS[7];
char *hemEW;
char *utcRaw; // raw UTC time from the NMEA sentence in the hhmmss format
char strUTC[8];
char hH[2]; // hours
char mM[2]; // minutes
char sS[2]; // seconds

//PV:SIM800L
char AT[]="AT";
char ECO[]="ATE0\r\n";
char ModoM[]="AT+CMGF=1\r\n";
char MSN[]="HOLA MUNDO\r\n";
char Number[]="AT+CMGS=\"+542615513690\"\r\n";
char enter[]="\0x1A\r\n";
uint8_t ctrlZin = 0x1A;
uint8_t da[]="";
char mensaje[MAX];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart1, cadena, 1);										//mensaje de inicio
  HAL_UART_Transmit(&huart1, (uint8_t *)start, strlen(start), 100);
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)helpini, strlen(helpini), 100);

  HAL_UART_Transmit(&huart3, (uint8_t *)disableRMC, strlen(disableRMC), 100);	//para desabilitar tramas del GPS
  HAL_UART_Transmit(&huart3, (uint8_t *)disableGSV, strlen(disableGSV), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)disableGSA, strlen(disableGSA), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)disableGGA, strlen(disableGGA), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)disableZDA, strlen(disableZDA), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)disableVTG, strlen(disableVTG), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)disableGLL, strlen(disableGLL), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableRMC, strlen(disableRMC), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableGSV, strlen(disableGSV), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableGSA, strlen(disableGSA), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableGGA, strlen(disableGGA), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableZDA, strlen(disableZDA), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableVTG, strlen(disableVTG), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)disableGLL, strlen(disableGLL), 100);

  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARMA_GPIO_Port, ALARMA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARMA_Pin */
  GPIO_InitStruct.Pin = ALARMA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARMA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void text_msn(mensaje){			//funcion para mandar los mensajes de texto.

	HAL_UART_Transmit(&huart1,(uint8_t *)AT,strlen(AT), HAL_MAX_DELAY);//Probamos el modulo
	HAL_UART_Transmit(&huart2,(uint8_t *)AT,strlen(AT), HAL_MAX_DELAY);


	  HAL_UART_Transmit(&huart1, (uint8_t *)ModoM,strlen(ModoM), HAL_MAX_DELAY);//Ponemos el modulo en modo mensaje
	  HAL_UART_Transmit(&huart2, (uint8_t *)ModoM,strlen(ModoM), HAL_MAX_DELAY);


	  HAL_UART_Transmit(&huart1, (uint8_t *)Number,strlen(Number), HAL_MAX_DELAY);//numero al que le mandamos mensaje
	  HAL_UART_Transmit(&huart2, (uint8_t *)Number,strlen(Number), HAL_MAX_DELAY);


	  HAL_UART_Transmit(&huart1, (uint8_t *)mensaje,strlen(mensaje), HAL_MAX_DELAY);//mensaje que mandamos
	  HAL_UART_Transmit(&huart2, (uint8_t *)mensaje,strlen(mensaje), HAL_MAX_DELAY);


	  HAL_UART_Transmit(&huart2, &ctrlZin, 1, 100);
	}

void interpreta_gps(void){

	HAL_UART_Transmit(&huart3, (uint8_t *) poll, strlen(poll), 100);			// enviamos comando $PUBX,00*33
	HAL_UART_Transmit(&huart1, (uint8_t *) poll, strlen(poll), 100);
	HAL_UART_Receive(&huart3, (uint8_t *)buff, 60,100);							// recibimos respuesta de hora y posicion
	HAL_UART_Transmit(&huart1, (uint8_t *) buff, strlen(buff), 100);
	cnt = 0;
	for (char *pV = strtok(buff, ","); pV != NULL; pV = strtok(NULL, ",")) {	// con un token separamos el string segun las comas
		switch (cnt) {															// y extraemos los datos correspondientes de hora y posicion
		case 2:																	// strtok(buff,",") separa el buff en punteros segun las comas
			utcRaw = strdup(pV);
			break;
		case 3:
			latRaw = strdup(pV);
			break;
		case 4:
			hemNS = strdup(pV);
			break;
		case 5:
			lonRaw = strdup(pV);
			break;
		case 6:
			hemEW = strdup(pV);
			break;
		}
		cnt++;
	}
	// adaptamos los valores extraidos segun si son grados, minutos, etc
	memcpy(latDg, &latRaw[0], 2);
	latDg[2] = '\0';
	memcpy(latMS, &latRaw[2], 7);
	latMS[7] = '\0';

	strcpy(latitud, latDg);
	strcat(latitud, " ");
	strcat(latitud, latMS);
	strcat(latitud, "\r\n");
	latitud[14] = '\0';

	strcpy(lat, &hemNS[0]);
	strcat(lat, ": ");
	lat[3]='\0';

	memcpy(lonDg, &lonRaw[0], 3);
	lonDg[3] = '\0';
	memcpy(lonMS, &lonRaw[3], 7);
	lonMS[7] = '\0';

	strcpy(longitud, lonDg);
	strcat(longitud, " ");
	strcat(longitud, lonMS);
	strcat(longitud, "\r\n");
	longitud[15] = '\0';

	strcpy(lon, &hemEW[0]);
	strcat(lon, ": ");
	lon[3]='\0';

	//covertimos UTC time en formato hh:mm:ss
	memcpy(hH, &utcRaw[0], 2);
	hH[2] = '\0';

	memcpy(mM, &utcRaw[2], 2);
	mM[2] = '\0';
	memcpy(sS, &utcRaw[4], 2);
	sS[2] = '\0';

	strcpy(tiempo, hH);
	strcat(tiempo, ":");
	strcat(tiempo, mM);
	strcat(tiempo, ":");
	strcat(tiempo, sS);
	strcat(tiempo, "\r\n");
	tiempo[12] = '\0';

	// adherimos todos los datos a un unico mensaje y lo mandamos a la funcion correspondiente
	memccpy(memccpy(memccpy(memccpy(memccpy(memccpy(buffer, hora, '\0', MAX) - 1, tiempo, '\0', MAX)- 1, lat, '\0', MAX)- 1, latitud, '\0', MAX)- 1, lon, '\0', MAX)- 1, longitud, '\0', MAX);

	text_msn(buffer);
}
void interpreta(void)
	  {
	uint16_t nuevot;
	nuevot=0;
	  	switch(dato_Rx[0])
	  		{
	  		case 'g':
	  		case 'G':  // devulve la ubicacion actual del GPS unasola vez
	  			if(strstr(dato_Rx, "ubicacion") || strstr(dato_Rx, "UBICACION") ){
	  				interpreta_gps();
	  			}
	  			memset(dato_Rx, 0, SIZE_RX);
	  			break;
	  		case 't':
	  		case 'T': // seteo del timer segun los valores predeterminados
	  			nuevot=atoi(&dato_Rx[1]);
	  			if(nuevot == 500 || nuevot == 2000 || nuevot == 1000){
		  			 __HAL_TIM_SET_PRESCALER(&htim2,nuevot);

	  			} else {
	  				HAL_UART_Transmit(&huart1, (uint8_t *)errortime, strlen(errortime), HAL_MAX_DELAY);
	  			}
	  			memset(dato_Rx, 0, SIZE_RX);
	  			  break;
	  		case 'a':
			case 'A': //on/off de la alarma (inicia y para el timer respectivamente)
				if(strstr(dato_Rx, "ON") || strstr(dato_Rx, "on") ){
					HAL_TIM_Base_Start_IT(&htim2);
					text_msn(aon);
					HAL_UART_Transmit(&huart1, (uint8_t *)aon, strlen(aon), HAL_MAX_DELAY);
				  }
				if(strstr(dato_Rx, "OFF") || strstr(dato_Rx, "off") ){
					HAL_TIM_Base_Stop_IT(&htim2);
					HAL_UART_Transmit(&huart1, (uint8_t *)aoff, strlen(aoff), HAL_MAX_DELAY);
					text_msn(aoff);
				  }
				memset(dato_Rx, 0, SIZE_RX);
				  break;
	  		case 'h':
	  		case 'H':	//helps
	  			if(strstr(dato_Rx,"Gubicacion") || strstr(dato_Rx, "GUBICACION") || strstr(dato_Rx,"gubicacion") || strstr(dato_Rx, "gUBICACION")){
	  				HAL_UART_Transmit(&huart1, (uint8_t *)helpg, strlen(helpg), HAL_MAX_DELAY);
	  			}
	  			if(strstr(dato_Rx,"T") || strstr(dato_Rx, "t")){
	  				HAL_UART_Transmit(&huart1, (uint8_t *)helpt, strlen(helpt), HAL_MAX_DELAY);
	  			}
	  			if(strstr(dato_Rx,"AON") || strstr(dato_Rx, "aon") || strstr(dato_Rx,"AOFF") || strstr(dato_Rx, "aoff")){
	  				HAL_UART_Transmit(&huart1, (uint8_t *)helpa, strlen(helpa), HAL_MAX_DELAY);
	  			}
	  			memset(dato_Rx, 0, SIZE_RX);
	  		default:
	  			memset(dato_Rx, 0, SIZE_RX);
	  			break;
	  		}
	  }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //interupciones por uart
{
	if(huart->Instance == USART2){ //interrupciones de uart 2, es lo que manda el GPRS
			HAL_UART_Transmit(&huart1, da, 1, HAL_MAX_DELAY);
			HAL_UART_Receive_IT(&huart2, da, 1);
		}

	if(huart->Instance == USART1) //interrupciones de uart 1
	{
		uint8_t dato = cadena[0];
		if(indice>= SIZE_RX) indice = 0;
		switch(dato)
			{
			case ':': indice=0; //iniciador de trama
					break;
			case '\r':
			case ';':		// el case \r hace lo mismo que el case ; fin de linea
					dato_Rx[indice]=0;
					interpreta();
					break;
			case 8: 	// 'borrar' en codigo ansi
					if (indice) indice--;
					break;
			default:
					dato_Rx[indice++] = dato;
					break;

			}
		HAL_UART_Receive_IT(&huart1, cadena, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		// pedido y procesamiento de los datos del gps cada vez que se cumple el tiempo
		interpreta_gps();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
