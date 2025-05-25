/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "comm/NRF24_CORE.h" // Nosso driver NRF24
#include "comm/COMM_PACKETS.h" // Nosso gerenciador de pacotes << --- ADICIONADO/VERIFICAR CAMINHO
#include <stdio.h>      // Para printf
#include <string.h>     // Para strcmp, strlen, sprintf
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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// Variáveis globais para os pacotes e sequência

#if defined(TRANSMITTER_NODE) || defined(RECEIVER_NODE) // Assegura que só são compiladas se um modo NRF está ativo
static comm_packet_t nrf_packet_buffer; // Buffer para montar/receber o pacote
static uint8_t packet_seq_counter = 0;  // Contador de sequência de pacotes global ao arquivo main.c
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static comm_packet_t nrf_packet_buffer; // Buffer para montar/receber o pacote
static uint8_t packet_seq_counter = 0;  // Contador de sequência de pacotes

// Defina um dos dois para cada placa:
//#define TRANSMITTER_NODE  1 // Para a placa que vai ENVIAR pacotes SSL
#define RECEIVER_NODE     1 // Para a placa que vai RECEBER pacotes SSL

#if defined(TRANSMITTER_NODE) || defined(RECEIVER_NODE) // Este #if agora só protege os endereços/canal

// Endereços e canal (devem ser os mesmos para TX e RX)
uint8_t nrf_tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t nrf_rx_pipe1_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t nrf_rx_pipe2_lsb = 0xC3;
uint8_t NRF_COM_CHANNEL = 76;

#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // uint32_t tx_counter = 0; // Não mais usado para o conteúdo da mensagem principal
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n-- Teste NRF24L01+ com COMM_PACKETS --\r\n");

#if defined(TRANSMITTER_NODE) || defined(RECEIVER_NODE)
  printf("Inicializando NRF24L01+...\r\n");
  NRF24_Init();

  #if defined(TRANSMITTER_NODE) && !defined(RECEIVER_NODE)
    printf("Configurado como TRANSMISSOR SSL\r\n");
    NRF24_TxMode(nrf_tx_address, NRF_COM_CHANNEL);
    printf("Modo TX configurado. Endereco: %02X%02X%02X%02X%02X, Canal: %d\r\n",
           nrf_tx_address[0], nrf_tx_address[1], nrf_tx_address[2], nrf_tx_address[3], nrf_tx_address[4], NRF_COM_CHANNEL);
  #elif defined(RECEIVER_NODE) && !defined(TRANSMITTER_NODE)
    printf("Configurado como RECEPTOR SSL\r\n");
    NRF24_RxMode(nrf_rx_pipe1_address, nrf_rx_pipe2_lsb, NRF_COM_CHANNEL);
    printf("Modo RX configurado. Pipe1: %02X%02X%02X%02X%02X, Canal: %d\r\n",
           nrf_rx_pipe1_address[0], nrf_rx_pipe1_address[1], nrf_rx_pipe1_address[2], nrf_rx_pipe1_address[3], nrf_rx_pipe1_address[4], NRF_COM_CHANNEL);
  #else
    #error "Defina TRANSMITTER_NODE ou RECEIVER_NODE, mas nao ambos."
    printf("ERRO FATAL: Defina TRANSMITTER_NODE ou RECEIVER_NODE!\r\n");
    while(1);
  #endif
#else
    printf("AVISO: Nenhum modo NRF24 (TRANSMITTER_NODE ou RECEIVER_NODE) foi definido.\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if defined(TRANSMITTER_NODE) && !defined(RECEIVER_NODE)
      // Montar e enviar um pacote SSL
      ssl_payload_t ssl_data_payload;

      // Preencher dados do payload SSL (exemplo)
      ssl_data_payload.command_subtype = SSL_CMD_SET_VELOCITIES; // Usando o enum de COMM_PACKETS.h
      ssl_data_payload.robot_id = 5; // ID do Robô
      ssl_data_payload.vx = (int16_t)(100 + (packet_seq_counter % 50)); // Velocidade X variando um pouco
      ssl_data_payload.vy = (int16_t)(-50 - (packet_seq_counter % 20)); // Velocidade Y variando um pouco
      ssl_data_payload.vw = (int16_t)(10 * (packet_seq_counter % 10));   // Velocidade Angular variando um pouco
      ssl_data_payload.referee_command = 0; // Sem comando de juiz específico
      ssl_data_payload.kick_front = (packet_seq_counter % 10 == 0) ? 1 : 0; // Chuta a cada 10 pacotes
      ssl_data_payload.kick_chip = 0;
      ssl_data_payload.capacitor_charge = 1;
      ssl_data_payload.kick_strength = 100;
      ssl_data_payload.dribbler_on = (packet_seq_counter % 2 == 0) ? 1 : 0; // Liga/desliga driblador
      ssl_data_payload.dribbler_speed = 128;
      ssl_data_payload.movement_locked = 0;
      ssl_data_payload.critical_move_turbo = (packet_seq_counter % 20 == 0) ? 1 : 0; // Turbo a cada 20 pacotes

      // Criar o pacote NRF usando a função auxiliar
      Comm_Packets_Create_SSLMessage(&nrf_packet_buffer, packet_seq_counter++, &ssl_data_payload);

      printf("Enviando Pacote SSL (Seq: %d): ID %d, Vx=%d, Vy=%d, Vw=%d, KickF=%d, Drib=%d\r\n",
             nrf_packet_buffer.header.seq_number,
             ssl_data_payload.robot_id,
             ssl_data_payload.vx,
             ssl_data_payload.vy,
             ssl_data_payload.vw,
             ssl_data_payload.kick_front,
             ssl_data_payload.dribbler_on);

      if (NRF24_Transmit((uint8_t *)&nrf_packet_buffer, sizeof(comm_packet_t))) {
        printf("-> Sucesso (ACK)!\r\n");
        HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
      } else {
        printf("-> Falha (MAX_RT ou Timeout).\r\n");
      }
      HAL_Delay(500); // Envia a cada 0.5 segundo

#elif defined(RECEIVER_NODE) && !defined(TRANSMITTER_NODE)
      uint8_t raw_nrf_input_buffer[NRF_MAX_PACKET_SIZE];

      if (isDataAvailable(1)) { // Verifica Pipe 1
        NRF24_Receive(raw_nrf_input_buffer); // Lê os 32 bytes brutos

        // Copia os dados brutos para a nossa estrutura de pacote para fácil acesso
        memcpy(&nrf_packet_buffer, raw_nrf_input_buffer, sizeof(comm_packet_t));

        HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin); // Pisca LED ao receber qualquer pacote

        printf("Pacote Recebido! Tipo Principal: %d, Seq: %d\r\n",
               nrf_packet_buffer.header.main_type,
               nrf_packet_buffer.header.seq_number);

        // Processar com base no tipo principal do pacote
        switch (nrf_packet_buffer.header.main_type) {
            case MAIN_PACKET_TYPE_SSL_MESSAGE:
            {
                ssl_payload_t* ssl_data = &nrf_packet_buffer.payload_u.ssl_msg;
                printf("  SSL_MESSAGE: Subtipo=%d, ID=%d, Vx=%d, Vy=%d, Vw=%d, KickF=%d, Drib=%d, Turbo=%d\r\n",
                       ssl_data->command_subtype,
                       ssl_data->robot_id,
                       ssl_data->vx,
                       ssl_data->vy,
                       ssl_data->vw,
                       ssl_data->kick_front,
                       ssl_data->dribbler_on,
                       ssl_data->critical_move_turbo);
                // Aqui você faria algo com os dados do robô SSL...
                break;
            }
            case MAIN_PACKET_TYPE_VSSS_MESSAGE:
            {
                vsss_payload_t* vsss_data = &nrf_packet_buffer.payload_u.vsss_msg;
                printf("  VSSS_MESSAGE: Subtipo=%d, ID=%d, M1=%d, M2=%d, PWM?=%d\r\n",
                       vsss_data->command_subtype,
                       vsss_data->robot_id,
                       vsss_data->motor1_value,
                       vsss_data->motor2_value,
                       vsss_data->is_pwm_flag);
                // Aqui você faria algo com os dados do robô VSSS...
                break;
            }
            case MAIN_PACKET_TYPE_DEBUG_TEXT:
            {
                char temp_text[DEBUG_TEXT_MAX_LEN + 1];
                memcpy(temp_text, nrf_packet_buffer.payload_u.debug_text.text, DEBUG_TEXT_MAX_LEN);
                temp_text[DEBUG_TEXT_MAX_LEN] = '\0';
                printf("  DEBUG_TEXT: '%s'\r\n", temp_text);
                break;
            }
            default:
                printf("  Tipo de pacote principal desconhecido ou nao tratado: %d\r\n", nrf_packet_buffer.header.main_type);
                break;
        }
      }
      HAL_Delay(10); // Pequeno delay no loop do receptor
#endif
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000; // Verifique se seu conversor USB-Serial suporta
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
}

/**
  * @brief USB_OTG_FS Initialization Function
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DEBUG_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
     if (HAL_GetTick() % 200 < 100) {
        HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);
     } else {
        HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
     }
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  printf("Assert failed: file %s on line %d\r\n", (char*)file, (int)line);
  Error_Handler();
}
#endif /* USE_FULL_ASSERT */
