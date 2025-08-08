/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware da Placa 1 - Parser da Radiosonda e Transmissor LoRa
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"
#include "stm32wlxx_nucleo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "radio_driver.h"

/* USER CODE BEGIN 0 */
// Redireciona a saída do printf para a USART2 (nossa porta de debug para o PC)
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Definições do Parser da Radiosonda ---
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    int32_t  latitude_raw;
    int32_t  longitude_raw;
    int32_t  altitude_raw;
    uint16_t voltage_mv;
    int8_t   radio_temp_c;
    uint8_t  sats_and_fix;
} LoRaPayload_t;

const uint8_t SYNC_WORD = 0xAA;
#define PAYLOAD_SIZE sizeof(LoRaPayload_t)

enum ParserState { AWAITING_SYNC, RECEIVING_PAYLOAD, AWAITING_CHECKSUM };

#define RADIOSONDE_UART_BUFFER_SIZE 256

// --- Parâmetros LoRa ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm (Aumentado para o máximo)
#define LORA_BANDWIDTH                              0         // 0: 125 kHz (Mantido)
#define LORA_SPREADING_FACTOR                       10        // SF10 (Aumentado de 7)
#define LORA_CODINGRATE                             4         // 4: 4/8 (Aumentado de 1, que era 4/5)
#define LORA_PREAMBLE_LENGTH                        8         // (Mantido)
/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// --- Variáveis do Parser e DMA ---
uint8_t radiosonde_rx_buffer[RADIOSONDE_UART_BUFFER_SIZE];
uint16_t old_pos = 0;
uint8_t payloadBuffer[PAYLOAD_SIZE];
int byteCounter = 0;
enum ParserState currentState = AWAITING_SYNC;
volatile bool tx_done = true; // Flag para controlar o estado da transmissão LoRa

// Array de lookup para Bandwidth
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void SendLoRaPacket(LoRaPayload_t* payload);
uint8_t calculate_checksum(uint8_t* data, int length);
void ProcessByte(uint8_t receivedByte);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq); // Callback para eventos do rádio
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();  // UART da Radiosonda
  MX_USART2_UART_Init();  // UART de Debug para o PC
  MX_SUBGHZ_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);

  printf("\r\n--- Placa 1: Parser Radiosonda & Tx LoRa ---\r\n");

  Radio_Init();
  printf("Radio LoRa inicializado.\r\n");

  printf("Estado: Aguardando SYNC_WORD (0xAA) da radiosonda na USART1...\r\n");

  // Inicia a recepção via DMA na USART1 (rádiosonda)
  HAL_UART_Receive_DMA(&huart1, radiosonde_rx_buffer, RADIOSONDE_UART_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t new_pos = RADIOSONDE_UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (new_pos != old_pos)
    {
      if (new_pos > old_pos) {
        for (int i = old_pos; i < new_pos; i++) { ProcessByte(radiosonde_rx_buffer[i]); }
      } else { // Wrap-around
        for (int i = old_pos; i < RADIOSONDE_UART_BUFFER_SIZE; i++) { ProcessByte(radiosonde_rx_buffer[i]); }
        for (int i = 0; i < new_pos; i++) { ProcessByte(radiosonde_rx_buffer[i]); }
      }
      old_pos = new_pos;
    }

    // NÃO é necessário chamar nenhuma função de callback do rádio aqui.
    // O sistema de interrupções (NVIC) cuida disso automaticamente em background.

  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// --- Nossas funções de apoio ---

void ProcessByte(uint8_t receivedByte)
{
  switch (currentState)
  {
    case AWAITING_SYNC:
      if (receivedByte == SYNC_WORD) {
        byteCounter = 0;
        currentState = RECEIVING_PAYLOAD;
      }
      break;

    case RECEIVING_PAYLOAD:
      if (byteCounter < PAYLOAD_SIZE) {
        payloadBuffer[byteCounter++] = receivedByte;
      }
      if (byteCounter >= PAYLOAD_SIZE) {
        currentState = AWAITING_CHECKSUM;
      }
      break;

    case AWAITING_CHECKSUM:
      {
        uint8_t receivedChecksum = receivedByte;
        uint8_t calculatedChecksum = calculate_checksum(payloadBuffer, PAYLOAD_SIZE);

        if (receivedChecksum == calculatedChecksum) {
          printf("Checksum OK. Pacote da radiosonda validado.\r\n");
          BSP_LED_Toggle(LED_GREEN);
          SendLoRaPacket((LoRaPayload_t*)payloadBuffer);
        } else {
          printf("Falha no Checksum! Esperado: 0x%02X, Recebido: 0x%02X\r\n", calculatedChecksum, receivedChecksum);
        }
        currentState = AWAITING_SYNC;
      }
      break;
  }
}

void SendLoRaPacket(LoRaPayload_t* payload)
{
    if (tx_done == true) // Só transmite se a transmissão anterior já terminou
    {
        tx_done = false; // Bloqueia novas transmissões
        printf("Transmitindo pacote LoRa ID: %lu\r\n", payload->packet_id);
        SUBGRF_SendPayload((uint8_t*)payload, PAYLOAD_SIZE, 0); // Timeout 0 para não bloquear
    }
    else
    {
        printf("WARN: Rádio ocupado, pacote descartado.\r\n");
    }
}

void Radio_Init(void)
{
    SUBGRF_Init(RadioOnDioIrq);

    SUBGRF_SetStandby(STDBY_RC);
    SUBGRF_SetPacketType(PACKET_TYPE_LORA);
    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
    modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
    modulationParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
    modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
    SUBGRF_SetModulationParams(&modulationParams);

    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH; // Mudar para CABEÇALHO FIXO
    packetParams.Params.LoRa.PayloadLength = PAYLOAD_SIZE;        // Definir o tamanho exato
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    // Configura as interrupções do rádio que queremos ouvir (TX_DONE e TIMEOUT)
    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

// Callback para os eventos de interrupção do rádio
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_TX_DONE:
            printf("LoRa TX Done.\r\n");
            tx_done = true; // Libera para a próxima transmissão
            // Para baixo consumo, aqui chamaríamos SUBGRF_SetSleep()
            break;
        case IRQ_RX_DONE:
            // Não estamos usando recepção LoRa na Placa 1
            break;
        case IRQ_RX_TX_TIMEOUT:
            printf("WARN: LoRa TX Timeout.\r\n");
            tx_done = true; // Libera para tentar de novo
            break;
        default:
            break;
    }
}

uint8_t calculate_checksum(uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) { checksum ^= data[i]; }
    return checksum;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
