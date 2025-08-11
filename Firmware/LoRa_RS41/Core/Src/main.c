/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware da Placa 1 - LoRa_RS41 (SLAVE NODE)
  *                   Parser da Radiosonda + Resposta a Comandos LoRa
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
// --- Definições do Payload de Telemetria ---
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    int32_t  latitude_raw;
    int32_t  longitude_raw;
    int32_t  altitude_raw;
    uint16_t voltage_mv;
    int8_t   radio_temp_c;
    uint8_t  sats_and_fix;
} LoRaPayload_t;

// --- Definições do Comando ---
typedef enum {
    CMD_REQUEST_TELEMETRY = 0x01,
    CMD_ACK = 0x03
} CommandType_t;

typedef struct __attribute__((packed)) {
    CommandType_t command;
    uint32_t sequence_number;
    uint8_t reserved[3];
} CommandPayload_t;

// --- Parser da Radiosonda ---
const uint8_t SYNC_WORD = 0xAA;
#define TELEMETRY_PAYLOAD_SIZE sizeof(LoRaPayload_t)
#define COMMAND_PAYLOAD_SIZE sizeof(CommandPayload_t)

enum ParserState { AWAITING_SYNC, RECEIVING_PAYLOAD, AWAITING_CHECKSUM };

#define RADIOSONDE_UART_BUFFER_SIZE 256

// --- Parâmetros LoRa ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm
#define LORA_BANDWIDTH                              0         // 0: 125 kHz
#define LORA_SPREADING_FACTOR                       10        // SF10
#define LORA_CODINGRATE                             4         // 4: 4/8
#define LORA_PREAMBLE_LENGTH                        8
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// --- Variáveis do Parser da Radiosonda ---
uint8_t radiosonde_rx_buffer[RADIOSONDE_UART_BUFFER_SIZE];
uint16_t old_pos = 0;
uint8_t payloadBuffer[TELEMETRY_PAYLOAD_SIZE];
int byteCounter = 0;
enum ParserState currentState = AWAITING_SYNC;

// --- Variáveis da Comunicação LoRa ---
uint8_t lora_rx_buffer[32]; // Buffer para receber comandos
LoRaPayload_t latest_telemetry; // Última telemetria válida da radiosonda
volatile bool telemetry_available = false; // Flag indicando se há telemetria válida
volatile bool lora_tx_busy = false; // Flag para controlar transmissão LoRa

// Array de lookup para Bandwidth
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ProcessByte(uint8_t receivedByte);
void ProcessLoRaCommand(uint8_t* buffer, uint8_t size);
void SendTelemetryResponse(void);
void SendAckResponse(uint32_t sequence_number);
uint8_t calculate_checksum(uint8_t* data, int length);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
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

  printf("\r\n--- LoRa_RS41 (SLAVE NODE) ---\r\n");
  printf("Parser da Radiosonda + Resposta a Comandos LoRa\r\n");

  Radio_Init();
  printf("Radio LoRa inicializado em modo SLAVE.\r\n");

  // Inicia a recepção via DMA na USART1 (radiosonda)
  HAL_UART_Receive_DMA(&huart1, radiosonde_rx_buffer, RADIOSONDE_UART_BUFFER_SIZE);
  printf("Parser da radiosonda ativo na USART1.\r\n");

  // Coloca o rádio LoRa em modo de recepção contínua (aguardando comandos)
  SUBGRF_SetRx(0); // Timeout 0 = recepção contínua
  printf("Aguardando comandos LoRa da estacao base...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // --- Processa dados da radiosonda via DMA ---
    uint16_t new_pos = RADIOSONDE_UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (new_pos != old_pos)
    {
      if (new_pos > old_pos) {
        for (int i = old_pos; i < new_pos; i++) {
          ProcessByte(radiosonde_rx_buffer[i]);
        }
      } else { // Wrap-around
        for (int i = old_pos; i < RADIOSONDE_UART_BUFFER_SIZE; i++) {
          ProcessByte(radiosonde_rx_buffer[i]);
        }
        for (int i = 0; i < new_pos; i++) {
          ProcessByte(radiosonde_rx_buffer[i]);
        }
      }
      old_pos = new_pos;
    }

    // O processamento de comandos LoRa acontece nos callbacks de interrupção
    // O processador pode entrar em modo sleep para economizar energia
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
// --- Implementação das Funções ---

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
      if (byteCounter < TELEMETRY_PAYLOAD_SIZE) {
        payloadBuffer[byteCounter++] = receivedByte;
      }
      if (byteCounter >= TELEMETRY_PAYLOAD_SIZE) {
        currentState = AWAITING_CHECKSUM;
      }
      break;

    case AWAITING_CHECKSUM:
      {
        uint8_t receivedChecksum = receivedByte;
        uint8_t calculatedChecksum = calculate_checksum(payloadBuffer, TELEMETRY_PAYLOAD_SIZE);

        if (receivedChecksum == calculatedChecksum) {
          // Atualiza a telemetria mais recente
          memcpy(&latest_telemetry, payloadBuffer, TELEMETRY_PAYLOAD_SIZE);
          telemetry_available = true;

          printf("Telemetria atualizada - ID: %lu\r\n", latest_telemetry.packet_id);
          BSP_LED_Toggle(LED_GREEN);
        } else {
          printf("Falha no Checksum radiosonda! Esperado: 0x%02X, Recebido: 0x%02X\r\n",
                 calculatedChecksum, receivedChecksum);
        }
        currentState = AWAITING_SYNC;
      }
      break;
  }
}

void ProcessLoRaCommand(uint8_t* buffer, uint8_t size)
{
    if (size != COMMAND_PAYLOAD_SIZE) {
        printf("ERRO: Comando LoRa com tamanho inválido: %d\r\n", size);
        return;
    }

    CommandPayload_t* command = (CommandPayload_t*)buffer;

    printf("Comando recebido: %d, Seq: %lu\r\n", command->command, command->sequence_number);

    switch (command->command)
    {
        case CMD_REQUEST_TELEMETRY:
            printf("Processando pedido de telemetria...\r\n");
            SendTelemetryResponse();
            break;

        default:
            printf("WARN: Comando desconhecido: %d\r\n", command->command);
            break;
    }
}

void SendTelemetryResponse(void)
{
    if (lora_tx_busy) {
        printf("WARN: Radio ocupado, resposta descartada.\r\n");
        return;
    }

    if (!telemetry_available) {
        printf("WARN: Nenhuma telemetria disponível.\r\n");
        return;
    }

    lora_tx_busy = true;
    printf("Enviando telemetria - ID: %lu\r\n", latest_telemetry.packet_id);

    // Reconfigura para telemetria (tamanho diferente do comando)
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = TELEMETRY_PAYLOAD_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    SUBGRF_SendPayload((uint8_t*)&latest_telemetry, TELEMETRY_PAYLOAD_SIZE, 0);
}

void SendAckResponse(uint32_t sequence_number)
{
    if (lora_tx_busy) {
        printf("WARN: Radio ocupado, ACK descartado.\r\n");
        return;
    }

    CommandPayload_t ack_response;
    ack_response.command = CMD_ACK;
    ack_response.sequence_number = sequence_number;
    memset(ack_response.reserved, 0, sizeof(ack_response.reserved));

    lora_tx_busy = true;
    printf("Enviando ACK para seq: %lu\r\n", sequence_number);

    // Reconfigura para comando (tamanho diferente da telemetria)
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    SUBGRF_SendPayload((uint8_t*)&ack_response, COMMAND_PAYLOAD_SIZE, 0);
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

    // Configuração inicial para receber comandos
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE; // Inicialmente configurado para comandos
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    // Configura interrupções para RX e TX
    SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_TX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_RX_DONE | IRQ_TX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{

    switch (radioIrq)
    {
        case IRQ_RX_DONE:
            {
                uint8_t received_size = 0;
                PacketStatus_t packetStatus;

                SUBGRF_GetPayload(lora_rx_buffer, &received_size, sizeof(lora_rx_buffer));
                SUBGRF_GetPacketStatus(&packetStatus);

                printf("Comando LoRa recebido! RSSI: %d dBm, SNR: %d, Size: %d\r\n",
                       packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt, received_size);




                ProcessLoRaCommand(lora_rx_buffer, received_size);
            }
            break;

        case IRQ_TX_DONE:
            printf("LoRa TX concluida.\r\n");
            lora_tx_busy = false;



            // Reconfigura para receber comandos novamente
            PacketParams_t packetParams;
            packetParams.PacketType = PACKET_TYPE_LORA;
            packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
            packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
            packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
            packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
            packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
            SUBGRF_SetPacketParams(&packetParams);

            // Volta para modo de recepção contínua
            SUBGRF_SetRx(0);
            break;

        case IRQ_CRC_ERROR:
            printf("WARN: Erro de CRC no comando LoRa.\r\n");
            // Reconfigura para comandos e volta a escutar
            PacketParams_t packetParamsCrc;
            packetParamsCrc.PacketType = PACKET_TYPE_LORA;
            packetParamsCrc.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
            packetParamsCrc.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
            packetParamsCrc.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
            packetParamsCrc.Params.LoRa.CrcMode = LORA_CRC_ON;
            packetParamsCrc.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
            SUBGRF_SetPacketParams(&packetParamsCrc);
            SUBGRF_SetRx(0);
            break;

        case IRQ_RX_TX_TIMEOUT:
            printf("WARN: LoRa timeout.\r\n");
            lora_tx_busy = false;
            // Reconfigura para comandos e volta a escutar
            PacketParams_t packetParamsTimeout;
            packetParamsTimeout.PacketType = PACKET_TYPE_LORA;
            packetParamsTimeout.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
            packetParamsTimeout.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
            packetParamsTimeout.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
            packetParamsTimeout.Params.LoRa.CrcMode = LORA_CRC_ON;
            packetParamsTimeout.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
            SUBGRF_SetPacketParams(&packetParamsTimeout);
            SUBGRF_SetRx(0);
            break;

        default:
            break;
    }
}

uint8_t calculate_checksum(uint8_t* data, int length)
{
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
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
