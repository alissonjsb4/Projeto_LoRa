/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware da Placa 2 - Estação Base Receptora LoRa
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_nucleo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "radio_driver.h"
/* USER CODE END Includes */


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
// --- Definições do Payload (devem ser IDÊNTICAS às da Placa 1) ---
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    int32_t  latitude_raw;
    int32_t  longitude_raw;
    int32_t  altitude_raw;
    uint16_t voltage_mv;
    int8_t   radio_temp_c;
    uint8_t  sats_and_fix;
} LoRaPayload_t;

#define PAYLOAD_SIZE sizeof(LoRaPayload_t)

// --- Parâmetros LoRa (devem ser IDÊNTICOS aos da Placa 1) ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm (Aumentado para o máximo)
#define LORA_BANDWIDTH                              0         // 0: 125 kHz (Mantido)
#define LORA_SPREADING_FACTOR                       10        // SF10 (Aumentado de 7)
#define LORA_CODINGRATE                             4         // 4: 4/8 (Aumentado de 1, que era 4/5)
#define LORA_PREAMBLE_LENGTH                        8         // (Mantido)
/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t LoRa_rx_buffer[255]; // Buffer para guardar o pacote LoRa recebido
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ProcessAndPrintPayload(uint8_t* buffer, uint8_t size);
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
  MX_USART2_UART_Init();  // Apenas a UART de Debug para o PC
  MX_SUBGHZ_Init();
  // MX_DMA_Init(); // DMA foi desativado para a Placa 2

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);

  printf("\r\n--- Placa 2: Estacao Base LoRa Ativa ---\r\n");

  Radio_Init();
  printf("Radio LoRa inicializado em modo de recepcao.\r\n");

  printf("Pronto para receber pacotes LoRa...\r\n");
  // Coloca o rádio em modo de recepção contínua
  SUBGRF_SetRx(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // A mágica acontece nos callbacks de interrupção do rádio.
    // O processador pode dormir para economizar energia.
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
  // ... (Esta função é gerada pelo CubeMX e deve ser mantida como está) ...
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
// --- Nossas funções de apoio ---

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

    // Configura as interrupções do rádio para eventos de RECEPÇÃO
    SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
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

                BSP_LED_Toggle(LED_GREEN); // Pisca o LED para indicar recepção

                // Pega o payload e o status do rádio
                SUBGRF_GetPayload(LoRa_rx_buffer, &received_size, 255);
                SUBGRF_GetPacketStatus(&packetStatus);

                printf("Pacote LoRa Recebido! RSSI: %d dBm, SNR: %d\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);

                // Processa e imprime os dados
                ProcessAndPrintPayload(LoRa_rx_buffer, received_size);

                // IMPORTANTE: Coloca o rádio de volta em modo de recepção
                SUBGRF_SetRx(0);
            }
            break;
        case IRQ_CRC_ERROR:
            printf("WARN: Erro de CRC no pacote LoRa.\r\n");
            SUBGRF_SetRx(0); // Volta a escutar
            break;
        case IRQ_RX_TX_TIMEOUT:
            // Se houver um timeout de recepção, simplesmente volta a escutar
            SUBGRF_SetRx(0);
            break;
        default:
            break;
    }
}

void ProcessAndPrintPayload(uint8_t* buffer, uint8_t size) {
  if (size != PAYLOAD_SIZE) {
      printf("ERRO: Pacote com tamanho inesperado! Recebido: %d, Esperado: %d\r\n", size, (int)PAYLOAD_SIZE);
      return;
  }

  LoRaPayload_t* telemetry = (LoRaPayload_t*)buffer;

  float latitude = telemetry->latitude_raw / 10000000.0f;
  float longitude = telemetry->longitude_raw / 10000000.0f;
  float altitude_m = telemetry->altitude_raw / 1000.0f;

  bool gpsFixOK = (telemetry->sats_and_fix >> 7) & 0x01;
  uint8_t satCount = telemetry->sats_and_fix & 0x7F;

  printf("\r\n---[ PACOTE DE TELEMETRIA DECODIFICADO ]---\r\n");
  printf("  ID do Pacote:   %lu\r\n", telemetry->packet_id);
  printf("  Latitude:       %f\r\n", latitude);
  printf("  Longitude:      %f\r\n", longitude);
  printf("  Altitude:       %.2f m\r\n", altitude_m);
  printf("  Voltagem:       %u mV\r\n", telemetry->voltage_mv);
  printf("  Temp. Radio:    %d C\r\n", telemetry->radio_temp_c);
  printf("  Status GPS:     %s (Satelites: %u)\r\n", gpsFixOK ? "FIX OK" : "NO FIX", satCount);
  printf("-------------------------------------------\r\n\r\n");
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
