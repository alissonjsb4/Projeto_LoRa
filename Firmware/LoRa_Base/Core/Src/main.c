/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware da Placa 2 - Estação Base (Receptor e Transmissor de Comandos)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_nucleo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "radio_driver.h"
/* USER CODE END Includes */


/* USER CODE BEGIN 0 */
// Redireciona a saída do printf para a USART2
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Definições do Payload de Telemetria (deve ser IDÊNTICO ao da Placa 1) ---
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

// --- Definições do Payload de Comando (CONTRATO com a Placa 1) ---
typedef enum { CMD_SET_FREQ = 0x01, CMD_REBOOT = 0x02 } CommandType_t;
typedef struct __attribute__((packed)) { uint8_t command_type; uint32_t command_value; } CommandPayload_t;
#define COMMAND_SIZE sizeof(CommandPayload_t)

// --- Parâmetros LoRa (devem ser IDÊNTICOS aos da Placa 1) ---
#define RF_FREQUENCY          915000000
#define LORA_BANDWIDTH        0 // 125 kHz
#define LORA_SPREADING_FACTOR 10 // SF10 (robusto)
#define LORA_CODINGRATE       4  // 4/8 (robusto)
#define LORA_PREAMBLE_LENGTH  8
#define TX_OUTPUT_POWER       22 // Potência máxima

// Buffer para o DMA da USART2 (comandos do PC)
#define PC_COMMAND_BUFFER_SIZE 128
/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t LoRa_rx_buffer[255];
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
uint8_t pc_command_buffer[PC_COMMAND_BUFFER_SIZE];
uint16_t pc_cmd_old_pos = 0;
volatile bool tx_in_progress = false; // Flag para evitar colisões de transmissão
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ProcessAndPrintPayload(uint8_t* buffer, uint8_t size);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void ProcessSerialInput(void);
void ParseAndSendCommand(char* cmd_string);
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
  MX_USART2_UART_Init();
  MX_SUBGHZ_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  printf("\r\n--- Placa 2: Estacao Base LoRa (TX/RX) Ativa ---\r\n");
  printf("Firmware v2.0 - Comunicacao Bidirecional\r\n");
  Radio_Init();
  printf("Radio LoRa inicializado.\r\n");

  // Inicia recepção de comandos via UART (DMA)
  HAL_UART_Receive_DMA(&huart2, pc_command_buffer, PC_COMMAND_BUFFER_SIZE);

  // Coloca o rádio em modo de recepção contínua
  SUBGRF_SetRx(0);
  printf("Pronto para receber pacotes LoRa e comandos do PC...\r\n");
  printf("Comandos disponiveis:\r\n");
  printf("  SET_FREQ=<frequencia_Hz> - Configura frequencia da radiosonda\r\n");
  printf("  REBOOT                   - Reinicia a Placa 1\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessSerialInput();
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
  // ... (Código gerado pelo CubeMX, mantido como está) ...
}

/* USER CODE BEGIN 4 */
void ProcessSerialInput(void)
{
    static char line_buffer[PC_COMMAND_BUFFER_SIZE];
    static uint16_t line_pos = 0;

    uint16_t new_pos = PC_COMMAND_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    while(pc_cmd_old_pos != new_pos)
    {
        uint8_t current_char = pc_command_buffer[pc_cmd_old_pos];

        // Echo do caractere digitado
        HAL_UART_Transmit(&huart2, &current_char, 1, 100);

        if (current_char == '\r' || current_char == '\n')
        {
            if (line_pos > 0)
            {
                printf("\r\n");
                line_buffer[line_pos] = '\0';
                ParseAndSendCommand(line_buffer);
                line_pos = 0;
            }
        }
        else if (line_pos < PC_COMMAND_BUFFER_SIZE - 1)
        {
            line_buffer[line_pos++] = current_char;
        }

        pc_cmd_old_pos = (pc_cmd_old_pos + 1) % PC_COMMAND_BUFFER_SIZE;
    }
}

void ParseAndSendCommand(char* cmd_string)
{
    CommandPayload_t command_payload;
    char* value_str;

    // Converter para maiúsculas para facilitar comparação
    for(int i = 0; cmd_string[i]; i++) {
        if(cmd_string[i] >= 'a' && cmd_string[i] <= 'z') {
            cmd_string[i] = cmd_string[i] - 'a' + 'A';
        }
    }

    if (strncmp(cmd_string, "SET_FREQ=", 9) == 0)
    {
        value_str = cmd_string + 9;
        command_payload.command_value = strtoul(value_str, NULL, 10);
        command_payload.command_type = CMD_SET_FREQ;

        if(command_payload.command_value < 400000000 ||
           command_payload.command_value > 500000000)
        {
            printf("ERRO: Frequencia fora da faixa (400-500 MHz)\r\n");
            return;
        }

        printf("Enviando comando LoRa: SET_FREQ com valor: %lu Hz\r\n",
               command_payload.command_value);

        if(!tx_in_progress)
        {
            tx_in_progress = true;
            BSP_LED_On(LED_RED);
            SUBGRF_SendPayload((uint8_t*)&command_payload, COMMAND_SIZE, 0);
        }
        else
        {
            printf("WARN: Radio ocupado, tente novamente.\r\n");
        }
    }
    else if (strcmp(cmd_string, "REBOOT") == 0)
    {
        command_payload.command_type = CMD_REBOOT;
        command_payload.command_value = 0;

        printf("Enviando comando LoRa: REBOOT\r\n");

        if(!tx_in_progress)
        {
            tx_in_progress = true;
            BSP_LED_On(LED_RED);
            SUBGRF_SendPayload((uint8_t*)&command_payload, COMMAND_SIZE, 0);
        }
        else
        {
            printf("WARN: Radio ocupado, tente novamente.\r\n");
        }
    }
    else if (strcmp(cmd_string, "HELP") == 0)
    {
        printf("\r\n=== COMANDOS DISPONIVEIS ===\r\n");
        printf("SET_FREQ=<frequencia_Hz>\r\n");
        printf("REBOOT\r\n\r\n");
    }
    else
    {
        printf("ERRO: Comando desconhecido. Digite HELP para ver as opcoes.\r\n");
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
    // ... (configuração dos modulationParams) ...
    SUBGRF_SetModulationParams(&modulationParams);

    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    // CRÍTICO: Usar VARIABLE_LENGTH para suportar diferentes tamanhos de payload
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.PayloadLength = 255;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    // Habilita interrupções para TX, RX e erros
    SUBGRF_SetDioIrqParams(
        IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
        IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
        IRQ_RADIO_NONE,
        IRQ_RADIO_NONE
    );
}

void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_RX_DONE:
            {
                uint8_t received_size = 0;
                PacketStatus_t packetStatus;
                SUBGRF_GetPayload(LoRa_rx_buffer, &received_size, 255);
                SUBGRF_GetPacketStatus(&packetStatus);
                printf("\r\nPacote LoRa Recebido! RSSI: %d dBm, SNR: %d\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
                BSP_LED_Toggle(LED_GREEN);
                ProcessAndPrintPayload(LoRa_rx_buffer, received_size);
            }
            SUBGRF_SetRx(0);
            break;

        case IRQ_TX_DONE:
            printf("Comando LoRa enviado com sucesso.\r\n");
            tx_in_progress = false;
            BSP_LED_Off(LED_RED);
            SUBGRF_SetRx(0); // Volta a ouvir
            break;

        case IRQ_CRC_ERROR:
             printf("WARN: Erro de CRC no pacote LoRa.\r\n");
             SUBGRF_SetRx(0);
             break;
        case IRQ_RX_TX_TIMEOUT:
            printf("WARN: Timeout no radio.\r\n");
            tx_in_progress = false; // Libera a flag de TX
            BSP_LED_Off(LED_RED);
            SUBGRF_SetRx(0);
             break;

        default:
            break;
    }
}

void ProcessAndPrintPayload(uint8_t* buffer, uint8_t size) {
  if (size == PAYLOAD_SIZE) {
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
  } else {
      printf("INFO: Recebido pacote LoRa de tipo desconhecido (tamanho: %d)\r\n", size);
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
