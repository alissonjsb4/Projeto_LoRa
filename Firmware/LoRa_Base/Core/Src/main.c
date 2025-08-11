/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware da Placa 2 - LoRa_Base (MASTER NODE)
  *                   Estação Base com Menu de Comandos
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

#define TELEMETRY_PAYLOAD_SIZE sizeof(LoRaPayload_t)
#define COMMAND_PAYLOAD_SIZE sizeof(CommandPayload_t)

// --- Parâmetros LoRa (IDÊNTICOS aos da Placa 1) ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm
#define LORA_BANDWIDTH                              0         // 0: 125 kHz
#define LORA_SPREADING_FACTOR                       10        // SF10
#define LORA_CODINGRATE                             4         // 4: 4/8
#define LORA_PREAMBLE_LENGTH                        8

// --- Timeouts ---
#define RESPONSE_TIMEOUT_MS                         5000      // 5 segundos
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t lora_rx_buffer[32]; // Buffer para receber respostas
uint8_t uart_rx_char; // Caractere recebido via UART
volatile bool aguardando_resposta = false; // Flag de controle de fluxo
volatile bool lora_tx_busy = false; // Flag de controle de transmissão
uint32_t command_sequence = 0; // Contador de sequência dos comandos
uint32_t response_timer = 0; // Timer para timeout de resposta
CommandType_t last_command_sent = CMD_ACK; // Último comando enviado

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ShowMenu(void);
void ProcessUserInput(void);
void SendCommand(CommandType_t command);
void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size);
void ProcessAckResponse(uint8_t* buffer, uint8_t size);
void CheckResponseTimeout(void);
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
  MX_USART2_UART_Init();  // UART de Debug/Menu para o PC
  MX_SUBGHZ_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);

  printf("\r\n--- LoRa_Base (MASTER NODE) ---\r\n");
  printf("Estacao Base com Menu de Comandos\r\n");

  Radio_Init();
  printf("Radio LoRa inicializado em modo MASTER.\r\n");

  ShowMenu();

  // Inicia recepção não-bloqueante da UART para o menu
  HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Verifica timeout de resposta
    if (aguardando_resposta) {
      CheckResponseTimeout();
    }

    // O processamento acontece nos callbacks de interrupção
    // O processador pode entrar em modo sleep para economizar energia
    if (!aguardando_resposta && !lora_tx_busy) {
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
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

void ShowMenu(void)
{
    printf("\r\n=== MENU DE COMANDOS ===\r\n");
    printf("1 - Solicitar Telemetria\r\n");
    printf("3 - Mostrar este menu\r\n");
    printf("Digite o numero do comando: ");
}

void ProcessUserInput(void)
{
    switch (uart_rx_char)
    {
        case '1':
            printf("1\r\n");
            if (!aguardando_resposta && !lora_tx_busy) {
                printf("Solicitando telemetria...\r\n");
                SendCommand(CMD_REQUEST_TELEMETRY);
            } else {
                printf("ERRO: Operacao em andamento, aguarde.\r\n");
            }
            break;

        case '3':
            printf("3\r\n");
            ShowMenu();
            break;

        default:
            printf("%c\r\n", uart_rx_char);
            printf("Comando invalido! Digite 1, 2 ou 3.\r\n");
            break;
    }

    // Reinicia a recepção da UART
    HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
}

void SendCommand(CommandType_t command)
{
    if (lora_tx_busy || aguardando_resposta) {
        printf("WARN: Radio ocupado ou aguardando resposta.\r\n");
        return;
    }

    CommandPayload_t cmd_packet;
    cmd_packet.command = command;
    cmd_packet.sequence_number = ++command_sequence;
    memset(cmd_packet.reserved, 0, sizeof(cmd_packet.reserved));

    lora_tx_busy = true;
    last_command_sent = command;

    // Configura o rádio para transmitir comandos
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    printf("Transmitindo comando %d (seq: %lu)...\r\n", command, cmd_packet.sequence_number);
    SUBGRF_SendPayload((uint8_t*)&cmd_packet, COMMAND_PAYLOAD_SIZE, 0);
}

void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size)
{
    if (size != TELEMETRY_PAYLOAD_SIZE) {
        printf("ERRO: Telemetria com tamanho inválido: %d\r\n", size);
        return;
    }

    LoRaPayload_t* telemetry = (LoRaPayload_t*)buffer;

    float latitude = telemetry->latitude_raw / 10000000.0f;
    float longitude = telemetry->longitude_raw / 10000000.0f;
    float altitude_m = telemetry->altitude_raw / 1000.0f;

    bool gpsFixOK = (telemetry->sats_and_fix >> 7) & 0x01;
    uint8_t satCount = telemetry->sats_and_fix & 0x7F;

    printf("\r\n===[ TELEMETRIA RECEBIDA ]===\r\n");
    printf("  ID do Pacote:   %lu\r\n", telemetry->packet_id);
    printf("  Latitude:       %.7f\r\n", latitude);
    printf("  Longitude:      %.7f\r\n", longitude);
    printf("  Altitude:       %.2f m\r\n", altitude_m);
    printf("  Voltagem:       %u mV\r\n", telemetry->voltage_mv);
    printf("  Temp. Radio:    %d C\r\n", telemetry->radio_temp_c);
    printf("  Status GPS:     %s (Sats: %u)\r\n", gpsFixOK ? "FIX OK" : "NO FIX", satCount);
    printf("==============================\r\n");

    // Retorna ao menu
    ShowMenu();
}

void ProcessAckResponse(uint8_t* buffer, uint8_t size)
{
    if (size != COMMAND_PAYLOAD_SIZE) {
        printf("ERRO: ACK com tamanho inválido: %d\r\n", size);
        return;
    }

    CommandPayload_t* ack = (CommandPayload_t*)buffer;

    if (ack->command == CMD_ACK) {
        printf("ACK recebido para seq: %lu\r\n", ack->sequence_number);
        if (ack->sequence_number == command_sequence) {
            printf("Comando executado com sucesso!\r\n");
        } else {
            printf("WARN: Sequencia do ACK nao confere.\r\n");
        }
    }

    // Retorna ao menu
    ShowMenu();
}

void CheckResponseTimeout(void)
{
    if (HAL_GetTick() - response_timer > RESPONSE_TIMEOUT_MS) {
        printf("TIMEOUT: Nenhuma resposta recebida em %d ms.\r\n", RESPONSE_TIMEOUT_MS);
        aguardando_resposta = false;

        // Reconfigura para modo de recepção padrão (comandos)
        PacketParams_t packetParams;
        packetParams.PacketType = PACKET_TYPE_LORA;
        packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
        packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
        packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
        packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
        packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
        SUBGRF_SetPacketParams(&packetParams);

        SUBGRF_SetRx(0); // Volta para recepção contínua

        ShowMenu();
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

    // Configuração inicial para receber ACKs (comandos têm tamanho menor)
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    // Configura interrupções para TX e RX
    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    // Inicia em modo de recepção contínua (aguardando possíveis respostas)
    SUBGRF_SetRx(0);
}

void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_TX_DONE:
            printf("LoRa TX concluida.\r\n");
            lora_tx_busy = false;

            // Se era um comando, agora aguarda resposta
            if (last_command_sent == CMD_REQUEST_TELEMETRY) {
                printf("Aguardando telemetria...\r\n");
                aguardando_resposta = true;
                response_timer = HAL_GetTick();

                // Configura para receber telemetria (tamanho maior)
                PacketParams_t packetParams;
                packetParams.PacketType = PACKET_TYPE_LORA;
                packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
                packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
                packetParams.Params.LoRa.PayloadLength = TELEMETRY_PAYLOAD_SIZE;
                packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
                packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
                SUBGRF_SetPacketParams(&packetParams);

            }

                // Mantém configuração para comandos (ACK tem mesmo tamanho)


            // Volta para modo de recepção
            SUBGRF_SetRx(0);
            break;

        case IRQ_RX_DONE:
            {
                uint8_t received_size = 0;
                PacketStatus_t packetStatus;

                BSP_LED_Toggle(LED_GREEN);

                SUBGRF_GetPayload(lora_rx_buffer, &received_size, sizeof(lora_rx_buffer));
                SUBGRF_GetPacketStatus(&packetStatus);

                printf("Resposta LoRa recebida! RSSI: %d dBm, SNR: %d, Size: %d\r\n",
                       packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt, received_size);

                aguardando_resposta = false;

                // Processa resposta baseado no tamanho
                if (received_size == TELEMETRY_PAYLOAD_SIZE) {
                    ProcessTelemetryResponse(lora_rx_buffer, received_size);
                } else if (received_size == COMMAND_PAYLOAD_SIZE) {
                    ProcessAckResponse(lora_rx_buffer, received_size);
                } else {
                    printf("ERRO: Resposta com tamanho inesperado: %d\r\n", received_size);
                    ShowMenu();
                }

                // Reconfigura para comandos (tamanho padrão menor)
                PacketParams_t packetParams;
                packetParams.PacketType = PACKET_TYPE_LORA;
                packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
                packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
                packetParams.Params.LoRa.PayloadLength = COMMAND_PAYLOAD_SIZE;
                packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
                packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
                SUBGRF_SetPacketParams(&packetParams);

                SUBGRF_SetRx(0); // Volta para recepção contínua
            }
            break;

        case IRQ_CRC_ERROR:
            printf("WARN: Erro de CRC na resposta LoRa.\r\n");
            aguardando_resposta = false;
            SUBGRF_SetRx(0);
            ShowMenu();
            break;

        case IRQ_RX_TX_TIMEOUT:
            printf("WARN: LoRa timeout.\r\n");
            lora_tx_busy = false;
            aguardando_resposta = false;
            SUBGRF_SetRx(0);
            ShowMenu();
            break;

        default:
            break;
    }
}

// Callback da UART para processar entrada do usuário
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ProcessUserInput();
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
