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
#include "radio_driver.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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


#define TELEMETRY_PAYLOAD_SIZE sizeof(LoRaPayload_t)
//#define COMMAND_PAYLOAD_SIZE sizeof(CommandPacket_t)

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

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t lora_rx_buffer[TELEMETRY_PAYLOAD_SIZE]; // Buffer para receber respostas
uint8_t uart_rx_char; // Caractere recebido via UART
volatile bool aguardando_resposta = false; // Flag de controle de fluxo
volatile bool lora_tx_busy = false; // Flag de controle de transmissão
uint32_t command_sequence = 0; // Contador de sequência dos comandos
uint32_t response_timer = 0; // Timer para timeout de resposta
CommandType_t last_command_sent = CMD_TYPE_REQUEST; // Último comando enviado

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ShowMenu(void);
void ProcessUserInput(void);
void SendCommand(CommandType_t type, ParameterID_t param, uint8_t* data, uint8_t len);
void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size);
void ProcessAckResponse(uint8_t* buffer, uint8_t size);
void CheckResponseTimeout(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redireciona a saída do printf para a USART2 (nossa porta de debug para o PC)
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
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
  MX_DMA_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
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

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
    printf("2 - Realizar uma leitura ou escrita\r\n");
    printf("3 - Solicitar a execução de um comando\r\n");
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
                SendCommand(CMD_TYPE_REQUEST, REQUEST_TELEMETRY_PACKET, NULL, 0);
            } else {
                printf("ERRO: Operacao em andamento, aguarde.\r\n");
            }
            break;

        case '2':
            printf("2\r\n");
            if (!aguardando_resposta && !lora_tx_busy) {
                printf("Solicitando execução de comando da radiossonda...\r\n");
                SendCommand(CMD_TYPE_EXECUTE, ACTION_RESET_MCU, NULL, 0);
            } else {
				printf("ERRO: Operacao em andamento, aguarde.\r\n");
			}
            break;

        case '3':
            printf("3\r\n");
            if (!aguardando_resposta && !lora_tx_busy) {
                printf("Solicitando leitura/escrita da radiossonda...\r\n");
                SendCommand(CMD_TYPE_WRITE, 0x00, lora_rx_buffer, 100); //eventualmente deverá ser outra função ou uma função adaptada para receber o parâmetro de escrita ou leitura
            } else {
                printf("ERRO: Operacao em andamento, aguarde.\r\n");
            }
            break;

        default:
            printf("%c\r\n", uart_rx_char);
            printf("Comando invalido! Digite 1, 2 ou 3.\r\n");
            break;
    }

    // Reinicia a recepção da UART
    HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
}

void SendCommand(CommandType_t type, ParameterID_t param, uint8_t* data, uint8_t len) {
    // 1. Inicializa a struct inteira com zeros.
    // Isso resolve o aviso de "uninitialized" e já limpa o payload_data.
    CommandPacket_t cmd_packet = {0};

    // 2. Preenche o cabeçalho do pacote com os valores corretos.
    cmd_packet.sequence_number = ++command_sequence; // Supondo que command_sequence é uma var global
    cmd_packet.command_type = type;
    cmd_packet.parameter_id = param;
    cmd_packet.payload_len = len;

    // 3. Copia os dados do payload, se houver.
    // A validação de 'len' garante que não copiaremos lixo de memória.
    if (data!= NULL && len > 0 && len <= MAX_PAYLOAD_DATA_SIZE) {
        memcpy(cmd_packet.payload_data, data, len);
    }

    // A chamada 'memset' anterior foi removida por ser desnecessária e perigosa.

    lora_tx_busy = true;
    last_command_sent = type; // Corrigido para usar o 'type' passado para a função

    // Configura os parâmetros do rádio (seu código aqui está correto)
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = MAX_PACKET_SIZE; // Sempre envia o pacote de tamanho máximo
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    printf("Transmitindo comando %d (seq: %lu)...\r\n", type, cmd_packet.sequence_number);

    // Envia a struct inteira. O receptor irá ignorar o padding.
    SUBGRF_SendPayload((uint8_t*)&cmd_packet, MAX_PACKET_SIZE, 0);
}

void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size)
{
    // 1. Validação do tamanho do pacote recebido. Esta parte está perfeita.
    if (size!= TELEMETRY_PAYLOAD_SIZE) {
        printf("ERRO: Telemetria com tamanho de pacote inválido: %d (esperado: %d)\r\n", size, TELEMETRY_PAYLOAD_SIZE);
        return;
    }

    // 2. Cria uma variável local para armazenar a telemetria de forma segura.
    LoRaPayload_t telemetry_data;

    // 3. Copia de forma segura APENAS os bytes que correspondem à telemetria real
    // do início do buffer maior para a nossa variável local.
    // Isso ignora efetivamente o padding no final do buffer.
    memcpy(&telemetry_data, buffer, sizeof(LoRaPayload_t));

    // 4. Todas as operações agora usam a cópia local limpa e segura.
    float latitude = telemetry_data.latitude_raw / 10000000.0f;
    float longitude = telemetry_data.longitude_raw / 10000000.0f;
    float altitude_m = telemetry_data.altitude_raw / 1000.0f;

    bool gpsFixOK = (telemetry_data.sats_and_fix >> 7) & 0x01;
    uint8_t satCount = telemetry_data.sats_and_fix & 0x7F;

    printf("\r\n======\r\n");
    printf("  ID do Pacote:   %lu\r\n", telemetry_data.packet_id);
    printf("  Latitude:       %.7f\r\n", latitude);
    printf("  Longitude:      %.7f\r\n", longitude);
    printf("  Altitude:       %.2f m\r\n", altitude_m);
    printf("  Voltagem:       %u mV\r\n", telemetry_data.voltage_mv);
    printf("  Temp. Radio:    %d C\r\n", telemetry_data.radio_temp_c);
    printf("  Status GPS:     %s (Sats: %u)\r\n", gpsFixOK? "FIX OK" : "NO FIX", satCount);
    printf("==============================\r\n");

    // Retorna ao menu
//    ShowMenu();
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
        packetParams.Params.LoRa.PayloadLength = MAX_PACKET_SIZE;
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
    packetParams.Params.LoRa.PayloadLength = MAX_PACKET_SIZE;
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

            // Após QUALQUER transmissão, a configuração de recepção é SEMPRE a mesma.
            // Não há mais necessidade de alternar configurações.

            // Configura o rádio para receber um pacote de tamanho fixo e máximo.
            PacketParams_t packetParams;
            packetParams.PacketType = PACKET_TYPE_LORA;
            packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
            packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH; // Mantemos o tamanho fixo
            packetParams.Params.LoRa.PayloadLength = TELEMETRY_PAYLOAD_SIZE;      // AJUSTE CRÍTICO: Sempre esperamos o tamanho máximo
            packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
            packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
            SUBGRF_SetPacketParams(&packetParams);

            // Se enviamos um comando que espera resposta, iniciamos o timer.
            if (last_command_sent == CMD_TYPE_REQUEST || last_command_sent == CMD_TYPE_READ) {
                printf("Aguardando resposta...\r\n");
                aguardando_resposta = true;
                response_timer = HAL_GetTick();
            }

            // Volta para modo de recepção
            SUBGRF_SetRx(0);
            break;

        case IRQ_RX_DONE:
            {
                uint8_t received_size = 0;
                PacketStatus_t packetStatus;

                BSP_LED_Toggle(LED_GREEN);
                aguardando_resposta = false;

                SUBGRF_GetPayload(lora_rx_buffer, &received_size, TELEMETRY_PAYLOAD_SIZE);
                SUBGRF_GetPacketStatus(&packetStatus);

                printf("Resposta LoRa recebida! RSSI: %d dBm, SNR: %d, Size: %d\r\n",
                       packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt, received_size);

                // AJUSTE CRÍTICO: A validação agora é sobre o tamanho máximo esperado.
                if (received_size == TELEMETRY_PAYLOAD_SIZE) {
                    // O pacote tem o tamanho correto. Agora, podemos inspecionar seu conteúdo
                    // para decidir como processá-lo.
                    // Ex: ProcessTelemetryResponse(lora_rx_buffer, received_size);
                    // Ex: ProcessAckResponse(lora_rx_buffer, received_size);
                    // A lógica de qual função chamar virá da análise do conteúdo do buffer,
                    // não mais do seu tamanho.
                	ProcessTelemetryResponse(lora_rx_buffer, received_size);
                    ShowMenu();
                } else {
                    printf("ERRO: Resposta com tamanho inesperado: %d (esperado: %d)\r\n", received_size, TELEMETRY_PAYLOAD_SIZE);
                    ShowMenu();
                }

                // NÃO é mais necessário reconfigurar os parâmetros do pacote aqui.
                // A configuração para recepção já está correta e não muda.
                SUBGRF_SetRx(0); // Apenas volte a ouvir.
            }
            break;

        case IRQ_CRC_ERROR:
            printf("WARN: Erro de CRC na resposta LoRa.\r\n");
            aguardando_resposta = false;
            SUBGRF_SetRx(0);
            ShowMenu();
            break;

        case IRQ_RX_TX_TIMEOUT:
            // Se o timeout ocorreu enquanto esperávamos uma resposta.
            if (aguardando_resposta) {
                printf("ERRO: Timeout! Nenhuma resposta recebida.\r\n");
            } else {
                printf("WARN: LoRa timeout.\r\n");
            }
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
