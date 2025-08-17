#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

// Em um arquivo de cabeçalho compartilhado (ex: protocol.h)

#include <stdint.h>
// Enum para os 4 tipos de AÇÃO do comando
typedef enum {
    CMD_TYPE_READ    = 0x01, // Ler o valor de um parâmetro da RS41
    CMD_TYPE_WRITE   = 0x02, // Escrever um novo valor em um parâmetro da RS41
    CMD_TYPE_EXECUTE = 0x03, // Mandar a RS41 executar uma ação (ex: reset)
    CMD_TYPE_REQUEST = 0x04  // Solicitar um tipo específico de resposta (ex: telemetria)
} CommandType_t;

// Enum para os ALVOS do comando (parâmetros e ações)
typedef enum {
    // Parâmetros que podem ser lidos/escritos
    PARAM_DEEP_SLEEP_INTERVAL = 0x10, // uint32_t
    PARAM_TX_INTERVAL         = 0x11, // uint16_t

    // Ações que podem ser executadas
    ACTION_RESET_MCU          = 0x80,

    // Requisições que podem ser feitas
    REQUEST_TELEMETRY_PACKET  = 0xA0
} ParameterID_t;

#define MAX_PAYLOAD_DATA_SIZE 16 // Define um tamanho máximo para o campo de dados

typedef struct __attribute__((packed)) {
    uint32_t      sequence_number;    // Número de sequência para rastreamento
    CommandType_t command_type;       // Ação a ser tomada (READ, WRITE, EXECUTE, REQUEST)
    ParameterID_t parameter_id;       // Alvo da ação (qual parâmetro ou ação)
    uint8_t       payload_len;        // Comprimento dos dados no campo 'payload_data'
    uint8_t*       payload_data; // Buffer para os dados
} CommandPacket_t;

#define MAX_PACKET_SIZE (sizeof(CommandPacket_t))

#endif
