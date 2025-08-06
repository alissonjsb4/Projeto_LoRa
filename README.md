# Sistema de Retransmissão de Telemetria LoRa para Radiosonda

Este repositório contém o firmware para um sistema de duas placas STM32 NUCLEO-WL55JC1 projetado para capturar, validar e retransmitir dados de uma radiosonda via LoRa.

## Arquitetura

O sistema é composto por dois nós principais:

1.  **Placa 1 (Nó de Campo / Transmissor):**
    * Localizada em `Firmware/Placa1_Tx_Radiosonda`.
    * **Função:** Conecta-se a uma radiosonda via UART (USART1 a 19200 baud). Utiliza DMA em modo circular para receber um fluxo de dados contínuo. Implementa uma máquina de estados para validar pacotes através de um `SYNC_WORD` e `checksum`. Pacotes válidos são então transmitidos via LoRa.

2.  **Placa 2 (Estação Base / Receptor):**
    * Localizada em `Firmware/Placa2_Rx_Estacao`.
    * **Função:** Atua como um gateway LoRa. Recebe os pacotes de telemetria enviados pela Placa 1. Decodifica a estrutura de dados e a envia de forma legível para um computador via porta serial virtual (USART2 a 115200 baud) para monitoramento.

## Parâmetros de Comunicação LoRa

Para garantir a comunicação, ambas as placas estão configuradas com os seguintes parâmetros robustos:

* **Frequência:** 915.0 MHz
* **Spreading Factor (SF):** 10
* **Bandwidth (BW):** 125 kHz
* **Coding Rate (CR):** 4/8
* **Potência de Transmissão:** 22 dBm
* **Modo de Pacote:** Tamanho Fixo (Cabeçalho Implícito)

## Hardware Necessário

* 2x Placas de desenvolvimento STM32 NUCLEO-WL55JC1
* 2x Antenas para 915 MHz
* 1x Radiosonda com saída serial (configurada para 19200 baud)
* Cabos e jumpers para conexão

## Como Compilar e Usar

1.  Clone este repositório.
2.  Abra o STM32CubeIDE e importe os dois projetos localizados na pasta `Firmware`.
3.  Compile cada projeto.
4.  Grave o firmware `Placa1_Tx_Radiosonda` no primeiro dispositivo e `Placa2_Rx_Estacao` no segundo.
5.  Conecte a Placa 1 à radiosonda e a Placa 2 a um computador com um terminal serial (ex: PuTTY, 115200 baud) para visualizar os dados recebidos.
