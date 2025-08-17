# STM32 LoRa Telemetry Relay System

A robust firmware project for a dual-node STM32 NUCLEO-WL55JC1 system designed to capture, validate, and retransmit data from a radiosonde via LoRa for long-range telemetry. This project was developed as a technical study in embedded systems and wireless communication.

## Key Features

- [cite_start]**Robust Hardware Platform:** Utilizes two **STM32 NUCLEO-WL55JC1** boards, which integrate a powerful microcontroller and a LoRa transceiver on a single chip[cite: 712, 726].
- [cite_start]**Efficient Firmware:** Employs **DMA in circular mode** for non-blocking UART data reception from the radiosonde, freeing up the CPU for other tasks[cite: 713, 719, 753].
- [cite_start]**Reliable Data Parsing:** A **Finite State Machine (FSM)** validates incoming data packets using a `SYNC_WORD` and `checksum` to ensure data integrity before retransmission[cite: 720].
- [cite_start]**Low-Power, Event-Driven Receiver:** The base station is fully interrupt-driven, operating in a low-power sleep mode until a LoRa packet is received, making it highly efficient[cite: 816, 817].
- [cite_start]**Layered Software Architecture:** The firmware is built on STMicroelectronics' **HAL (Hardware Abstraction Layer)** and **BSP (Board Support Package)**, ensuring clean, portable, and maintainable code[cite: 713, 736].

## Project Structure

The repository is organized into a clean, modular structure:


/
├── README.md
├── src/
│   ├── Field_Node/
│   ├── Base_Station/
│   └── common/
│       └── protocol.h
├── docs/
│   ├── Relatorio_Tecnico.pdf
│   └── images/
└── .gitignore


## System Architecture

The system consists of two main nodes that work in tandem:

1.  **Node 1: Field Node (Transmitter)**
    * [cite_start]Connects directly to a radiosonde via `USART1` (19200 baud) to capture a continuous stream of telemetry data[cite: 719].
    * [cite_start]Parses the data stream using an FSM to identify and validate complete data packets[cite: 720].
    * [cite_start]Valid packets, verified by a checksum, are transmitted wirelessly using the onboard LoRa radio[cite: 720, 721].

2.  **Node 2: Base Station (Receiver)**
    * [cite_start]Acts as a LoRa gateway, remaining in a continuous reception mode[cite: 722].
    * [cite_start]Upon receiving a valid LoRa packet, it decodes the binary payload into human-readable data[cite: 723, 848].
    * [cite_start]Sends the formatted data to a connected PC via a Virtual COM Port (`USART2` at 115200 baud) for real-time monitoring[cite: 723].

## LoRa Communication Parameters

To ensure a robust and long-range communication link, the following LoRa parameters were configured for both nodes:

| Parameter               | Value      |
|:------------------------|:-----------|
| **Frequency** | 915.0 MHz  |
| **Spreading Factor (SF)** | 10         |
| **Bandwidth (BW)** | 125 kHz    |
| **Coding Rate (CR)** | 4/8        |
| **Transmission Power** | 22 dBm     |
| **Packet Mode** | Fixed Size (Implicit Header) |

## How to Use

1.  **Hardware Required:**
    * [cite_start]2x STM32 NUCLEO-WL55JC1 development boards [cite: 726]
    * 2x 915 MHz Antennas
    * A radiosonde or other serial data source (19200 baud)

2.  **Setup:**
    * Clone this repository.
    * [cite_start]Open **STM32CubeIDE** and import the two projects located in `src/Field_Node/` and `src/Base_Station/`[cite: 727].
    * Compile both projects.
    * Flash the `Field_Node` firmware onto the first board and `Base_Station` onto the second.

3.  **Execution:**
    * Connect the Field Node to the radiosonde.
    * [cite_start]Connect the Base Station to a PC and open a serial terminal (like PuTTY or Tera Term) on the corresponding COM port at 115200 baud to view the received telemetry data[cite: 723, 734].

## Validation & Results

The system was fully tested, demonstrating a successful end-to-end communication link. The following are sample outputs captured from the serial terminals of both nodes during a test run.

**Transmitter (Field Node) Terminal Output:**

Checksum OK. Pacote da radiosonda validado.
Transmitindo pacote LoRa ID: 1
LoRa TX Done.

[cite_start][cite: 871]

**Receiver (Base Station) Terminal Output:**

Pacote LoRa Recebido! RSSI: -50 dBm, SNR: 9

---[ PACOTE DE TELEMETRIA DECODIFICADO ]---
ID do Pacote: 1
Latitude:     -3.740123
Longitude:    -38.570456
Altitude:     50.12 m
Voltagem:     3300 mV
Temp. Radio:  25 C
Status GPS:   FIX OK (Satelites: 8)

[cite_start][cite: 876, 878, 880, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891]

## Future Work & Roadmap

This project serves as a solid foundation for a field-ready telemetry system. Recommended next steps include:

- [ ] [cite_start]**Power Optimization:** Implement the final low-power sleep modes (`SUBGRF_SetSleep`) on the Field Node to maximize battery life[cite: 899].
- [ ] [cite_start]**Data Visualization:** Develop a GUI or logging script on the PC to better visualize, store, and analyze the incoming telemetry data[cite: 900].
- [ ] [cite_start]**Field Testing:** Conduct real-world, open-field range tests to validate the performance and maximum range of the LoRa link[cite: 901].
- [ ] **Bi-directional Communication:** Implement a command system to send configuration messages from the Base Station back to the Field Node.

## Authors & Acknowledgments

This project was developed by:
- Alisson Jaime Sales Barros

Special thanks to the STMicroelectronics HAL/BSP driver teams for providing a robust software foundation.
