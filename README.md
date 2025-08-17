# STM32 LoRa Telemetry Relay System

[cite_start]A robust firmware project for a dual-node STM32 NUCLEO-WL55JC1 system designed to capture, validate, and retransmit data from a radiosonde via LoRa for long-range telemetry[cite: 696, 700]. This project was developed as a technical study in embedded systems and wireless communication.

## Key Features

- [cite_start]**Robust Hardware Platform:** Utilizes two **STM32 NUCLEO-WL55JC1** development boards, which integrate a powerful microcontroller and a LoRa transceiver on a single chip[cite: 700, 714].
- [cite_start]**Efficient Firmware:** Employs **DMA in circular mode** for non-blocking UART data reception from the radiosonde, freeing up the CPU for other tasks[cite: 701, 707, 741].
- [cite_start]**Reliable Data Parsing:** A **Finite State Machine (FSM)** validates incoming data packets using a `SYNC_WORD` and `checksum` to ensure data integrity before retransmission[cite: 708, 744].
- [cite_start]**Low-Power, Event-Driven Receiver:** The base station is fully interrupt-driven, operating in a low-power sleep mode until a LoRa packet is received, making it highly efficient[cite: 804, 805].
- [cite_start]**Layered Software Architecture:** The firmware is built on STMicroelectronics' **HAL (Hardware Abstraction Layer)** and **BSP (Board Support Package)**, ensuring clean, portable, and maintainable code[cite: 701, 724].

## Project Structure

The repository is organized into a clean, modular structure:

```
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
```

## System Architecture

[cite_start]The system consists of two main nodes that work in tandem[cite: 705]:

1.  **Node 1: Field Node (Transmitter)**
    * [cite_start]Connects directly to a radiosonde via `USART1` (19200 baud) to capture a continuous stream of telemetry data[cite: 707].
    * [cite_start]Parses the data stream using an FSM to identify and validate complete data packets[cite: 708].
    * [cite_start]Valid packets, verified by a checksum, are transmitted wirelessly using the onboard LoRa radio[cite: 708, 709].

2.  **Node 2: Base Station (Receiver)**
    * [cite_start]Acts as a LoRa gateway, remaining in a continuous reception mode[cite: 710].
    * [cite_start]Upon receiving a valid LoRa packet, it decodes the binary payload into human-readable data[cite: 711].
    * [cite_start]Sends the formatted data to a connected PC via a Virtual COM Port (`USART2` at 115200 baud) for real-time monitoring[cite: 711].

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
    * 2x STM32 NUCLEO-WL55JC1 development boards
    * 2x 915 MHz Antennas
    * A radiosonde or other serial data source (19200 baud)

2.  **Setup:**
    * Clone this repository.
    * Open **STM32CubeIDE** and import the two projects located in `src/Field_Node/` and `src/Base_Station/`.
    * Compile both projects.
    * Flash the `Field_Node` firmware onto the first board and `Base_Station` onto the second.

3.  **Execution:**
    * Connect the Field Node to the radiosonde.
    * Connect the Base Station to a PC and open a serial terminal (like PuTTY) on the corresponding COM port at 115200 baud to view the received telemetry data.

## Validation & Results

[cite_start]The system was fully tested, demonstrating a successful end-to-end communication link[cite: 856]. The following are sample outputs captured from the serial terminals of both nodes during a test run.

[cite_start]**Transmitter (Field Node) Terminal Output[cite: 859]:**
```
Checksum OK. Pacote da radiosonda validado.
Transmitindo pacote LoRa ID: 1
LoRa TX Done.
```

[cite_start]**Receiver (Base Station) Terminal Output[cite: 864, 866, 868, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879]:**
```
Pacote LoRa Recebido! RSSI: -50 dBm, SNR: 9

---[ PACOTE DE TELEMETRIA DECODIFICADO ]---
ID do Pacote: 1
Latitude:     -3.740123
Longitude:    -38.570456
Altitude:     50.12 m
Voltagem:     3300 mV
Temp. Radio:  25 C
Status GPS:   FIX OK (Satelites: 8)
```

## Future Work & Roadmap

This project serves as a solid foundation for a field-ready telemetry system. [cite_start]Recommended next steps include[cite: 886]:

- [ ] [cite_start]**Power Optimization:** Implement the final low-power sleep modes (`SUBGRF_SetSleep`) on the Field Node to maximize battery life[cite: 887].
- [ ] [cite_start]**Data Visualization:** Develop a GUI or logging script on the PC to better visualize, store, and analyze the incoming telemetry data[cite: 888].
- [ ] [cite_start]**Field Testing:** Conduct real-world, open-field range tests to validate the performance and maximum range of the LoRa link[cite: 889].
- [ ] **Bi-directional Communication:** Implement a command system to send configuration messages from the Base Station back to the Field Node.

## Author

- [cite_start]**Alisson** [cite: 683]
