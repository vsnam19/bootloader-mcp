This repo contain the source code for Basic Bootloader base on Freescale Board.

Program:

- Build bootloader program for FRDM-MKL46Z4.

- Receive SREC file from PC send through UART.

Features:

- Auto verify srec file format.
- Auto backup previous firmware before update new firmware.
- Auto restore previous firmware when can not update new firmware.
- When a error happen, can upload firmware to continue update without reset cpu.
- Auto detect losing connection.

Properties:

- Max application programn size 16KB.
- Application region start: 0xA000, end 0xDFFF.
- Backup region start: 0xE000, end 0x1FFFF.
