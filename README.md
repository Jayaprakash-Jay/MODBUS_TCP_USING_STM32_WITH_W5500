# MODBUS_TCP_USING_STM32_WITH_W5500
This project implements a Modbus TCP Server (Slave) on an STM32 microcontroller using the W5500 Ethernet controller. Communication is based on the Modbus protocol over TCP/IP and supports basic function codes such as:

0x03 – Read Holding Registers
0x04 – Read Input Registers
0x06 – Write Single Register
0x10 – Write Multiple Registers

🚀 Features
Modbus TCP Server on port 502
Registers mapped in RAM

Support for:

Read Holding Registers (0x03)
Read Input Registers (0x04)
Write Single Register (0x06)
Basic Modbus Exception Handling
Socket management using W5500 (via WIZnet library)

