/*
 * modbus_tcp.c
 *
 *  Created on: May 13, 2025
 *      Author: jayap
 */

#include "main.h"
#include "wizchip_conf.h"
#include "modbus_tcp.h"
#include "socket.h"
#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <string.h>

#include "ina260.h"


/*
MODBUS TCP FRAME

Example (Modbus TCP Read Holding Registers Request)

Suppose a client wants to read 2 registers starting from address 0x000A from a Modbus TCP slave.

Field	           Value	Description
---------------------------------------------------------------------------------------------------
Transaction ID	   0x0001	Client-defined
Protocol ID	       0x0000	Always 0 for TCP
Length	           0x0006	Unit ID (1) + Function Code (1) + Address (2) + Quantity (2) = 6 bytes
Unit ID	           0x01		Slave address
Function Code	   0x03		Read Holding Registers
Start Address	   0x000A	Register start address
Quantity	       0x0002	Number of registers to read

Raw frame (hex):

00 01   00 00    00 06      01        03        00 0A     00 02
=====   =====    =====    ======    ======     =======   =======
T ID    P ID     Length    U ID     FnCode      St Add    Quantity


Modbus TCP Response Frame

If the slave responds with two 16-bit values, say 0x1234 and 0x5678, the response would look like:

Field	            Value
----------------------------------
Transaction ID  	0x0001
Protocol ID	        0x0000
Length	            0x0007
Unit ID	            0x01
Function Code	    0x03
Byte Count	        0x04
Data	            0x12 0x34 0x56 0x78

Raw frame (hex):

00 01   00 00    00 07      01        03         04      12 34 56 78
=====   =====    =====    ======    ======     =======   ===========
T ID    P ID     Length    U ID     FnCode      Byte C      DATA


Function codes
========================
readHoldingRegs    == 3
writemultipleRegs  == 16
readInputRegs      == 4
writeSingleReg     == 6


uint16_t Holding_Registers_Database[50]={
		0,    0,    0,    0,    0,    0,    0,    0,                // voltage buff
		500,  500,  500,  500,  500,  500,  500,  500,              // vmax
		5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000,             // vmin
		125,  125,  125,  125,  125,  125,  125,  125,              // spare
		30000,30000,30000,30000,30000,30000,30000,30000,            // spare
		0,    0,    0,    0,    0,    0,    0,    0,                // spare
		0,    0,                                                    // spare
};

uint16_t Input_Registers_Database[50]={
		1234,  4321,  1122,  2211,  1112,  2111,  6666,  7777,  8888,  9999,   // 0-9   30001-30010 voltage
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 30011-30020 current
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 30021-30030 temparature
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 30031-30040 spare
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 30041-30050 spare
};


#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION    0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS     0x02
#define MODBUS_EXCEPTION_ILLEGAL_VALUE       0x03
#define MODBUS_EXCEPTION_DEVICE_FAILURE      0x04


 */
uint16_t Holding_Registers_Database[6]={140,150,160,170,180,190};
uint16_t Input_Registers_Database[12]={0};


extern INA260_t sensor;


uint8_t buffer[260];





void modbusException(uint8_t exceptionCode, uint8_t functionCode) {
	uint8_t response[9];

	response[0] = buffer[0];
	response[1] = buffer[1];
	response[2] = 0x00;  // Protocol ID Hi
	response[3] = 0x00;  // Protocol ID Lo
	response[4] = 0x00;  // Length Hi
	response[5] = 0x03;  // Length Lo (Unit ID + Func + Code)
	response[6] = buffer[6];  // Unit Identifier
	response[7] = functionCode | 0x80;  // Exception response
	response[8] = exceptionCode;

	send(0,response, 9);  // Send via raw socket
}



void loop_modbus_slave() {

	uint8_t sock_num = 0;

	// Open socket once
	if (socket(sock_num, Sn_MR_TCP, 502, 0) != sock_num) {
		printf("Failed to open socket\n\r");
		return;
	}

	if (listen(sock_num) != SOCK_OK) {
		printf("Listen failed\n\r");
		close(sock_num);
		return;
	}

	printf("Modbus TCP slave ready\n\r");

	while (1) {
		uint8_t sock_status = getSn_SR(sock_num);

		if (sock_status == SOCK_ESTABLISHED) {
			//printf("Client connected\n\r");

			uint16_t rx_size = getSn_RX_RSR(sock_num);
			if (rx_size > 0) {
				int len = recv(sock_num, buffer, sizeof(buffer));
				//printf("Received len = %d\n\r", len);

				if (len > 0) {
					parse_modbus_request(buffer, len);
				}
			}
		}
		else if (sock_status == SOCK_CLOSE_WAIT) {
			printf("Closing socket\n\r");
			disconnect(sock_num);
		}
		else if (sock_status == SOCK_CLOSED) {
			printf("Socket closed, reopening\n\r");
			socket(sock_num, Sn_MR_TCP, 502, 0);
			listen(sock_num);
		}
		//read voltage and current
	//	readCV();
		//set voltage
	/*	for(uint8_t i=0;i<6;i++)
		{
			ps_set_volt(Holding_Registers_Database[i],i);
		}
*/
		HAL_Delay(10); // Small delay to avoid CPU hogging

	}
}

uint8_t response[260];

void parse_modbus_request(uint8_t *request, uint16_t len) {

	uint8_t function_code   = request[7];
	printf("\rReq :- ");
	for(int i=0;i<12;i++)
	{
		printf("%x ", request[i]);
	}
	printf("\n\r");
	// Read Holding Registers
	if (function_code == 3) {
		readHoldingRegs(request, len);
	}
	// Write Multiple Registers
	else if (function_code == 16) {
		writemultipleRegs(request,  len);
	}
	else if (function_code == 4) {
		readInputRegs(request, len);
	}
	// Write Single Register
	else if (function_code == 6) {
		writeSingleReg(request,  len);
	}
	else
	{
		modbusException(ILLEGAL_FUNCTION,request[7]);
		printf("modbusException from parse\n\r");
	}
}

void readHoldingRegs(uint8_t *request, uint16_t len)
{
	uint8_t unit_id         = request[6];
	uint16_t start_address = (request[8] << 8) | request[9];
	uint16_t quantity = (request[10] << 8) | request[11];

	response[0] = request[0];
	response[1] = request[1]; // Transaction ID
	response[2] = 0;//pid
	response[3] = 0;//pid
	response[4] = 0;//len
	response[5] = 3 + 2 * quantity;//len
	response[6] = unit_id;
	response[7] = 3; //fun code
	response[8] = 2 * quantity;

	uint16_t endAddr = start_address+quantity;  // end Register

	if (endAddr>6)  //write the number of bytes you want to read size of the array
	{
		for(int i = start_address; i < 6; i++)
		{
			for(int j = quantity; j > 0; j--)
			{
				if(i+j>6)
				{
					modbusException (ILLEGAL_DATA_VALUE,request[7]);  // send an exception
					printf("modbusException from readHoldingRegs illegal data value\n\r");
					return ;
				}
			}
		}
		modbusException(ILLEGAL_DATA_ADDRESS,request[7]);   // send an exception
		printf("modbusException from readHoldingRegs illegal data address\n\r");
		return ;
	}

	for (int i = 0; i < quantity; i++) {
		uint16_t val = 0;

		val = Holding_Registers_Database[start_address + i];

		response[9 + i * 2] = val >> 8;
		response[10 + i * 2] = val & 0xFF;
	}

	send(0, response, 9 + 2 * quantity);
}
void readInputRegs(uint8_t *request, uint16_t len)//fun code 4
{
	//uint8_t response[260];
	uint8_t unit_id = request[6];
	uint16_t start_address = (request[8] << 8) | request[9];
	uint16_t quantity = (request[10] << 8) | request[11];

	uint16_t endAddr = start_address+quantity;  // end Register

	if (endAddr>12)  //write the number of bytes you want to read size of the array
	{
		for(int i = start_address; i < 12; i++)
		{
			for(int j = quantity; j > 0; j--)
			{
				if(i+j>12)
				{
					modbusException (ILLEGAL_DATA_VALUE,request[7]);  // send an exception
					printf("modbusException from readHoldingRegs illegal data value\n\r");
					return ;
				}
			}
		}
		modbusException(ILLEGAL_DATA_ADDRESS,request[7]);   // send an exception
		printf("modbusException from readHoldingRegs illegal data address\n\r");
		return ;
	}

	// Build MBAP header (copy first 6 bytes from request)
	for (int i = 0; i < 6; i++) response[i] = request[i];

	response[6] = unit_id;
	response[7] = 4; // Function code
	response[8] = quantity * 2; // Byte count

	for (int i = 0; i < quantity; i++) {
		uint16_t value = 0;
		value = Input_Registers_Database[start_address + i]; // 0-7 mapped to voltages[]
		response[9 + i * 2] = (value >> 8) & 0xFF;
		response[10 + i * 2] = value & 0xFF;
	}
	printf("Tx :- ");
	for(uint8_t i=0;i<9 + quantity * 2;i++)
	{
		printf("%d ",response[i]);
	}
	printf("\n\r");
	send(0, response, 9 + quantity * 2);
}

void writeSingleReg(uint8_t *request, uint16_t len)//fun code 6
{
	uint16_t register_address = (request[8] << 8) | request[9];
	uint16_t value = (request[10] << 8) | request[11];

	// Example: we store values from register 16 to 23 into v_set[0] to v_set[7]
	/*if (register_address >= 16 && register_address < 24) {
		Holding_Registers_Database[register_address - 16] = value;
		printf("Holding_Registers_Database[%d] = %d\n", register_address - 16, value);
	}*/

	if(register_address > 7 && register_address < 0)
	{
		modbusException(ILLEGAL_DATA_ADDRESS,request[7]);
		printf("modbusException from writeSingleReg\n\r");
		return;
	}
	Holding_Registers_Database[register_address] = value;
	printf("\rHolding_Registers_Database[%d] = %d\n",register_address, value);

	// Echo the request back as a response
	for (int i = 0; i < 12; i++) {
		response[i] = request[i];
	}

	send(0, response, 12);
}
void writemultipleRegs(uint8_t *request, uint16_t len)//fun code 16
{
	/*uint8_t unit_id = request[6];
	uint16_t start_address = (request[8] << 8) | request[9];
	uint16_t quantity = (request[10] << 8) | request[11];
	//uint8_t byte_count = request[12];

	uint8_t endAddr,numRegs = byte_count;
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE,request[7]);  // send an exception
		printf("modbusException from writemultipleRegs\n\r");
		return 0;
	}

	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS,request[7]);   // send an exception
		printf("modbusException from writemultipleRegs\n\r");
		return 0;
	}


	for (int i = 0; i < quantity; i++) {
		uint16_t value = (request[13 + i * 2] << 8) | request[14 + i * 2];

		if (start_address + i >= 16 && start_address + i < 24) {
			Holding_Registers_Database[start_address + i - 16] = value;
		}
	}
	// Acknowledge write
	for (int i = 0; i < 6; i++) response[i] = request[i]; // Copy MBAP
	response[6] = unit_id;
	response[7] = 16;
	response[8] = request[8];
	response[9] = request[9];
	response[10] = request[10];
	response[11] = request[11];

	printf("Tx :- ");
	for(uint8_t i=0;i<12;i++)
	{
		printf("%d ",response[i]);
	}
	printf("\n\r");

	send(0, response, 12);*/
}

