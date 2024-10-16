/*
 * modbus.c
 *
 *  Created on: Oct 15, 2024
 *      Author: Victor Kalenda
 */

#include "modbus.h"
#include "error_codes.h"
//#include "usart.h"
#include "main.h"
#include <stdint.h>


#define TX_BUFFER_SIZE  RX_BUFFER_SIZE
#define MODBUS_TX_BUFFER_SIZE 255
#define MODBUS_RX_BUFFER_SIZE  255

uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];
uint8_t modbus_tx_buffer[MODBUS_TX_BUFFER_SIZE];
uint16_t tx_buffer[TX_BUFFER_SIZE];
uint16_t rx_buffer[RX_BUFFER_SIZE];

uint32_t response_interval = 1000;
uint32_t time = 0;
volatile uint8_t rx_int = 0;
volatile uint8_t tx_int = 0;

extern UART_HandleTypeDef huart1;

#define high_byte(value) ((value >> 8) & 0xFF)
#define low_byte(value) (value & 0xFF)
#define word(value1, value2) (((value1 >> 8) & 0xFF) | (value2 & 0xFF))


/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t crc_16(uint8_t *data, uint8_t size)
{
	uint8_t crc_hi = 0xFF;
	uint8_t crc_low = 0xFF;
	 unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (size--)
	{
		i = crc_low ^ *data++; /* calculate the CRC  */
		crc_low = crc_hi ^ table_crc_hi[i];
		crc_hi = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_low);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_int = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	tx_int = 1;
}

/**
Retrieve data from response buffer.

@see ModbusMaster::clearResponseBuffer()
@param u8Index index of response buffer array (0x00..0x3F)
@return value in position u8Index of response buffer (0x0000..0xFFFF)
*/
uint16_t get_response_buffer(uint8_t u8Index)
{
	if (u8Index < RX_BUFFER_SIZE)
	{
		// get the value in the uart recieve buffer
		return rx_buffer[u8Index];
	}
	else
	{
		return 0xFFFF;
	}
}

/**
Modbus function 0x03 Read Holding Registers.

This function code is used to read the contents of a contiguous block of
holding registers in a remote device. The request specifies the starting
register address and the number of registers. Registers are addressed
starting at zero.

The register data in the response buffer is packed as one word per
register.

@param read_address address of the first holding register (0x0000..0xFFFF)
@param read_quantity quantity of holding registers to read (1..125, enforced by remote device)
@param id the modbus id of the sensor or component to communicate to

@return 0 on success; exception number on failure
*/
int8_t read_holding_registers(uint16_t read_address, uint16_t read_quantity, uint8_t id)
{
	if(read_quantity > RX_BUFFER_SIZE)
	{
		return MB_MEMORY_ERROR;
	}
	uint8_t index = 0;
	// Append Modbus ID
	modbus_tx_buffer[index++] = id;
	// Append Function
	modbus_tx_buffer[index++] = 0x03;
	// Append the Read Address (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(read_address);
	modbus_tx_buffer[index++] = low_byte(read_address);
	// Append the quantity of registers to be read (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(read_quantity);
	modbus_tx_buffer[index++] = low_byte(read_quantity);
	// Append CRC (low byte then high byte)
	uint16_t crc = crc_16(modbus_tx_buffer, index);
	modbus_tx_buffer[index++] = low_byte(crc);
	modbus_tx_buffer[index++] = high_byte(crc);

	int8_t status = modbus_send(modbus_tx_buffer, index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for a response
	uint16_t rx_len = 0;
	status = modbus_poll_for_response(3 + read_quantity * 2 + 2, &rx_len);
	if(status != HAL_OK)
	{
		return status;
	}

	status = modbus_mic(id, 0x03, rx_len);

	store_rx_buffer();

	return status;
}


/**
Place data in transmit buffer.

@param index index of transmit buffer array (0x00..0x3F)
@param value value to place in position u8Index of transmit buffer (0x0000..0xFFFF)
@return 0 on success; exception number on failure
*/
int8_t set_tx_buffer(uint8_t index, uint16_t value)
{
	if (index < TX_BUFFER_SIZE)
	{
		tx_buffer[index] = value;
		return MB_SUCCESS;
	}
	else
	{
		return RANGE_ERROR;
	}
}

/**
Modbus function 0x10 Write Multiple Registers.

This function code is used to write a block of contiguous registers (1
to 123 registers) in a remote device.

The requested written values are specified in the transmit buffer. Data
is packed as one word per register.

@param write_address address of the holding register (0x0000..0xFFFF)
@param write_quantity quantity of holding registers to write (1..123, enforced by remote device)
@param id the modbus id of the sensor or component to communicate to

@return 0 on success; exception number on failure
*/
int8_t write_multiple_registers(uint16_t write_address, uint16_t write_quantity, uint8_t id)
{
	uint8_t index = 0;
	// Append Modbus ID
	modbus_tx_buffer[index++] = id;
	// Append Function
	modbus_tx_buffer[index++] = 0x10;
	// Append the Write Address (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_address);
	modbus_tx_buffer[index++] = low_byte(write_address);
	// Append the quantity of registers to be read (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_quantity);
	modbus_tx_buffer[index++] = low_byte(write_quantity);
	modbus_tx_buffer[index++] = low_byte(write_quantity << 1);
	// Append the tx_buffer (high byte then low byte
	for(uint8_t i = 0; i < low_byte(write_quantity); i++)
	{
		modbus_tx_buffer[index++] = high_byte(tx_buffer[i]);
		modbus_tx_buffer[index++] = low_byte(tx_buffer[i]);
	}

	// Append CRC (low byte then high byte)
	uint16_t crc = crc_16(modbus_tx_buffer, index);
	modbus_tx_buffer[index++] = low_byte(crc);
	modbus_tx_buffer[index++] = high_byte(crc);

	int8_t status = modbus_send(modbus_tx_buffer, index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for a response
	uint16_t rx_len = 0;
	status = modbus_poll_for_response(8, &rx_len);
	if(status != HAL_OK)
	{
		return status;
	}
	return modbus_mic(id, 0x10, rx_len);
}

/*
 * Modbus send function definition using stm32 uart
 *
 * Note: Response interval is split between the send and recieve function
 */
int8_t modbus_send(uint8_t *data, uint8_t size)
{
	int8_t status = HAL_OK;
	status = HAL_UART_Transmit_IT(&huart1, data, size);
	if(status != HAL_OK)
	{
		return status;
	}
	time = HAL_GetTick();
	while(!tx_int && (HAL_GetTick()) - time < 100);
	if(tx_int)
	{
		tx_int = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_TIMEOUT;
	}
}

int8_t modbus_poll_for_response(uint8_t size, uint16_t *rx_len)
{
	int8_t status = HAL_OK;
	status = HAL_UART_Receive_IT(&huart1, modbus_rx_buffer, size);
	if(status != HAL_OK)
	{
		return status;
	}
	while(!rx_int && (HAL_GetTick()) - time < response_interval);
	if(rx_int)
	{
		rx_int = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_TIMEOUT;
	}
}

int8_t modbus_setup_rx(uint8_t size)
{
	return HAL_UART_Receive_IT(&huart1, modbus_rx_buffer, size);
}

uint8_t modbus_rx()
{
	if(rx_int)
	{
		rx_int = 0;
		return 1;
	}
	return rx_int;
}

void set_response_interval(uint32_t delay)
{
	response_interval = delay;
}

uint32_t get_response_interval()
{
	return response_interval;
}

/*
 * Modbus message integrity check
 */
int8_t modbus_mic(uint8_t id, uint8_t function_code, uint8_t size)
{
	// Check the slave ID
	if(modbus_rx_buffer[0] != id)
	{
		return MB_SLAVE_ID_MISMATCH;
	}
	// Check the function code
	if((modbus_rx_buffer[1] & 0x7F) != function_code)
	{
		return MB_FUNCTION_MISMATCH;
	}

	// Check the modbus exception codes within the response if there is some sort of execution error
	if(((modbus_rx_buffer[1] >> 7) & 0x01))
	{
		return modbus_rx_buffer[2] + 0x03;
	}

	// Check the CRC
	if(size >= 5)
	{
		uint16_t crc = crc_16(modbus_rx_buffer, size - 2);
		if((low_byte(crc) != modbus_rx_buffer[size - 2]) || (high_byte(crc) != modbus_rx_buffer[size - 1]))
		{
			return MB_INVALID_CRC;
		}
	}
	return MB_SUCCESS;
}

void store_rx_buffer()
{
	// Store the message in the rx_buffer (this only applies to reading the holding registers)
	for(uint8_t i = 0; i < (modbus_rx_buffer[2] >> 1); i++)
	{
		if(i < RX_BUFFER_SIZE)
		{
			rx_buffer[i] = (modbus_rx_buffer[2 * i + 3] << 8) | modbus_rx_buffer[2 * i + 4];
		}
		// rx_buffer_len = i;
	}
}

int8_t modbus_change_baud_rate(uint8_t* baud_rate)
{
	int8_t status = 0;

	switch((*baud_rate))
	{
		case BAUD_RATE_4800:
		{
			huart1.Init.BaudRate = 4800;
			break;
		}
		case BAUD_RATE_9600:
		{
			huart1.Init.BaudRate = 9600;
			break;
		}
		case BAUD_RATE_19200:
		{
			huart1.Init.BaudRate = 19200;
			break;
		}
		case BAUD_RATE_38400:
		{
			huart1.Init.BaudRate = 38400;
			break;
		}
		case BAUD_RATE_57600:
		{
			huart1.Init.BaudRate = 57600;
			break;
		}
		case BAUD_RATE_115200:
		{
			huart1.Init.BaudRate = 115200;
			break;
		}
		case BAUD_RATE_128000:
		{
			huart1.Init.BaudRate = 128000;
			break;
		}
		case BAUD_RATE_256000:
		{
			huart1.Init.BaudRate = 256000;
			break;
		}
		default:
		{
			(*baud_rate) = BAUD_RATE_9600;
			huart1.Init.BaudRate = 9600;
			UART_SetConfig(&huart1);
			return MB_ILLEGAL_DATA_VALUE;
			break;
		}

	}
	status = UART_SetConfig(&huart1);

	if(status != HAL_OK)
	{
		return status;
	}

	//status = HAL_UART_Receive_IT(huart, pData, Size)

	return status;
}

int8_t modbus_set_baud_rate(uint8_t baud_rate)
{
	int8_t status = HAL_OK;
	/* Designed to hold baud rate in emulated EEPROM
	if(ee.modbus_baud_rate != baud_rate)
	{
		ee.modbus_baud_rate = baud_rate;

		if(!EE_Write())
		{
			osMutexRelease(eeprom_mutexHandle);
			return EE_WRITE_ERROR;
		}
	}
	*/
	return status;
}

int8_t modbus_get_baud_rate(uint8_t* baud_rate)
{
	int8_t status = HAL_OK;

	/* Designed to hold baud rate in emulated EEPROM
	*baud_rate = ee.modbus_baud_rate;
	*/

	return status;
}

uint8_t significant_error(int8_t status)
{
  switch(status)
  {
    case MB_ILLEGAL_FUNCTION ... MB_SLAVE_ERROR:
    {
      return 1;
    }
    case MB_ACK:
    {
      return 0;
    }
    case MB_SLAVE_BUSY ... MB_NEGATIVE_ACK:
    {
      return 1;
    }
    case MB_MEMORY_ERROR:
    {
      return 0;
    }
    case MB_GATEWAY_PATH_ERROR ... MB_FUNCTION_MISMATCH:
    {
      return 1;
    }
    case MB_INVALID_CRC:
    {
      return 0;
    }
    default:
    {
      return 0;
    }
  }
}

// Debug Functions
/*
uint8_t get_modbus_tx(uint8_t index)
{
	return modbus_tx_buffer[index];
}
*/

