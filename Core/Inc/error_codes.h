/*
 * error_codes.h
 *
 *  Created on: Jun 28, 2024
 *      Author: Victor Kalenda
 */

#ifndef APPLICATION_USER_CORE_CUSTOM_LAYERS_ERROR_CODES_H_
#define APPLICATION_USER_CORE_CUSTOM_LAYERS_ERROR_CODES_H_

// -6 to -1 reserved for OS errors
// 0x00 reserved for status OK in all contexts
// 0x01 to 0x03 reserved for HAL errors

// Modbus Exception Codes
#define MB_ILLEGAL_FUNCTION 		0x04
#define MB_ILLEGAL_DATA_ADDRESS		0x05
#define MB_ILLEGAL_DATA_VALUE 		0x06
#define MB_SLAVE_ERROR 				0x07
#define MB_ACK 						0x08
#define MB_SLAVE_BUSY 				0x09
#define MB_NEGATIVE_ACK 			0x0A
#define MB_MEMORY_ERROR				0x0B
#define MB_GATEWAY_PATH_ERROR 		0x0C
#define MB_GATEWAY_RX_ERROR			0x0D

// High level errors as a result of poor programming on my behalf or the sensors behalf (but most likely my behalf)
#define RANGE_ERROR					0x0E
#define MB_SLAVE_ID_MISMATCH 		0x0F
#define MB_FUNCTION_MISMATCH		0x10
#define MB_INVALID_CRC 				0x11

#endif /* APPLICATION_USER_CORE_CUSTOM_LAYERS_ERROR_CODES_H_ */
