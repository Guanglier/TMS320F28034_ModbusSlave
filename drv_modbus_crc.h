/*
 * drv_modbus_crc.h
 *
 *  Created on: May 1, 2025
 *      Author: c.monange
 */

//-----------------------------------------------------------------------
//
//
// Utilization
//
// uint16_t u16_crc = 0;
//
//
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
// 	u16_crc = crc16_update ( u16_crc, char_value);
//
//
//
//-----------------------------------------------------------------------

#ifndef SRC_DRV_MODBUS_CRC_H_
#define SRC_DRV_MODBUS_CRC_H_



	#include <stdint.h>

	#define DRV_MODBUS_CRC_INIT_VALUE	0xFFFF


	uint16_t 	drv_modbus_crc_crc16_update		(uint16_t li_u16_CRC_val, uint16_t li_u16_byteVal );
	uint16_t 	drv_modbus_crc_CRC16 			( uint16_t *liu16_PtrData, uint16_t liu16_byteCnt );
	uint16_t 	drv_modbus_crc_valid 			();


#endif /* SRC_DRV_MODBUS_CRC_H_ */

