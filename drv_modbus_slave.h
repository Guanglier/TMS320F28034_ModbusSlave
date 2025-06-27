





//=========================================================================================================================
//
//  https://ipc2u.com/articles/knowledge-base/modbus-rtu-made-simple-with-detailed-descriptions-and-examples/
//
//	https://modbus.org/docs/PI_MBUS_300.pdf

//		testing tool : 	master simulator https://qmodbus.sourceforge.net/
//						slave simulator : https://sourceforge.net/projects/modrssim2/
//
//  0x01 : Read digital output	(coil status)
//  0x02 : Read digital input (input status)
//  0x03 : Read analog output (holding register)
//  0x04 : Read analog input (input register)
//	0x05 : write discrete output (coil)
//	0x06 : write single holding register
//	0x0F : write multiple coils
//	0x10 : write multiple hodling registers
//
//
//=========================================================================================================================
#ifndef SRC_DRV_MODBUS_SLAVE_H_
#define SRC_DRV_MODBUS_SLAVE_H_


#include "stdint.h"



//-- following is the length of RX and TX buffer
// increase if you need to send more data on single frame
#define MOSBUS_SLAVE_REQFRAME_DATA_MAX_LENGTH		20
#define MOSBUS_SLVAE_ANSWER_DATA_AND_CRC_SIZE		20
#define MOSBUS_SLVAE_TRAMETXBUFF_SIZE_U16_CNT		20

//----------- this is the define to be used to get or set reg value --------------------
typedef enum{
	MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL,
	MODBUS_SLAVE_REG_TYPE_INPUTCOIL,
	MODBUS_SLAVE_REG_TYPE_INPUTREG,
	MODBUS_SLAVE_REG_TYPE_OUTPUTREG,
	MODBUS_SLAVE_REG_TYPE_ERROR
}MODBUS_SLAVE_REG_TYPE_t;


//----------- definiton of one register -----------------
typedef struct MODBUS_SLAVE_UPDATED_REG_VALUE_s {
	uint16_t					u16_reg_adr;
	uint16_t					u16_reg_new_val;
	MODBUS_SLAVE_REG_TYPE_t		reg_type;
}MODBUS_SLAVE_UPDATED_REG_VALUE_t;




//---------------------------------------------------------------------------
//	definnition of the reception of the frame
//---------------------------------------------------------------------------
typedef struct  MOSBUS_SLVAE_REQFRAME_s{
	struct{
		uint16_t		u16_slave_adr;
		uint16_t		u16_function_code;
		uint16_t 	u16_addr;
		uint16_t 	u16_crc;
	}Common;
	union{
		struct{
			uint16_t 	u16_reg_cnt;
		}ReadReq;
		struct{
			uint16_t 	u16_val;
		}WriteSingleReq;
		struct{
			uint16_t 	u16_NumberReg;
			uint16_t  	u16_NbBytes;
			uint16_t 	u16_TabValues[MOSBUS_SLAVE_REQFRAME_DATA_MAX_LENGTH];
		}WriteMultipleReq;
	}ReqType;
}MOSBUS_SLVAE_REQFRAME_t;
//---------------------------------------------------------------------------
//	definition of the answer frame
//---------------------------------------------------------------------------
typedef union  MOSBUS_SLVAE_ANSWER_FRAME_s{
	struct{
		uint16_t	u16_slave_adr;
		uint16_t	u16_function_code;
		uint16_t 	u16_data_byte_cnt;
		uint16_t	u16_data_tab[MOSBUS_SLVAE_ANSWER_DATA_AND_CRC_SIZE];
	}aswr_read;
	union{
		struct{
			uint16_t	u16_slave_adr;
			uint16_t	u16_function_code;
			uint16_t 	u16_reg_addr_MSB;
			uint16_t 	u16_reg_addr_LSB;
			uint16_t 	u16_reg_value_MSB;
			uint16_t 	u16_reg_value_LSB;
			uint16_t	u16_crc;
		}aswr_single_wr;
		struct{
			uint16_t	u16_slave_adr;
			uint16_t	u16_function_code;
			uint16_t 	u16_reg_addr_MSB;
			uint16_t 	u16_reg_addr_LSB;
			uint16_t 	u16_nb_of_reg_written_MSB;
			uint16_t 	u16_nb_of_reg_written_LSB;
			uint16_t	u16_crc;
		}aswr_multiples_wr;
	}writes;
}MOSBUS_SLVAE_ANSWER_FRAME_s;
typedef struct  MOSBUS_SLVAE_STATE_s{
	uint16_t	u16_etat;
	uint16_t	u16_rx_buff_index;

	// is 0 when disabled, put to a value (TIMEOUT_CPT_VALUE) and then decremented by the timer.
	// When reaching 0 by timer then timeout error. necessary to do so to share one timer for all channels
	uint16_t	u16_TimeOutErrCpt;
}MOSBUS_SLVAE_STATE_t;





//---------------------------------------------------------------------------
//	definnition of the modbus register table
//---------------------------------------------------------------------------
typedef struct DRV_MODBUS_SLAVE_REGDEF_s {
	const uint16_t				u16_adr;
	uint16_t					u16_value;
	uint16_t						u16_updated;
}DRV_MODBUS_SLAVE_REGDEF_t;

typedef struct DRV_MODBUS_SLAVE_REGBASE_s {
	struct{
		DRV_MODBUS_SLAVE_REGDEF_t	* const Ptr;
		uint16_t					NbElmts;
	}OutputReg;
	struct{
		DRV_MODBUS_SLAVE_REGDEF_t	* const Ptr;
		uint16_t					NbElmts;
	}OutputCoils;
	struct{
		DRV_MODBUS_SLAVE_REGDEF_t	* const Ptr;
		uint16_t					NbElmts;
	}InputReg;
	struct{
		DRV_MODBUS_SLAVE_REGDEF_t	* const Ptr;
		uint16_t					NbElmts;
	}InputCoils;
}DRV_MODBUS_SLAVE_REGBASE_t;



//---------------------------------------------------------------------------
//	instance of modbus, this is the handler, the structure that holds everything
//	on the modbus. two or more sturctures can be made to create multiple modbuses
//---------------------------------------------------------------------------
typedef struct  MOSBUS_SLVAE_INSTANCE_s{
	struct{
		DRV_MODBUS_SLAVE_REGBASE_t	* RegistersStrPtr;
		uint16_t 					u16_ModbusAddress;
		uint16_t						u16_uart_id;
	}config;
	struct{
		uint16_t					u16_crc;
		MOSBUS_SLVAE_STATE_t		State;
		MOSBUS_SLVAE_REQFRAME_t		TrameRx;
		//MOSBUS_SLVAE_ANSWER_FRAME_s	TrameTX;
		uint16_t					TrameTXTab[MOSBUS_SLVAE_TRAMETXBUFF_SIZE_U16_CNT];
	}internal;
}MOSBUS_SLVAE_INSTANCE_t;




//-------------------------------------------------------------------------------------------------
//	to be call regularly by the app to see if one reg has been updated
//	if return is non 0, then it's the function code of the received write
//		on the structure the corresponding register address and value are returned
//		the returned register is "cleared" ( marked as non updated by the modbus layer)
//	this function must then be called until the return is 0 (all updated regs has been seen)
//-------------------------------------------------------------------------------------------------
uint16_t drv_modbus_slave_GetUpdatedRegValue (MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance, MODBUS_SLAVE_UPDATED_REG_VALUE_t *liPtrUpdatedRegValue);


//-------------------------------------------------------------------------------------------------
//	change the value of one register, return 0 if succeed
//-------------------------------------------------------------------------------------------------
uint16_t drv_modbus_slave_SetNewRegValue ( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance, const MODBUS_SLAVE_REG_TYPE_t liu16_RegType, const uint16_t u16_reg_adr, const uint16_t u16_reg_value );

//-------------------------------------------------------------------------------------------------
//	get the value of one register, return 0 if succeed
//-------------------------------------------------------------------------------------------------
uint16_t drv_modbus_get_reg_value (MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance,  const MODBUS_SLAVE_REG_TYPE_t liu16_RegType, const uint16_t u16_reg_adr, uint16_t* u16_ptr_data);



void drv_modbus_slave_init						( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance );
void drv_modbus_slave_TimeoutTimer_callback 	( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance );
void drv_modbus_slave_rxtUartHandler 			( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance, uint16_t li_u16_char );
void drv_modbus_handler_tx_ended 				( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance);





#define MODBUSSLAVE_POS_ADDR					0	//position in byte of the address byte
#define MODBUSSLAVE_POS_FCTCODE					1	//
#define MODBUSSLAVE_ERRFRAME_POS_ERRCODE		2	//
#define MODBUSSLAVE_BYTE_CNT					2	//
#define MODBUSSLAVE_POS_DATA					3	//

#define MODBUSSLAVE_ASWWR05_POS_REGADRMSB		2	//
#define MODBUSSLAVE_ASWWR15_POS_REGVALMSB		4	//
#define MODBUSSLAVE_ASWWR15_POS_REGVALLSB		5	//
#define MODBUSSLAVE_ASWWR15_POS_NBREGSMSB		4	//



#endif /* SRC_DRV_MODBUS_SLAVE_H_ */




