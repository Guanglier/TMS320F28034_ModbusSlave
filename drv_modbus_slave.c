




//=========================================================================================================
//
//		https://ozeki.hu/p_5873-modbus-function-codes.html
//		https://ozeki.hu/p_5846-appendix.html
//			https://ozeki.hu/p_5874-modbus-references.html
//
//		- TO DO : 	violation of atomicity of multiple write : if one fails, some register could have been modified
//					but modbus specifies it's atomic, if one fails, no registers should have been written.
//		- TO DO : 	Gérer le overrunn error qui bloque tout ..
//		- TO VALIDATE 		ignore frames for other slaves, wait for timeout of transmission before looking for address (semble être bon)
//		- DONE 		modifier le mécanisme de timeout pour n'utiliser qu'un timer pour tous les canaux -> ok
//		- DONE 		la valeur à mettre pour drv_modbus_get_reg_value n'est pas consistante
//		- DONE : 	n'envoyer la trame d'erreur qu'une fois à la fin et pas un peu partout dans le code
//		- DONE - 	Lors de la fin de la transmission, désactiver le driver de bus, donc gérer l'état transmission en cours
//		- DONE - 	Utiliser les IT pour le TX
//		- CANCEL - 	activer le DE (driver enable) qui est sorti sur le RTS (PA12)
//		- CANCEL -	 Utiliser le DMA pour le TX (non ici les IT)
//		- CANCEL 	merger les listes coil reg etc.. pour n'avaoir qu'une liste, taille définie à la compilation et contenue dans l'application
//					pointeur vers la liste mise dans la structure de handler
//		- DONE - 	add RX timeout mechanism
//		- DONE - 	renvoyer trame erreur si en write coil on envoie pas 0 ou FF00
//		- DONE - 	à gérer quand on dépasse en réception de write multiple MOSBUS_SLAVE_DATA_MAX_LENGTH ça semble planter
//=========================================================================================================


#include "drv_modbus_slave.h"
#include "drv_modbus_crc.h"
//#include "hal_uart.h"


#define ENABLE_VALIDATION	

#ifdef ENABLE_VALIDATION
	void app_modbus_valid_TxCharFunction (const uint16_t *liu16_ptr, const uint16_t liu16_size);
#endif


#define U8_MAX_VALUE							255
#define U16_COIL_SET_VALUE_TO_ONE	(uint16_t) 	0xFF00
#define LSB_MASK								0x00FF
#define BYTE_SHIFT								8

//--------- modbus supported commands ------------------
enum{
	MODBUS_FCT_CODE_RD_OUTPUTCOILS 			= 0x01,
	MODBUS_FCT_CODE_RD_INPUTCOIL 			= 0x02,
	MODBUS_FCT_CODE_RD_OUTPUTREG			= 0x03,
	MODBUS_FCT_CODE_RD_INPUTREGS			= 0x04,
	MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL	= 0x05,
	MODBUS_FCT_CODE_WR_SINGLE_OUTPUTREG		= 0x06,
	MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTCOIL 	= 0x0F,
	MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG 	= 0x10,
	MODBUS_FCT_CODE_ERROR					= 0x80
};

//--------- errors while processing req ------------------
typedef enum  {
	MODBUS_SLAVE_ERR_NO_ERROR 				= 0x00,
	MODBUS_SLAVE_ERR_CANNOT_PROCESS 		= 0x01,
	MODBUS_SLAVE_ERR_DATA_NOT_AVAILABLE 	= 0x02,
	MODBUS_SLAVE_ERR_INVALID_VALUE			= 0x03,
	MODBUS_SLAVE_ERR_ERRWHILEPROCESSINGREQ 	= 0x04
}MODBUS_SLAVE_ERR_t;


//--------- states of RX ------------------
enum{
	MODBUS_SLAVE_STATE_INIT,
	MODBUS_SLAVE_STATE_WAIT_RX_ADR,
	MODBUS_SLAVE_STATE_WAIT_RX_FCODE,
	MODBUS_SLAVE_STATE_WAIT_RX_ADRMSB,
	MODBUS_SLAVE_STATE_WAIT_RX_ADRLSB,
	MODBUS_SLAVE_STATE_WAIT_RX_LENGTHMSB,
	MODBUS_SLAVE_STATE_WAIT_RX_LENGTHLSB,
	MODBUS_SLAVE_STATE_WAIT_RX_NBOFBYTES,
	MODBUS_SLAVE_STATE_WAIT_RX_MULTIPLEDATA,
	MODBUS_SLAVE_STATE_WAIT_RX_CRCMSB,
	MODBUS_SLAVE_STATE_WAIT_RX_CRCLSB,
	MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME
};

//--------- definition of the PIN to enable / disable bus ------------------
/*
#define MB1_SET_PIN_DE_HIGH		GPIOA->BSRR = 0x00001000
#define MB1_SET_PIN_DE_LOW		GPIOA->BSRR = 0x10000000

#define MB1_ENABLE_TRANSMIT		MB1_SET_PIN_DE_LOW
#define MB1_DISABLE_TRANSMIT	MB1_SET_PIN_DE_HIGH



#define MB2_SET_PIN_DE_HIGH		GPIOA->BSRR = 0x00000002
#define MB2_SET_PIN_DE_LOW		GPIOA->BSRR = 0x00020000

#define MB2_ENABLE_TRANSMIT		MB2_SET_PIN_DE_LOW
#define MB2_DISABLE_TRANSMIT	MB2_SET_PIN_DE_HIGH
*/

#define MB1_DISABLE_TRANSMIT
#define MB2_DISABLE_TRANSMIT
#define MB1_ENABLE_TRANSMIT
#define MB2_ENABLE_TRANSMIT



//---------------------------------------------------------------------------------
//		9600 bauds/s   timeout of 3.5 characters -> we take 3 characters so 30 bits = 3.12 ms
//	this function needs to be on the ISR of the timer, timer with a timeout of 3.12 ms
//
//       void TIM17_IRQHandler(void)
//       {
//       	drv_modbus_slave_TimeoutTimer_callback ();
//       }
//
//
//---------------------------------------------------------------------------------
#define TIMEOUT_CPT_VALUE	3
void drv_modbus_slave_Timeout_EnableAndReset ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance ){
	liPtrModbusInstance->internal.State.u16_TimeOutErrCpt = TIMEOUT_CPT_VALUE;	//timeout disabled
}
void drv_modbus_slave_TimeoutTimer_Disable ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance ){
	liPtrModbusInstance->internal.State.u16_TimeOutErrCpt = 0;	//timeout disabled
}
void drv_modbus_slave_TimeoutTimer_callback ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance ){

	//-- if cpt value != 0 then the timeout must be executed
	if ( 0 != liPtrModbusInstance->internal.State.u16_TimeOutErrCpt )
	{
		liPtrModbusInstance->internal.State.u16_TimeOutErrCpt--;
		if (0 == liPtrModbusInstance->internal.State.u16_TimeOutErrCpt )	//timeout elapsed
		{
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADR;
		}
	}
}
void drv_modbus_slave_TimeoutTimer_Configure ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance ){
	drv_modbus_slave_Timeout_EnableAndReset ( liPtrModbusInstance );

/*
	//sustem clock 48 Mhz  prescaler = 100 autoreload=150  period = 3.125 ms
	TIM17->ARR = 1500;
	TIM17->PSC = 100;
	TIM17->CR1 = 0;
	TIM17->CR2 = 0x0000;

	TIM17->EGR = 0x0001; 	// force update of timer
	TIM17->DIER = 0x0001;	// update interrupt enabled

	TIM17->SR &= 0xFFFE;		// clear IT flag
	TIM17->CR1 = 0x0001;		// enable counting
    */
}



//---------------------------------------------------------------------------------
//		init of the modbus slave layer
//
//---------------------------------------------------------------------------------
void drv_modbus_slave_init ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance ){
	DRV_MODBUS_SLAVE_REGDEF_t	*l_PtrTab;
	uint16_t	lu16_TabSize;
    uint16_t    u16_index;

	drv_modbus_slave_TimeoutTimer_Disable ( liPtrModbusInstance );
	drv_modbus_slave_TimeoutTimer_Configure ( liPtrModbusInstance );

	liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADR;
	//liPtrModbusInstance->config.u16_ModbusAddress = 0x12;
	liPtrModbusInstance->internal.State.u16_TimeOutErrCpt = 0;
	liPtrModbusInstance->internal.TrameRx.Common.u16_slave_adr = 0;
	liPtrModbusInstance->internal.u16_crc = 0;


	l_PtrTab = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.Ptr;
	lu16_TabSize = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.NbElmts;
	for ( u16_index=0; u16_index<lu16_TabSize; u16_index++){
		l_PtrTab[u16_index].u16_updated = 0;
	}

	l_PtrTab = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.Ptr;
	lu16_TabSize = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.NbElmts;
	for ( u16_index=0; u16_index<lu16_TabSize; u16_index++){
		l_PtrTab[u16_index].u16_updated = 0;
	}

	switch ( liPtrModbusInstance->config.u16_uart_id ){
	case 1:
		MB1_DISABLE_TRANSMIT;
		break;
	case 2:
		MB2_DISABLE_TRANSMIT;
		break;
	}


}





//---------------------------------------------------------------------------------
//	takes a tab of all frame to send ,check some parameters,
//	compute crc and send the frame
//---------------------------------------------------------------------------------
static uint16_t drv_modbus_make_and_send_frame ( MOSBUS_SLVAE_INSTANCE_t *const liPtrModbusInstance, uint16_t *liu16_ptr, const uint16_t liu16_size_frame_and_crc, const uint16_t u16_buff_max_size_InBytes){
	uint16_t 	lu16_crc = 0xFFFF;
	uint16_t	u16_LengthDataToCrc = (uint16_t) liu16_size_frame_and_crc-2;

	if ( liu16_size_frame_and_crc < 2){
		return MODBUS_SLAVE_ERR_ERRWHILEPROCESSINGREQ;
	}
	if ( u16_LengthDataToCrc > (u16_buff_max_size_InBytes-2 ) ){
		return MODBUS_SLAVE_ERR_ERRWHILEPROCESSINGREQ;
	}
	lu16_crc = drv_modbus_crc_CRC16 ( liu16_ptr, u16_LengthDataToCrc);
	//lu16_ptr_Set += liu16_size_frame_and_crc - 2;		// skip address, function code, length

	//reverse CRC
	lu16_crc =  (lu16_crc<<8) | (lu16_crc>>8);;

	__byte( (int*) liu16_ptr,(liu16_size_frame_and_crc - 2)) = (uint16_t) (lu16_crc >> 8);
	__byte( (int*) liu16_ptr,(liu16_size_frame_and_crc - 2 + 1)) = (uint16_t) (LSB_MASK & lu16_crc) ;
	//*lu16_ptr_Set++ = (uint16_t) (lu16_crc >> 8);
	//*lu16_ptr_Set++ = (uint16_t) (LSB_MASK & lu16_crc) ;

	// envoi de la trame
	switch ( liPtrModbusInstance->config.u16_uart_id ){
		case 1:
			MB1_ENABLE_TRANSMIT;
			break;
		case 2:
			MB2_ENABLE_TRANSMIT;
			break;
	}
	//ENABLE_TRANSMIT;
	switch ( liPtrModbusInstance->config.u16_uart_id ){
		case 1:
			//hal_uart1_transmit_IT ((const uint16_t*) liu8_ptr, (const uint16_t) liu16_size_frame_and_crc);
			#ifndef ENABLE_VALIDATION
			hal_uart1_transmit_IT( liu16_ptr , liu16_size_frame_and_crc );
			#else
			app_modbus_valid_TxCharFunction( liu16_ptr , liu16_size_frame_and_crc );
			#endif
			break;
		case 2:
			//hal_uart2_transmit_IT ((const uint16_t*) liu8_ptr, (const uint16_t) liu16_size_frame_and_crc);
			break;
	}


	return MODBUS_SLAVE_ERR_NO_ERROR;
}


//------------------------------------------------------------------------------------------------------------------
//		When TX ends we need to stop emitting on the bus
//------------------------------------------------------------------------------------------------------------------
void drv_modbus_handler_tx_ended ( MOSBUS_SLVAE_INSTANCE_t *const liPtrModbusInstance){
	switch ( liPtrModbusInstance->config.u16_uart_id ){
		case 1:
			MB1_DISABLE_TRANSMIT;
			break;
		case 2:
			MB2_DISABLE_TRANSMIT;
			break;
	}
}

//---------------------------------------------------------------------------------
//	send an error frame, with function code and error code
//	the 0x80 is automatically added to provided function code
//---------------------------------------------------------------------------------
#define MODBUS_ERRFRAME_LENGTH		5
static void drv_modbus_send_err_frame ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance, uint16_t liu16_fct_code, const uint16_t liu16_err_code ){
	liu16_fct_code |= MODBUS_FCT_CODE_ERROR;

	//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_slave_adr = liPtrModbusInstance->config.u16_ModbusAddress;
	__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_ADDR) = liPtrModbusInstance->config.u16_ModbusAddress;
	__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_FCTCODE) = liu16_fct_code;
	__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ERRFRAME_POS_ERRCODE) = liu16_err_code;

	drv_modbus_make_and_send_frame (liPtrModbusInstance,  
		(uint16_t *) &liPtrModbusInstance->internal.TrameTXTab,
		MODBUS_ERRFRAME_LENGTH, 
		2*MOSBUS_SLVAE_TRAMETXBUFF_SIZE_U16_CNT );
}



//------------------------------------------------------------------------------------------------------------------
//		Get pointer to structure that contains the corresponding data + length of that tab
//		input is regsiter type (not modbus function code)
//------------------------------------------------------------------------------------------------------------------
void GetRegPtrAndCount (const MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance, const  MODBUS_SLAVE_REG_TYPE_t liu16_RegType, DRV_MODBUS_SLAVE_REGDEF_t ** RegPtrOutPut, uint16_t * RegPtrCnt ){
	switch ( liu16_RegType ){
		case MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL:
			*RegPtrOutPut = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.Ptr;
			*RegPtrCnt = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.NbElmts;
			break;
		case MODBUS_SLAVE_REG_TYPE_OUTPUTREG:
			*RegPtrOutPut = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.Ptr;
			*RegPtrCnt = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.NbElmts;
			break;
		case MODBUS_SLAVE_REG_TYPE_INPUTCOIL:
			*RegPtrOutPut = liPtrModbusInstance->config.RegistersStrPtr->InputCoils.Ptr;
			*RegPtrCnt = liPtrModbusInstance->config.RegistersStrPtr->InputCoils.NbElmts;
			break;
		case MODBUS_SLAVE_REG_TYPE_INPUTREG:
			*RegPtrOutPut = liPtrModbusInstance->config.RegistersStrPtr->InputReg.Ptr;
			*RegPtrCnt = liPtrModbusInstance->config.RegistersStrPtr->InputReg.NbElmts;
			break;
		case MODBUS_SLAVE_REG_TYPE_ERROR:
		default:
			*RegPtrCnt = 0;
			break;
	}
}




//------------------------------------------------------------------------------------------------------------------
//		Convert function code of modbus request to corresponding data register
//------------------------------------------------------------------------------------------------------------------
MODBUS_SLAVE_REG_TYPE_t FunctionCodeToRegType ( const uint16_t liu16_FctCode ){
	switch (liu16_FctCode){
		case MODBUS_FCT_CODE_RD_OUTPUTCOILS:
		case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTCOIL:
		case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL:
			return MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL;

		case MODBUS_FCT_CODE_RD_INPUTCOIL:
			return MODBUS_SLAVE_REG_TYPE_INPUTCOIL;

		case MODBUS_FCT_CODE_RD_INPUTREGS:
			return MODBUS_SLAVE_REG_TYPE_INPUTREG;
			
		case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG:
		case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTREG:
		case MODBUS_FCT_CODE_RD_OUTPUTREG:
			return MODBUS_SLAVE_REG_TYPE_OUTPUTREG;
			
		default:
			return MODBUS_SLAVE_REG_TYPE_ERROR;
			
	}

}




//-------------------------------------------------------------------------------------------
//		return one register value on u16_ptr_data
//		Return 0 if success, 1 otherwise
//-------------------------------------------------------------------------------------------
MODBUS_SLAVE_ERR_t drv_modbus_get_reg_value (MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance,  const MODBUS_SLAVE_REG_TYPE_t liu16_RegType, const uint16_t u16_reg_adr, uint16_t* u16_ptr_data){
	DRV_MODBUS_SLAVE_REGDEF_t	*l_PtrTab;
	uint16_t	lu16_PtrTabSize;
	uint16_t 	u16_index;

	GetRegPtrAndCount ( liPtrModbusInstance, liu16_RegType, &l_PtrTab, &lu16_PtrTabSize);

	for ( u16_index=0; u16_index<lu16_PtrTabSize; u16_index++){
		if ( u16_reg_adr == l_PtrTab[u16_index].u16_adr){
			*u16_ptr_data = l_PtrTab[u16_index].u16_value;
			return MODBUS_SLAVE_ERR_NO_ERROR;
		}
	}

	return MODBUS_SLAVE_ERR_DATA_NOT_AVAILABLE;
}




//-------------------------------------------------------------------------------------------------------------------------
//  @brief Writes a single register to a Modbus slave instance.
//
//  This function handles writing a single register (coil or holding register) to a Modbus slave.
//  It determines the type of register to write (output coil or output register) and then
//  iterates through the appropriate register table to find the matching address.
//  Once found, it updates the register's value and sets the 'updated' flag.
//
//  @param liPtrModbusInstance  :Pointer to the Modbus slave instance structure.
//  @param liu16_RegType   :The type of Modbus register (e.g., MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, MODBUS_SLAVE_REG_TYPE_OUTPUTREG).
//  @param u16_reg_adr  : The 16-bit address of the register to write.
//  @param u16_data_to_write : The 16-bit data to write to the register.
//  @return A Modbus slave error code indicating the success or type of failure of the operation.
//  - MODBUS_SLAVE_ERR_NO_ERROR: The write operation was successful.
//  - MODBUS_SLAVE_ERR_INVALID_VALUE: An invalid value was provided for an output coil (only 0 or U16_COIL_SET_VALUE_TO_ONE are allowed).
//  - MODBUS_SLAVE_ERR_CANNOT_PROCESS: The register type is not supported for writing (e.g., input coils or input registers).
//  - MODBUS_SLAVE_ERR_DATA_NOT_AVAILABLE: The specified register address was not found in the register table.
//-------------------------------------------------------------------------------------------------------------------------
static uint16_t drv_modbus_write_single_register (MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance, const MODBUS_SLAVE_REG_TYPE_t liu16_RegType, const uint16_t u16_reg_adr, const uint16_t u16_data_to_write){
	// Pointer to the start of the register definition table for the given register type.
	DRV_MODBUS_SLAVE_REGDEF_t	*l_PtrTab;
	// Size of the register definition table (number of registers).
	uint16_t	lu16_PtrTabSize;
	uint16_t 	u16_index;

	// Retrieves the pointer to the appropriate register table and its size based on the Modbus instance and register type.
	GetRegPtrAndCount ( liPtrModbusInstance, liu16_RegType, &l_PtrTab, &lu16_PtrTabSize);

	// Branches the execution based on the type of Modbus register being written.
	switch(liu16_RegType){	// Case for writing to an Output Coil (discrete output).

		case MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL:
			for ( u16_index=0; u16_index<lu16_PtrTabSize; u16_index++){		// Iterate through the output coil register table.
				// Check if the current register's address matches the requested address.
				if ( u16_reg_adr == l_PtrTab[u16_index].u16_adr){
					// Further switch to handle the specific data value for an output coil.
					// Coils typically represent binary states (ON/OFF).
					switch ( u16_data_to_write )
					{
						// If the data to write is 0, set the coil value to 0 (OFF).
						case 0:
							l_PtrTab[u16_index].u16_value = 0;
							break;
						// If the data to write is U16_COIL_SET_VALUE_TO_ONE, set the coil value to 1 (ON).
						case U16_COIL_SET_VALUE_TO_ONE: // Assuming U16_COIL_SET_VALUE_TO_ONE is defined as the value for setting a coil to 1 (e.g., 0xFF00)
							l_PtrTab[u16_index].u16_value = 1;
							break;
						// For any other value, it's an invalid write for an output coil.
						default:
							return MODBUS_SLAVE_ERR_INVALID_VALUE; // Return an error indicating an invalid value.
					}
					// Mark the coil as updated.
					l_PtrTab[u16_index].u16_updated = 1;
					// Return success if the coil was found and updated.
					return MODBUS_SLAVE_ERR_NO_ERROR;
				}
			}
			break; // Break from the MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL case.

		// Case for writing to an Output Register (holding register).
		case MODBUS_SLAVE_REG_TYPE_OUTPUTREG:
			// Iterate through the output register table.
			for ( u16_index=0; u16_index<lu16_PtrTabSize; u16_index++){
				// Check if the current register's address matches the requested address.
				if ( u16_reg_adr == l_PtrTab[u16_index].u16_adr){
					// Write the provided data directly to the output register.
					l_PtrTab[u16_index].u16_value = u16_data_to_write;
					// Mark the register as updated.
					l_PtrTab[u16_index].u16_updated = 1;
					// Return success if the register was found and updated.
					return MODBUS_SLAVE_ERR_NO_ERROR;
				}
			}
			break; // Break from the MODBUS_SLAVE_REG_TYPE_OUTPUTREG case.

		// Cases for register types that cannot be written to by this function, or an unknown type.
		case MODBUS_SLAVE_REG_TYPE_INPUTCOIL:   // Input coils are read-only.
		case MODBUS_SLAVE_REG_TYPE_INPUTREG:    // Input registers are read-only.
		case MODBUS_SLAVE_REG_TYPE_ERROR:       // This is an error type, not a register type for writing.
		default:
			// Return an error indicating that the operation cannot be processed for this register type.
			return MODBUS_SLAVE_ERR_CANNOT_PROCESS;
	}
	// If the function reaches this point, it means the requested register address was not found
	// in the appropriate register table for the given register type.
	return MODBUS_SLAVE_ERR_DATA_NOT_AVAILABLE;
}






//-------------------------------------------------------------------------------------------
//		HANDLE all READ commands, that is
//	check function code and data, get data, send frame
//-------------------------------------------------------------------------------------------
static uint16_t drv_modbus_answer_read_regs ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance, const uint16_t liu16_fct_code, uint16_t u16_start_adr, const uint16_t u16_reg_cnt){
	uint16_t 	u16_data;
	uint16_t 	i=0;
	MODBUS_SLAVE_ERR_t	l_error;
	uint16_t	lu16_Mask;
	uint16_t	lu16_ByteIndex;
	MODBUS_SLAVE_REG_TYPE_t 	RegType;
	//uint16_t	*lu16_DataPtr = (uint16_t*)liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab;
	

	__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_ADDR) = liPtrModbusInstance->config.u16_ModbusAddress;
	__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_FCTCODE) = liu16_fct_code;
	

	//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_slave_adr = liPtrModbusInstance->config.u16_ModbusAddress;
	//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_function_code = liu16_fct_code;

	RegType = FunctionCodeToRegType (liu16_fct_code);

	if ( MODBUS_SLAVE_REG_TYPE_ERROR ==  RegType){
		return MODBUS_SLAVE_ERR_ERRWHILEPROCESSINGREQ;
	}
	switch(liu16_fct_code)
	{
		// for these function codes, concatenate the bits to bytes
		case MODBUS_FCT_CODE_RD_OUTPUTCOILS:
		case MODBUS_FCT_CODE_RD_INPUTCOIL:
			lu16_ByteIndex = 0;
			lu16_Mask = 0x01;
			//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab[0] = 0;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_DATA) = 0;
			for ( i=0 ; (i<u16_reg_cnt)&&(lu16_ByteIndex<(MOSBUS_SLVAE_ANSWER_DATA_AND_CRC_SIZE-2)) ; i++)
			{
				if ( 0x0100 <= lu16_Mask){
					lu16_Mask = 0x01;
					lu16_ByteIndex++;
					// liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab[lu16_ByteIndex] = 0;
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_DATA+lu16_ByteIndex) = 0;
				}
				l_error = drv_modbus_get_reg_value (liPtrModbusInstance, RegType, u16_start_adr++, &u16_data);
				if ( 0 != l_error ){
					return MODBUS_SLAVE_ERR_INVALID_VALUE;
				}else{
					if ( 0 != u16_data ){
						// liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab[lu16_ByteIndex] |= lu16_Mask;
						__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_DATA+lu16_ByteIndex) |= lu16_Mask;
					}
					lu16_Mask = lu16_Mask << 1;
				}
			}
			//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_byte_cnt = 1+lu16_ByteIndex;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_BYTE_CNT ) = (0x00FF & ( 1+lu16_ByteIndex) );
			break;

		// for these function code, 2 frame bytes for one returned register
		case MODBUS_FCT_CODE_RD_OUTPUTREG:
		case MODBUS_FCT_CODE_RD_INPUTREGS:
			//liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_byte_cnt = 2*u16_reg_cnt;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_BYTE_CNT ) = (0x00FF & ( 2*u16_reg_cnt ) );

			for ( i=0 ; (i<(2*u16_reg_cnt))&&(i<(MOSBUS_SLVAE_ANSWER_DATA_AND_CRC_SIZE-2)) ; i+=2)
			{
				l_error = drv_modbus_get_reg_value (liPtrModbusInstance, RegType, u16_start_adr++, &u16_data);
				if ( 0 != l_error ){
					return MODBUS_SLAVE_ERR_INVALID_VALUE;
				}else{
					// liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab[i] = (uint16_t) (u16_data >> BYTE_SHIFT);
					// liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_tab[i+1] = (uint16_t) (u16_data & LSB_MASK);
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_DATA+i) = (u16_data >> BYTE_SHIFT);
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_DATA+i+1) = (u16_data & LSB_MASK);
				}
			}
			break;
		default:
			return MODBUS_SLAVE_ERR_INVALID_VALUE;

	}
/*
	return drv_modbus_make_and_send_frame (
			liPtrModbusInstance,
			(uint16_t *) &liPtrModbusInstance->internal.TrameTX.aswr_read.u16_slave_adr,
			3+liPtrModbusInstance->internal.TrameTX.aswr_read.u16_data_byte_cnt+2,
			sizeof (MOSBUS_SLVAE_ANSWER_FRAME_s) );
			*/
	return drv_modbus_make_and_send_frame (
			liPtrModbusInstance,
			(uint16_t *)liPtrModbusInstance->internal.TrameTXTab,
			3+2+ __byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_BYTE_CNT ),
			2*MOSBUS_SLVAE_TRAMETXBUFF_SIZE_U16_CNT );
}




//---------------------------------------------------------------------------------
//	PROCESS the received frame and call the proper response function
//	Handle directly write requests
//---------------------------------------------------------------------------------
static uint16_t drv_modbus_process_request ( MOSBUS_SLVAE_INSTANCE_t *const liPtrModbusInstance){
	uint16_t  	lu16_CntRegToWrite = 0;
	uint16_t		lu16_Err;
	uint16_t	lu16_RegToWrite_Value = 0;
	uint16_t	lu16_RegToWrite_Addr = 0;
	MODBUS_SLAVE_REG_TYPE_t 	RegType;
	uint16_t	lu16_FctCode = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
	uint16_t 	lu16_RegToWrite_index;
	uint16_t	lu16_SrcDataByteIndex = 0;
	uint16_t	lu16_SrcDataMask = 0x01;

	RegType = FunctionCodeToRegType (lu16_FctCode);
	if ( MODBUS_SLAVE_REG_TYPE_ERROR ==  RegType){
		return MODBUS_SLAVE_ERR_ERRWHILEPROCESSINGREQ;
	}

	switch ( lu16_FctCode ){
		case MODBUS_FCT_CODE_RD_OUTPUTCOILS:
		case MODBUS_FCT_CODE_RD_INPUTCOIL:
		case MODBUS_FCT_CODE_RD_OUTPUTREG:
		case MODBUS_FCT_CODE_RD_INPUTREGS:
			lu16_Err = drv_modbus_answer_read_regs(
					liPtrModbusInstance,
					liPtrModbusInstance->internal.TrameRx.Common.u16_function_code,
					liPtrModbusInstance->internal.TrameRx.Common.u16_addr,
					liPtrModbusInstance->internal.TrameRx.ReqType.ReadReq.u16_reg_cnt & LSB_MASK );
			return lu16_Err;
			
		case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTREG:
		case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL:

			if ( MODBUS_SLAVE_ERR_NO_ERROR == drv_modbus_write_single_register (
					liPtrModbusInstance,
					RegType,
					liPtrModbusInstance->internal.TrameRx.Common.u16_addr,
					liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val ) ){
				if ( MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL == liPtrModbusInstance->internal.TrameRx.Common.u16_function_code )
				{
					if ( 0 != liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val ){
						//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_value_MSB = 0xFF;
						__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_REGVALMSB) = 0xFF;
					}else{
						//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_value_MSB = 0x00;
						__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_REGVALMSB) = 0x00;
					}
					//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_value_LSB = 0x00;
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_REGVALLSB ) = 0x00;
				}else{
					//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_value_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val >> BYTE_SHIFT);
					//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_value_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val & LSB_MASK);
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_REGVALMSB ) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val >> BYTE_SHIFT);
					__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_REGVALLSB ) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteSingleReq.u16_val & LSB_MASK);
				}
				//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_slave_adr = liPtrModbusInstance->config.u16_ModbusAddress;
				//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_function_code = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
				__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_ADDR) = liPtrModbusInstance->config.u16_ModbusAddress;
				__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_FCTCODE) = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
				//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_addr_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
				//liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_reg_addr_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
				__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
				__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB+1) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
				/*drv_modbus_make_and_send_frame (
						liPtrModbusInstance,
						(uint16_t *) &liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr.u16_slave_adr,
						sizeof(liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr),
						sizeof (liPtrModbusInstance->internal.TrameTX.writes.aswr_single_wr) );*/
				drv_modbus_make_and_send_frame (
						liPtrModbusInstance,
						(uint16_t *) liPtrModbusInstance->internal.TrameTXTab,
						8,
						8 );
				return MODBUS_SLAVE_ERR_NO_ERROR;
			}else{
				return MODBUS_SLAVE_ERR_INVALID_VALUE;
			}


		case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG:
			lu16_CntRegToWrite = (uint16_t) liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg;
			lu16_RegToWrite_Addr = liPtrModbusInstance->internal.TrameRx.Common.u16_addr;
			for ( lu16_RegToWrite_index = 0; (lu16_RegToWrite_index<lu16_CntRegToWrite)&&(2*lu16_RegToWrite_index<MOSBUS_SLAVE_REQFRAME_DATA_MAX_LENGTH) ; lu16_RegToWrite_index++ )
			{
				//------ get write reg value ------------
				// lu16_RegToWrite_Value = liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues[2*lu16_RegToWrite_index];
				lu16_RegToWrite_Value = __byte( (int*)liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues, 2*lu16_RegToWrite_index );
				lu16_RegToWrite_Value = lu16_RegToWrite_Value << 8;
				// lu16_RegToWrite_Value += liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues[2*lu16_RegToWrite_index+1];
				lu16_RegToWrite_Value += __byte( (int*)liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues, 2*lu16_RegToWrite_index+1);

				if ( MODBUS_SLAVE_ERR_NO_ERROR != drv_modbus_write_single_register ( liPtrModbusInstance, RegType, lu16_RegToWrite_Addr, lu16_RegToWrite_Value ) ){
					return MODBUS_SLAVE_ERR_INVALID_VALUE;
				}
				lu16_RegToWrite_Addr++;
			}
			//-- prepare and send response
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_slave_adr = liPtrModbusInstance->config.u16_ModbusAddress;
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_function_code = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_reg_addr_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_reg_addr_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_nb_of_reg_written_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg >> BYTE_SHIFT);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_nb_of_reg_written_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg & LSB_MASK );

			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_ADDR) = liPtrModbusInstance->config.u16_ModbusAddress;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_FCTCODE) = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB+1) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_NBREGSMSB) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg >> BYTE_SHIFT);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_NBREGSMSB+1) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg & LSB_MASK );

			/*drv_modbus_make_and_send_frame (
					liPtrModbusInstance,
					(uint16_t *) &liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_slave_adr,
					sizeof(liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr),
					sizeof (liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr) );*/
			drv_modbus_make_and_send_frame ( liPtrModbusInstance, (uint16_t *) liPtrModbusInstance->internal.TrameTXTab, 8, 8 );
			return MODBUS_SLAVE_ERR_NO_ERROR;

		case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTCOIL:
			lu16_SrcDataByteIndex = 0;
			lu16_SrcDataMask = 0x01;
			lu16_CntRegToWrite = (uint16_t) liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg;
			lu16_RegToWrite_Addr = liPtrModbusInstance->internal.TrameRx.Common.u16_addr;
			for ( lu16_RegToWrite_index = 0; lu16_RegToWrite_index<lu16_CntRegToWrite ; lu16_RegToWrite_index++ )
			{
				//------ get write reg value ------------
				// if (lu16_SrcDataMask & liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValuesu16_TabValues[lu16_SrcDataByteIndex]) {
				if (lu16_SrcDataMask & __byte( (int*)(liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues), lu16_SrcDataByteIndex) ) {
					lu16_RegToWrite_Value = 0xFF00; // Si le bit est à 1, la valeur Modbus pour ON est 0xFF00
				} else {
					lu16_RegToWrite_Value = 0x0000; // Si le bit est à 0, la valeur Modbus pour OFF est 0x0000
				}

				lu16_SrcDataMask = lu16_SrcDataMask << 1;
				if ( 0 == lu16_SrcDataMask){
					lu16_SrcDataByteIndex++;
					lu16_SrcDataMask = 0x01;
				}

				if ( MODBUS_SLAVE_ERR_NO_ERROR != drv_modbus_write_single_register ( liPtrModbusInstance, RegType, lu16_RegToWrite_Addr, lu16_RegToWrite_Value ) ){
					return MODBUS_SLAVE_ERR_INVALID_VALUE;
				}
				lu16_RegToWrite_Addr++;
			}
			//-- prepare and send response
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_slave_adr = liPtrModbusInstance->config.u16_ModbusAddress;
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_function_code = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_reg_addr_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_reg_addr_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_nb_of_reg_written_MSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg >> BYTE_SHIFT);
			//liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_nb_of_reg_written_LSB = (uint16_t) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg & LSB_MASK );

			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_ADDR) = liPtrModbusInstance->config.u16_ModbusAddress;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_POS_FCTCODE) = liPtrModbusInstance->internal.TrameRx.Common.u16_function_code;
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr >> BYTE_SHIFT);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR05_POS_REGADRMSB+1) = (liPtrModbusInstance->internal.TrameRx.Common.u16_addr & LSB_MASK);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_NBREGSMSB) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg >> BYTE_SHIFT);
			__byte( (int*) (liPtrModbusInstance->internal.TrameTXTab), MODBUSSLAVE_ASWWR15_POS_NBREGSMSB+1) = (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg & LSB_MASK );


/*
			drv_modbus_make_and_send_frame (
					liPtrModbusInstance,
					(uint16_t *) &liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr.u16_slave_adr,
					sizeof(liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr),
					sizeof (liPtrModbusInstance->internal.TrameTX.writes.aswr_multiples_wr) );
					*/
			drv_modbus_make_and_send_frame ( liPtrModbusInstance, (uint16_t *) liPtrModbusInstance->internal.TrameTXTab, 8, 8 );
			return MODBUS_SLAVE_ERR_NO_ERROR;

		default:
			return MODBUS_SLAVE_ERR_INVALID_VALUE;
	}
	//return MODBUS_SLAVE_ERR_INVALID_VALUE;
}










//---------------------------------------------------------------------------------
//			RECEIVER state machine
//---------------------------------------------------------------------------------
void drv_modbus_slave_rxtUartHandler ( MOSBUS_SLVAE_INSTANCE_t * const liPtrModbusInstance, const uint16_t li_u16_char ){
	uint16_t		lu16_NbExpectedBytes=0;
	uint16_t		lu16_ErrCode;


	// compute the CRC
	switch ( liPtrModbusInstance->internal.State.u16_etat ){
		case MODBUS_SLAVE_STATE_WAIT_RX_ADR:
			liPtrModbusInstance->internal.u16_crc = DRV_MODBUS_CRC_INIT_VALUE;
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_FCODE:
		case MODBUS_SLAVE_STATE_WAIT_RX_ADRMSB:
		case MODBUS_SLAVE_STATE_WAIT_RX_ADRLSB:
		case MODBUS_SLAVE_STATE_WAIT_RX_LENGTHMSB:
		case MODBUS_SLAVE_STATE_WAIT_RX_LENGTHLSB:
		case MODBUS_SLAVE_STATE_WAIT_RX_NBOFBYTES:
		case MODBUS_SLAVE_STATE_WAIT_RX_MULTIPLEDATA:
			drv_modbus_slave_Timeout_EnableAndReset ( liPtrModbusInstance );
			liPtrModbusInstance->internal.u16_crc = drv_modbus_crc_crc16_update ( liPtrModbusInstance->internal.u16_crc, li_u16_char );
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_CRCMSB:
		case MODBUS_SLAVE_STATE_WAIT_RX_CRCLSB:
			break;

		default:
			break;
	}


	// process the received char
	switch ( liPtrModbusInstance->internal.State.u16_etat )
	{
		// in this state we are waiting to receive the address of the frame.
		case MODBUS_SLAVE_STATE_WAIT_RX_ADR:
			if ( li_u16_char == liPtrModbusInstance->config.u16_ModbusAddress ){
				liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_FCODE;
				liPtrModbusInstance->internal.TrameRx.Common.u16_slave_adr = li_u16_char;
				liPtrModbusInstance->internal.u16_crc = drv_modbus_crc_crc16_update ( liPtrModbusInstance->internal.u16_crc, li_u16_char );
				drv_modbus_slave_Timeout_EnableAndReset ( liPtrModbusInstance );

			//-- the byte received is not the address, so it's the frame for another slave.
			// then wait for the timeout of reception that is no more byte sent, frame finished
			}else{
				liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME;
				drv_modbus_slave_Timeout_EnableAndReset ( liPtrModbusInstance );
			}
			break;
		// waiting to receive the function code
		case MODBUS_SLAVE_STATE_WAIT_RX_FCODE:
			liPtrModbusInstance->internal.TrameRx.Common.u16_function_code = li_u16_char;
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADRMSB;
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_ADRMSB:
			liPtrModbusInstance->internal.TrameRx.Common.u16_addr = li_u16_char << BYTE_SHIFT;
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADRLSB;
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_ADRLSB:
			liPtrModbusInstance->internal.TrameRx.Common.u16_addr += li_u16_char;

			switch ( liPtrModbusInstance->internal.TrameRx.Common.u16_function_code ){
				case MODBUS_FCT_CODE_RD_OUTPUTCOILS:
				case MODBUS_FCT_CODE_RD_INPUTCOIL:
				case MODBUS_FCT_CODE_RD_OUTPUTREG:
				case MODBUS_FCT_CODE_RD_INPUTREGS:
				case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL:
				case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTREG:
				case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTCOIL:
				case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG:
					liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_LENGTHMSB;
					break;
				default:
					//liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADR;
					//drv_modbus_slave_TimeoutTimer_Disable ( liPtrModbusInstance );
					//while(1);
					//-- in that case do not send error frame (RX is ongoing) and wait for the end of the frame
					liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME;
					break;
			}
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_LENGTHMSB:
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_LENGTHLSB;
			liPtrModbusInstance->internal.TrameRx.ReqType.ReadReq.u16_reg_cnt = li_u16_char << BYTE_SHIFT;
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_LENGTHLSB:
			liPtrModbusInstance->internal.TrameRx.ReqType.ReadReq.u16_reg_cnt += li_u16_char;

			switch ( liPtrModbusInstance->internal.TrameRx.Common.u16_function_code ){
				case MODBUS_FCT_CODE_RD_OUTPUTCOILS:
				case MODBUS_FCT_CODE_RD_INPUTCOIL:
				case MODBUS_FCT_CODE_RD_OUTPUTREG:
				case MODBUS_FCT_CODE_RD_INPUTREGS:
				case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTCOIL:
				case MODBUS_FCT_CODE_WR_SINGLE_OUTPUTREG:
					liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_CRCMSB;
					break;
				case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTCOIL:
				case MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG:
					liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_NBOFBYTES;
					break;
				default:
					//-- in that case do not send error frame (RX is ongoing) and wait for the end of the frame
					liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME;
					break;
			}
			break;

		//- receive the number of following bytes
		case MODBUS_SLAVE_STATE_WAIT_RX_NBOFBYTES:
			liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NbBytes = li_u16_char;
			liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg = liPtrModbusInstance->internal.TrameRx.ReqType.ReadReq.u16_reg_cnt;

			//--- various checks
			if ( MODBUS_FCT_CODE_WR_MULTIPLE_OUTPUTREG == liPtrModbusInstance->internal.TrameRx.Common.u16_function_code ){
				lu16_NbExpectedBytes = (uint16_t) (2 * liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg );
			}else{
				lu16_NbExpectedBytes = (uint16_t) ( liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg/8);
				if ( 0 != ( liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NumberReg % 8) ){
					lu16_NbExpectedBytes++;
				}
			}
			if (
					((uint16_t) liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NbBytes != lu16_NbExpectedBytes ) ||
					( liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NbBytes > MOSBUS_SLAVE_REQFRAME_DATA_MAX_LENGTH )
			){
				drv_modbus_slave_TimeoutTimer_Disable ( liPtrModbusInstance );
				drv_modbus_send_err_frame ( liPtrModbusInstance, liPtrModbusInstance->internal.TrameRx.Common.u16_function_code, MODBUS_SLAVE_ERR_INVALID_VALUE );
				liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADR;
				break;
			}else{
				liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_MULTIPLEDATA;
				liPtrModbusInstance->internal.State.u16_rx_buff_index = 0;
			}
			break;

		//-- read the "following" bytes that is the data to be written by write req when multiple write req
		case MODBUS_SLAVE_STATE_WAIT_RX_MULTIPLEDATA:
			// liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues[liPtrModbusInstance->internal.State.u16_rx_buff_index++] = li_u16_char;
			__byte( (int*) (liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_TabValues), liPtrModbusInstance->internal.State.u16_rx_buff_index) = li_u16_char;
			liPtrModbusInstance->internal.State.u16_rx_buff_index++;
			if ( liPtrModbusInstance->internal.State.u16_rx_buff_index >= liPtrModbusInstance->internal.TrameRx.ReqType.WriteMultipleReq.u16_NbBytes ){
				liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_CRCMSB;
			}
			break;

		case MODBUS_SLAVE_STATE_WAIT_RX_CRCMSB:
			liPtrModbusInstance->internal.TrameRx.Common.u16_crc = li_u16_char << BYTE_SHIFT;
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_CRCLSB;
			break;
		case MODBUS_SLAVE_STATE_WAIT_RX_CRCLSB:
			liPtrModbusInstance->internal.TrameRx.Common.u16_crc += li_u16_char;

			drv_modbus_slave_TimeoutTimer_Disable ( liPtrModbusInstance );

			// check if CRC is ok
			liPtrModbusInstance->internal.u16_crc = (liPtrModbusInstance->internal.u16_crc<<BYTE_SHIFT) | (liPtrModbusInstance->internal.u16_crc>>BYTE_SHIFT);	//swap the fucking crc
			if ( liPtrModbusInstance->internal.u16_crc == liPtrModbusInstance->internal.TrameRx.Common.u16_crc){
				lu16_ErrCode = drv_modbus_process_request (liPtrModbusInstance);
				if ( MODBUS_SLAVE_ERR_NO_ERROR != lu16_ErrCode){
					drv_modbus_send_err_frame (liPtrModbusInstance, liPtrModbusInstance->internal.TrameRx.Common.u16_function_code, lu16_ErrCode );
				}
			}else{
				drv_modbus_send_err_frame (liPtrModbusInstance, liPtrModbusInstance->internal.TrameRx.Common.u16_function_code, MODBUS_SLAVE_ERR_INVALID_VALUE );
			}
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_RX_ADR;
			break;

		case MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME:
			break;

		// error !!
		default:
			//-- in that case do not send error frame (RX is ongoing) and wait for the end of the frame
			liPtrModbusInstance->internal.State.u16_etat = MODBUS_SLAVE_STATE_WAIT_END_OTHERSLAVE_FRAME;
			break;

	}
}





//------------------------------------------------------------------------------------------------------------------
//	see if one reg has been updated (written), return 0 if no registers updated, 1 if at least one reg updated
//	if one reg is updated, address and value returned in liPtrUpdatedRegValue
//------------------------------------------------------------------------------------------------------------------
uint16_t drv_modbus_slave_GetUpdatedRegValue (MOSBUS_SLVAE_INSTANCE_t *const liPtrModbusInstance, MODBUS_SLAVE_UPDATED_REG_VALUE_t * const liPtrUpdatedRegValue){
	DRV_MODBUS_SLAVE_REGDEF_t	*l_PtrTab;
	uint16_t	lu16_TabSize;
	uint16_t 	u16_index;

	l_PtrTab = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.Ptr;
	lu16_TabSize = liPtrModbusInstance->config.RegistersStrPtr->OutputCoils.NbElmts;
	for ( u16_index=0; u16_index<lu16_TabSize; u16_index++){
		if ( 0 != l_PtrTab[u16_index].u16_updated){
			liPtrUpdatedRegValue->u16_reg_new_val = l_PtrTab[u16_index].u16_value;
			liPtrUpdatedRegValue->u16_reg_adr = l_PtrTab[u16_index].u16_adr;
			liPtrUpdatedRegValue->reg_type = MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL;
			l_PtrTab[u16_index].u16_updated = 0;
			return 1;
		}
	}
	l_PtrTab = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.Ptr;
	lu16_TabSize = liPtrModbusInstance->config.RegistersStrPtr->OutputReg.NbElmts;
	for ( u16_index=0; u16_index<lu16_TabSize; u16_index++){
		if ( 0 != l_PtrTab[u16_index].u16_updated){
			liPtrUpdatedRegValue->u16_reg_new_val = l_PtrTab[u16_index].u16_value;
			liPtrUpdatedRegValue->u16_reg_adr = l_PtrTab[u16_index].u16_adr;
			liPtrUpdatedRegValue->reg_type = MODBUS_SLAVE_REG_TYPE_OUTPUTREG;
			l_PtrTab[u16_index].u16_updated = 0;
			return 1;
		}
	}
	return 0;
}

//------------------------------------------------------------------------------------------------------------------
//	set a new value for a register, return 0 if succeed
//------------------------------------------------------------------------------------------------------------------
uint16_t drv_modbus_slave_SetNewRegValue ( MOSBUS_SLVAE_INSTANCE_t *liPtrModbusInstance, const MODBUS_SLAVE_REG_TYPE_t liu16_RegType, const uint16_t u16_reg_adr, const uint16_t u16_reg_value ){

	DRV_MODBUS_SLAVE_REGDEF_t	*l_PtrTab;
	uint16_t	lu16_TabSize;
	uint16_t 	u16_index;

	GetRegPtrAndCount ( liPtrModbusInstance, liu16_RegType, &l_PtrTab, &lu16_TabSize);

	for ( u16_index=0; u16_index<lu16_TabSize; u16_index++){
		if ( u16_reg_adr == l_PtrTab[u16_index].u16_adr){
			l_PtrTab[u16_index].u16_value = u16_reg_value;
			return 0;
		}
	}
	return 2;

}







