

#include "drv_modbus_slave.h"




MOSBUS_SLVAE_INSTANCE_t		G_ModbusSlave_instance;

MODBUS_SLAVE_UPDATED_REG_VALUE_t	reg_cfg;


void app_wrapper_modbuslavetimeoutcallback (){
	drv_modbus_slave_TimeoutTimer_callback (&G_ModbusSlave_instance);
}
void wrapper_modbus1_drv_modbus_slave_rxtUartHandler ( uint16_t li_data ){
	drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, li_data );
}
void  wrapper_modbus1_drv_modbus_handler_tx_ended (){
	 drv_modbus_handler_tx_ended (&G_ModbusSlave_instance);
}

#define MB_OUTPUTCOIL_BASEADR	0x006D
#define MB_INTPUTCOIL_BASEADR	0x008D
#define MB_OUTPUTREG_BASEADR	0x00AD
#define MB_INPUTREG_BASEADR		0x00BD


DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_OutputRegs[]=
{
		{ MB_OUTPUTREG_BASEADR + 0, 0, 0},
		{ MB_OUTPUTREG_BASEADR + 1, 0, 0},
		{ MB_OUTPUTREG_BASEADR + 2, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_IntputRegs[]=
{
		{ MB_INPUTREG_BASEADR + 0 , 0, 0},
		{ MB_INPUTREG_BASEADR + 1 , 1, 0},
		{ MB_INPUTREG_BASEADR + 2 , 1, 0},
		{ MB_INPUTREG_BASEADR + 3 , 0, 0},
		{ MB_INPUTREG_BASEADR + 4 , 0, 0},
		{ MB_INPUTREG_BASEADR + 5 , 1, 0},
		{ MB_INPUTREG_BASEADR + 6 , 1, 0},
		{ MB_INPUTREG_BASEADR + 7 , 0, 0},
		{ MB_INPUTREG_BASEADR + 8 , 0, 0},
		{ MB_INPUTREG_BASEADR + 9 , 0, 0},
		{ MB_INPUTREG_BASEADR + 10, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_OutputCoils[]=
{
		{ MB_OUTPUTCOIL_BASEADR + 0, 0, 0},
		{ MB_OUTPUTCOIL_BASEADR + 1, 1, 0},
		{ MB_OUTPUTCOIL_BASEADR + 2, 1, 0},
		{ MB_OUTPUTCOIL_BASEADR + 3, 0, 0},
		{ MB_OUTPUTCOIL_BASEADR + 4, 0, 0},
		{ MB_OUTPUTCOIL_BASEADR + 5, 1, 0},
		{ MB_OUTPUTCOIL_BASEADR + 6, 1, 0},
		{ MB_OUTPUTCOIL_BASEADR + 7, 0, 0},
		{ MB_OUTPUTCOIL_BASEADR + 8, 0, 0}
};

DRV_MODBUS_SLAVE_REGDEF_t		Mb1_TableReg_Inputcoils[]=
{
		{ MB_INTPUTCOIL_BASEADR + 0, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 1, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 2, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 3, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 4, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 5, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 6, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 7, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 8, 0, 0},
		{ MB_INTPUTCOIL_BASEADR + 9, 0, 0}
};

DRV_MODBUS_SLAVE_REGBASE_t	Mb1__ModbusCfg = {
		{ Mb1_TableReg_OutputRegs,	sizeof(Mb1_TableReg_OutputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_OutputCoils, sizeof(Mb1_TableReg_OutputCoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_IntputRegs, 	sizeof(Mb1_TableReg_IntputRegs) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
		{ Mb1_TableReg_Inputcoils, 	sizeof(Mb1_TableReg_Inputcoils) / sizeof(DRV_MODBUS_SLAVE_REGDEF_t)	} ,
};




void app_modbus_init (){

	G_ModbusSlave_instance.config.RegistersStrPtr = &Mb1__ModbusCfg;
	G_ModbusSlave_instance.config.u16_ModbusAddress = 0x12;
	G_ModbusSlave_instance.config.u16_uart_id = 1;

	drv_modbus_slave_init ( &G_ModbusSlave_instance );

	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0080, 0x1234 );
	drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x0081, 0x5678 );

}


void app_modbus_autom (){


	  // if holding register 0x006B has been updated (written by modbus),
	  // take the value, add 1, and write it on INPUT register
	  // at adress 0x6C, one can read this register to see the new value
	  while ( 0 != drv_modbus_slave_GetUpdatedRegValue (&G_ModbusSlave_instance, &reg_cfg) )
	  {
		  // depending on the type of register
		  switch ( reg_cfg.reg_type )
		  {
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTREG:

				  // depending on the address of the register
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006B:
						  //reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  //reg_cfg2.u16_reg_adr = 0x6D;
						  //reg_cfg2.u16_reg_new_val = reg_cfg.u16_reg_new_val + 1;
						  //drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, 0x006D, reg_cfg2.u16_reg_new_val );
						  break;
					  default:
						  break;
				  }
				  break;

			  //-- un registre coil output a été mis à jour
			  case MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL:
				  switch ( reg_cfg.u16_reg_adr )
				  {
					  case 0x006C:
						  //reg_cfg2.reg_type = MODBUS_SLAVE_REG_TYPE_INPUTREG;
						  //reg_cfg2.u16_reg_adr = 0x6D;

						 // if (0 == reg_cfg.u16_reg_new_val ){
						//	  reg_cfg2.u16_reg_new_val = 0;
						 // }else{
						//	  reg_cfg2.u16_reg_new_val = 123;
						 // }
						  break;
					  default:
						  break;
				  }
				  break;

			//-- these types are not written by modbus master, so no update on these, but put there for compiler warning
			case MODBUS_SLAVE_REG_TYPE_INPUTCOIL:
			case MODBUS_SLAVE_REG_TYPE_INPUTREG:
			case MODBUS_SLAVE_REG_TYPE_ERROR:
			  default:
				  break;

		  }

	  }




}

struct  {
	uint16_t	Valid_BuffRx[10];
	uint16_t	u16_CntBytesToReceive;
	uint16_t	u16_ErrCode;
}Valid_RxStr;

struct{
	uint16_t	Valid_BuffTx[10];
	uint16_t	u16_CntBytesToSend;
	uint16_t	u16_ErrCode;
}Valid_TxStr;

//uint16_t    BuffTx[10];

//--- this function sinmulates the TX function of uart to validate what is been sent
//  return :  0 if ok, otherwise 1
void app_modbus_valid_TxCharFunction (const uint16_t *liu16_ptr, const uint16_t liu16_size){
	uint16_t u16_cpt=0;
	
	Valid_TxStr.u16_ErrCode = 0;

	if ( liu16_size != Valid_TxStr.u16_CntBytesToSend){
		Valid_TxStr.u16_ErrCode = 10;
		return;
	}
	for ( u16_cpt=0 ; u16_cpt<Valid_TxStr.u16_CntBytesToSend ; u16_cpt++){
		if (  __byte( (int*) liu16_ptr,u16_cpt ) != __byte( (int*) (Valid_TxStr.Valid_BuffTx), u16_cpt )   ){
			Valid_TxStr.u16_ErrCode = 9;
			return;
		}
	}
	return;
}
//-------------------------------------------------
//	Function code 1 : READ COILS -> read digital output
// read of 4 coils 
//-------------------------------------------------
void app_modbustest_fct01_t1 ( ) {
	uint16_t	u16_regaddr = MB_OUTPUTCOIL_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x01;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x04;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xAE;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xB7;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	//	12 01 01 (byte count) 06 (data value coil data is sent from msb to lsb in sent byte)  D5 0E
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x06;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xD5;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x0E;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 1 : READ COILS -> read digital output
// read of 8 coils 
//-------------------------------------------------
void app_modbustest_fct01_t2 ( ) {
	uint16_t	u16_regaddr = MB_OUTPUTCOIL_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x01;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x08;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xAE;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xB2;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	//	12 01 01 (byte count) 06 (data value coil data is sent from msb to lsb in sent byte)  D5 0E
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x76;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xD4;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xEA;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 1 : READ COILS -> read digital output
// read of 9 coils 
//-------------------------------------------------
void app_modbustest_fct01_t3 ( ) {
	uint16_t	u16_regaddr = MB_OUTPUTCOIL_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x01;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x09;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x6F;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x72;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTCOIL, u16_regaddr++, 1);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x76;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xDB;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x9F;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 2 : READ DISCRETE INPUT -> read digital intput
// read of 7 inputs
//-------------------------------------------------
void app_modbustest_fct02_t1 ( ) {
	uint16_t	u16_regaddr = MB_INTPUTCOIL_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x02;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x07;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xAB;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x40;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x76;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x24;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xEA;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 2 : READ DISCRETE INPUT -> read digital intput
// read of 10 inputs
//-------------------------------------------------
void app_modbustest_fct02_t2 ( ) {
	uint16_t	u16_regaddr = MB_INTPUTCOIL_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x02;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8 );
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) =(u16_regaddr & 0x00FF );
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x0A;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x6A;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x85;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 1);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTCOIL, u16_regaddr++, 0);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x76;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x01;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xDB;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xDB;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 3 : READ Multiple holding registers -> output register
// read of 1 register
//-------------------------------------------------
void app_modbustest_fct03_t1 ( ) {
	uint16_t	u16_regaddr = MB_OUTPUTREG_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x03;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x01;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x17;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x48;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTREG, u16_regaddr++, 0x4589);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x03;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x45;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x89;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xCE;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xB1;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}
//-------------------------------------------------
//	Function code 3 : READ Multiple holding registers -> output register
// read of 2 registers
//-------------------------------------------------
void app_modbustest_fct03_t2 ( ) {
	uint16_t	u16_regaddr = MB_OUTPUTREG_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x03;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x02;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x57;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x49;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTREG, u16_regaddr++, 0x4589);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_OUTPUTREG, u16_regaddr++, 0x78EF);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x03;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x04;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x45;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x89;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x78;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xEF;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x7F;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x98;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}

//-------------------------------------------------
//	Function code 4 : READ Multiple input registers -> input register
// read of 1 register
//-------------------------------------------------
void app_modbustest_fct04_t1 ( ) {
	uint16_t	u16_regaddr = MB_INPUTREG_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x04;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x01;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0xA3;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x4D;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, u16_regaddr++, 0x1234);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x04;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x02;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x34;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x31;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x84;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}

//-------------------------------------------------
//	Function code 4 : READ Multiple input registers -> input register
// read of 3 register
//-------------------------------------------------
void app_modbustest_fct04_t2 ( ) {
	uint16_t	u16_regaddr = MB_INPUTREG_BASEADR;
	uint16_t    u16_IndexSend = 0;
	uint16_t	u16_ErrSet = 0;

	Valid_RxStr.u16_CntBytesToReceive = 0;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x12;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x04;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr >> 8);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = (u16_regaddr & 0x00FF);
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x00;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x03;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x22;
    __byte((int*)Valid_RxStr.Valid_BuffRx,Valid_RxStr.u16_CntBytesToReceive++) = 0x8C;

	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, u16_regaddr++, 0x1234);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, u16_regaddr++, 0x5678);
	u16_ErrSet |= drv_modbus_slave_SetNewRegValue (&G_ModbusSlave_instance, MODBUS_SLAVE_REG_TYPE_INPUTREG, u16_regaddr++, 0x9ABC);
	if ( 0 != u16_ErrSet ){
		return;
	}

	//retour attendu
	Valid_TxStr.u16_CntBytesToSend = 0;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x04;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x06;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x12;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x34;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x56;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x78;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x9A;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xBC;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0xF1;
	__byte ( (int*) Valid_TxStr.Valid_BuffTx, Valid_TxStr.u16_CntBytesToSend++) = 0x95;

    for (u16_IndexSend=0 ; u16_IndexSend<Valid_RxStr.u16_CntBytesToReceive ; u16_IndexSend++){
        drv_modbus_slave_rxtUartHandler ( &G_ModbusSlave_instance, __byte((int*)Valid_RxStr.Valid_BuffRx,u16_IndexSend) );
    }
}



void app_modbus_test (){
    

	app_modbus_init ();
	drv_modbus_slave_init  (&G_ModbusSlave_instance);
/*
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct01_t1 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}

	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct01_t2 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}

	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct01_t3 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct02_t1 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct02_t2 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}

	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct03_t1 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}
	
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct03_t2 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}
	*/
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct04_t1 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}
	Valid_TxStr.u16_ErrCode = 32000;
	app_modbustest_fct04_t2 ();
	if (  0!= Valid_TxStr.u16_ErrCode ){
		while(1);
	}


	app_wrapper_modbuslavetimeoutcallback ();
	app_wrapper_modbuslavetimeoutcallback ();
	app_wrapper_modbuslavetimeoutcallback ();
	app_wrapper_modbuslavetimeoutcallback ();
	app_wrapper_modbuslavetimeoutcallback ();



	// tester la fonction drv_modbus_slave_SetNewRegValue  pour mauvaise adresse et mauvais type de reg


}

