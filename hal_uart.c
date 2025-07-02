





#include "DSP2803x_Device.h"
#include "stdint.h"

#define Uint16   uint16_t 
#define Uint32   uint32_t 


#include "DSP2803x_PieVect.h"
#include "DSP2803x_PieCtrl.h"
#include "DSP2803x_Sci.h"
#include "DSP2803x_Gpio.h"

// Définitions et variables globales
#define RS485_TX_ENABLE_GPIO    GPIO6  // Exemple de GPIO pour l'activation de l'émetteur/récepteur RS485 (à adapter)

#define RS485_ENABLE_TX     GpioDataRegs.GPASET.bit.RS485_TX_ENABLE_GPIO = 1
#define RS485_DISABLE_TX    GpioDataRegs.GPACLEAR.bit.RS485_TX_ENABLE_GPIO = 1


volatile uint16_t *g_tx_buffer_ptr;
volatile uint16_t g_tx_buffer_size;
volatile uint16_t g_tx_index;
volatile uint16_t g_tx_in_progress = 0;

// Fonctions de rappel pour les interruptions (déclarations forward)
//interrupt void sciaTxFifoIsr(void);
//interrupt void sciaRxFifoIsr(void);
interrupt void hal_uart1_Interrupt      (void) ;
interrupt void hal_uart1_Rx_Interrupt   ( void );


// ----------------------------------------------------------------------------------------
//          Fonction d'Initialisation de l'UART et des Broches
//
//      Cette fonction configure le module SCI-A du TMS320F28034 pour une communication UART asynchrone à 9600 bauds, 
//      8 bits de données, pas de parité et 1 bit d'arrêt. Elle configure également les broches GPIO associées au SCI 
//      et la broche de contrôle de l'émetteur/récepteur RS485.
// ----------------------------------------------------------------------------------------
void hal_uart1_init(void) {
    EALLOW; // Permettre l'accès aux registres protégés

    g_tx_in_progress = 0;
    g_tx_buffer_size = 0;

    // on dirait SYSCLKOUT= 65 Mhz
    // LSPCLK = 16.25 Mhz

    // Configuration des broches GPIO pour SCI-A
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Activer pull-up pour SCIRXDA (GPIO28)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Configurer SCIRXDA sur asynchrone
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // Associer GPIO28 à SCIRXDA

    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Activer pull-up pour SCITXDA (GPIO29)
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // Associer GPIO29 à SCITXDA

    RS485_DISABLE_TX;

    // Configuration de la broche de contrôle RS485 (exemple)
    //GpioCtrlRegs.GPAPUD.bit.RS485_TX_ENABLE_GPIO = 0;   // Activer pull-up
    //GpioCtrlRegs.GPAMUX2.bit.RS485_TX_ENABLE_GPIO = 0;  // Configurer comme GPIO
    //GpioCtrlRegs.GPADIR.bit.RS485_TX_ENABLE_GPIO = 1;   // Configurer comme sortie
    //GpioDataRegs.GPACLEAR.bit.RS485_TX_ENABLE_GPIO = 1; // Désactiver le transceiver par défaut (RX mode)

    EDIS; // Désactiver l'accès aux registres protégés

    // Initialisation du module SCI-A
    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit, No parity, 8-bit character (0x0007)
    

    SciaRegs.SCICTL1.all = 0x0002;  // Enable SCI, Enable TX, Disable RX (pour l'instant)
    SciaRegs.SCICTL1.bit.RXENA = 1;

    //SciaRegs.SCICTL2.bit.TXINTENA = 1; // Activer l'interruption TX vide du FIFO
    //SciaRegs.SCICTL2.bit.RXBKINTENA = 1; // Activer l'interruption RX prête
    SciaRegs.SCICTL2.all = 0;

    // Calcul du baud rate (LSB et H dans SCIHBAUD et SCILBAUD)  
    SciaRegs.SCIHBAUD = 0; // MSB du Baud rate
    SciaRegs.SCILBAUD = 194;  // LSB du Baud rate (pour 9600 bauds)

    // Configuration des FIFO
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFTX.all = 0xE000;
    SciaRegs.SCIFFCT.all = 0x0;

    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;
    //SciaRegs.SCIFFTX.all = 0x0021;

    // Configuration des interruptions au niveau du PIE

    EALLOW; // Permettre l'accès aux registres protégés
    PieVectTable.SCITXINTA = &hal_uart1_Interrupt; // Pointer vers la routine d'interruption TX
    PieVectTable.SCIRXINTA = &hal_uart1_Rx_Interrupt; // Pointer vers la routine d'interruption RX

    //SciaRegs.SCICTL2.bit.RXBKINTENA = 1; // Activer l'interruption RX prête

    EDIS; // Désactiver l'accès aux registres protégés

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1; // Activer SCI-A RX dans le groupe 9 du PIE
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1; // Activer SCI-A TX dans le groupe 9 du PIE

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Activer le PIE

    SciaRegs.SCICTL1.bit.SWRESET = 1;

    IER |= M_INT9; // Activer l'interruption groupe 9 dans le CPU

    EINT;   // Activer les interruptions globales (INTM)
    ERTM;   // Activer les interruptions en temps réel (DBGM)

    

}


// ----------------------------------------------------------------------------------------
//      lancement d'une transmission par IT
// ----------------------------------------------------------------------------------------
void hal_uart1_transmit_IT(const uint16_t *liu16_ptr, const uint16_t liu16_size) {
    g_tx_buffer_ptr = (uint16_t*) liu16_ptr;
    g_tx_buffer_size = liu16_size;
    g_tx_index = 0;

    // Activer le transceiver RS485 en mode émission
    RS485_ENABLE_TX;

    //-- fill up the fifo with bytes to be sent
    while  ( (g_tx_index<g_tx_buffer_size) && (SciaRegs.SCIFFTX.bit.TXFFST < 4) ){
        SciaRegs.SCITXBUF = __byte( (int*) g_tx_buffer_ptr, g_tx_index++ );
    }

    // Effacer le flag d'interruption TX FIFO si déjà défini
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  //enable IT for the fifo 

}



// ----------------------------------------------------------------------------------------
//      ISR  de tansmission par interruption
// ----------------------------------------------------------------------------------------
interrupt void hal_uart1_Interrupt(void) {

    // fin de transmission on désactive les IT et on attends la transmission
    // of the last byte to assert the deactivation of modbus tansceiver
    if ( g_tx_index >= g_tx_buffer_size ){
        g_tx_buffer_size = 0;
        g_tx_index = 0;
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0; 
        SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;

        while (SciaRegs.SCICTL2.bit.TXEMPTY == 0){        }
        RS485_DISABLE_TX;
    }

    //-- fill up the fifo with bytes to be sent
    while  ( (g_tx_index<g_tx_buffer_size) && (SciaRegs.SCIFFTX.bit.TXFFST < 4) ){
        SciaRegs.SCITXBUF = __byte( (int*) g_tx_buffer_ptr, g_tx_index++ );
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1; // Effacer le flag d'interruption TX FIFO
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Acquitter l'interruption au niveau du PIE

}
extern uint16_t  gbuff[10];

interrupt void hal_uart1_Rx_Interrupt ( void ) {
    volatile uint16_t   lu16_Char;

    lu16_Char = SciaRegs.SCIRXBUF.all;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR  =1;

    __byte ( gbuff, 0) = ( lu16_Char & 0x00FF);
}


/*

// ----------------------------------------------------------------------------------------
//      fonction d'interruption RX
// ----------------------------------------------------------------------------------------
//interrupt void sciaRxFifoIsr(void) {
void sciaRxFifoIsr(void) {
    uint16_t lu16_rx_char;
    while (SciaRegs.SCIFFRX.bit.RXFFST > 0) { // Tant qu'il y a des données dans le FIFO RX
        lu16_rx_char = SciaRegs.SCIRXBUF.all;

    }

    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1; // Effacer le flag d'interruption RX FIFO
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Acquitter l'interruption au niveau du PIE
}

*/









