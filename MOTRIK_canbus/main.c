/**
 * @file 	  main.c
 * @brief 	  Partie du projet Motrik qui sera utilisé pour la communication en utilisant le protocole CanBus.
 *
 * @author 	  Thomas Desrosiers
 * @version   1.0
 * @date 	  2022/02/28

 * @mainpage  MOTRIK_capt-acceleration
 * @author 	  Thomas Desrosiers
 * @section   MainSection1 Description
			  Partie du projet Motrik qui sera utilisé pour la communication en utilisant le protocole CanBus.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "usart.h"
#include "canbus.h"


/**********
 * DEFINE *
 **********/
#define _TIMER_1_TOP 250 - 1
#define _TIMER_1_CYC_CNT 50

#define PROCESS_BUFFER_SIZE 200

#define TRANSMIT //TRANSMIT / RECEIVE


/************
 * VARIABLE *
 ************/
volatile uint8_t cntCentMs = 0;
volatile uint8_t centMSFlag = 0;
volatile uint8_t cntDeuxS = 0;
volatile uint8_t deuxSFlag = 0;
volatile uint8_t refreshMesure = 0;
uint8_t incrementVal = 0;

static char app_buf[ PROCESS_BUFFER_SIZE ] = { 0 };
static int32_t app_buf_len = 0;
static int32_t app_buf_cnt = 0;
char demo_message[ 9 ] = { 'M', 'i', 'k', 'r', 'o', 'E', 13, 10, 0 };


/******************
 *      ENUM      *
 * STRUCT & UNION *
 ******************/
struct canFrame_s
{
	uint8_t can_id[4];  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t  can_dlc; //Length
	uint8_t  data[8];
	//uint16_t crc;
	//uint16_t endFrame;
	};
	struct canFrame_s canFrame;
	
//union canFrame_u
//{
	//uint32_t bytes[10];
	//struct canFrame_s data;
	//};
	//canFrame_u canFrame;


/**************************
 * PROTOTYPES DE FONCTION *
 **************************/
void application_init(void);

void application_task(void);

/**
 * @brief CAN Bus clearing application buffer.
 * @details This function clears memory of application buffer and reset it's length and counter.
 * @note None.
 */
static void canbus_clear_app_buf ( void );

/**
 * @brief CAN Bus data reading function.
 * @details This function reads data from device and concatenates data to application buffer.
 *
 * @return @li @c  0 - Read some data.
 *         @li @c -1 - Nothing is read.
 *         @li @c -2 - Application buffer overflow.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
static int8_t canbus_process ( void );

/**
*@brief  .
*/
void canBuilt(void);

/**
*@brief  Fonction d'initialisation des différents I/O et fonctions.
*/
void miscInit(void);

/**
*@brief  Fonction d'initialisation des différents I/O et fonctions.
*/
void canBuilt(void);

/**
 * @brief  Fonction d'initialisation du timer 0 avec une période de 4ms.
 */
void timer1Init();


/********
 * MAIN *
 ********/
int main(void)
{
	miscInit();
	while (1)
	{
		application_task( );
	}
}


/****************
 * INTERRUPTION *
 ****************/
/**
 *@brief  Le timer 1 est initialisé à 4ms. à chaques 4ms, refresh mesure est HAUT et après 100ms(50 x 4ms) centMSFlag est HAUT.
 */
ISR(TIMER1_COMPA_vect)
{
	centMSFlag++;
	if (centMSFlag >= _TIMER_1_CYC_CNT)
	{
		centMSFlag -= _TIMER_1_CYC_CNT;
		centMSFlag = 1; //À chaque 100ms. Ce flag sera utilisé pour changer les valeurs envoyés par les capteurs.
		cntDeuxS++;
		if (cntDeuxS >= 20)
		{
			cntDeuxS -= 20;
			deuxSFlag = 1;
		}
	}
}


/************************
 * DÉFINITION FONCTIONS *
 ************************/
void application_init(void) {
    //log_cfg_t log_cfg;        /**< Logger config object. */
    //canbus_cfg_t canbus_cfg;  /**< Click config object. */

    /** 
     * Logger initialization.
     * Default baud rate: 115200
     * Default log level: LOG_LEVEL_DEBUG
     * @note If USB_UART_RX and USB_UART_TX 
     * are defined as HAL_PIN_NC, you will 
     * need to define them manually for log to work. 
     * See @b LOG_MAP_USB_UART macro definition for detailed explanation.
     */
    //LOG_MAP_USB_UART( log_cfg );
    //log_init( &logger, &log_cfg );
    //log_info( &logger, " Application Init " );

    // Click initialization.

    canbus_cfg_setup();
    //CANBUS_MAP_MIKROBUS( canbus_cfg, MIKROBUS_1 );
    /* err_t init_flag  =  */canbus_init();
/*     if ( init_flag == UART_ERROR ) {
        log_error( &logger, " Application Init Error. " );
        log_info( &logger, " Please, run program again... " );

        for ( ; ; );
    } */

    canbus_default_cfg (1);
    app_buf_len = 0;
    app_buf_cnt = 0;
    //log_info( &logger, " Application Task " );
	centMSFlag = 0;
	centMSFlag = 0;
    while (!centMSFlag);

	centMSFlag = 0;

    canbus_set_high_speed_mode(1);
	centMSFlag = 0;
	centMSFlag = 0;
    while (!centMSFlag);
	centMSFlag = 0;
    
/*     #ifdef TRANSMIT
    
        log_printf( &logger, "    Send data:    \r\n" );
        log_printf( &logger, "      MikroE      \r\n" );
        log_printf( &logger, "------------------\r\n" );
        log_printf( &logger, "  Transmit data   \r\n" );
        Delay_ms( 1000 );

    #endif
        
    #ifdef RECIEVE

        log_printf( &logger, "   Receive data  \r\n" );
        Delay_ms( 2000 );
    
    #endif
        
    log_printf( &logger, "------------------\r\n" ); */
}

void application_task(void) {
   #ifdef TRANSMIT
    
        canbus_send_data(demo_message);
        //log_printf( &logger, "\t%s", demo_message );
		deuxSFlag = 0;
		while(!deuxSFlag);
		deuxSFlag = 0;
        //log_printf( &logger, "------------------\r\n" );    
    
    #endif
    
    #ifdef RECIEVE
    
        canbus_process( );

        if ( app_buf_len > 0 ) {
            //log_printf( &logger, "%s", app_buf );
            canbus_clear_app_buf(  );
        }
    
    #endif
}

/* void canBuilt(void)
{
	canFrame.can_id[0] = 0x0000;
	canFrame.can_id[1] = (0x0000 >> 8);
	canFrame.can_id[2] = (0x0000 >> 16);
	canFrame.can_id[3] = (0x0000 >> 24);
	canFrame.can_dlc = 4;
	canFrame.data[0] = 0xFF;
	canFrame.data[1] = 0xFF;
	canFrame.data[2] = 0xFF;
	canFrame.data[3] = 0xFF;
} */

static void canbus_clear_app_buf ( void ) {
    memset( app_buf, 0, app_buf_len );
    app_buf_len = 0;
    app_buf_cnt = 0;
}

static int8_t canbus_process(void) {
    int32_t rx_size;
    char rx_buff[ PROCESS_BUFFER_SIZE ] = { 0 };

    //rx_size = canbus_generic_read( &canbus, rx_buff, PROCESS_BUFFER_SIZE );

    if ( rx_size > 0 ) {
        int32_t buf_cnt = 0;

        if ( app_buf_len + rx_size >= PROCESS_BUFFER_SIZE ) {
            canbus_clear_app_buf( );
            return CANBUS_ERROR;
        } else {
            buf_cnt = app_buf_len;
            app_buf_len += rx_size;
        }

        for ( int32_t rx_cnt = 0; rx_cnt < rx_size; rx_cnt++ ) {
            if ( rx_buff[ rx_cnt ] != 0 ) {
                app_buf[ ( buf_cnt + rx_cnt ) ] = rx_buff[ rx_cnt ];
            } else {
                app_buf_len--;
                buf_cnt--;
            }

        }
        return CANBUS_OK;
    }
    return CANBUS_ERROR;
}

void miscInit(void)
{
	timer1Init(); // Initialisation du timers #1.
	application_init( );
}

void timer1Init()
{
	// TCCR1A : COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11 WGM10
	// TCCR1B: ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
	// TIMSK1: – – ICIE1 – OCIE1C OCIE1B OCIE1A TOIE1
	TCCR1B = (1 << WGM12);	 // mode CTC.
	TCCR1B |= (1 << CS12);	 // Prescaler de 256.
	TIMSK1 |= (1 << OCIE1A); // Output Compare A Match Interrupt Enable
	OCR1A = _TIMER_1_TOP - 1;		 // 62.5ns * 256 * 250 * 125 = 500ms
	sei();
}
