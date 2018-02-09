/*
 * gbflasher.h
 *
 *  	GBShooper firmware
 *      Author: David Pello http://ladecadence.net
 *      Licensed under a GNU GPL v3 License. See LICENSE.
 */

//#define F_CPU			3686400UL 	/* 3.6864 MHz */

#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>

/******************************************************************************/
/****************************** DEFINES ***************************************/
/******************************************************************************/

#define BUFFER_SIZE		256

#define SMSF_ID			0x17	/* 23 decimal */
#define VER_MAYOR		'0'
#define VER_MINOR		'1'

/* Bits */
#define BIT7			0x80
#define BIT6			0x40
#define BIT5			0x20

/* Status */
#define STAT_OK			0x14	/* 10.4 ;-) */
#define STAT_ERROR		0xEE

/* Paquetes */
#define PKT_COMMAND		0x11
#define PKT_DATA		0x22
#define PKT_STAT		0x33
#define PKT_INFO		0x44

/* Comandos */
#define CMD_ID			0x11
#define	CMD_READ_FLASH	0x22
#define CMD_READ_RAM	0x33
#define CMD_PRG_FLASH	0x44
#define CMD_PRG_RAM		0x55
#define CMD_ERASE_FLASH	0x66
#define CMD_ERASE_RAM	0x77
#define CMD_READ_HEADER	0x88
#define CMD_END			0xFF

/* Puertos */
#define ADDR_0_7		PORTB
#define ADDR_8_15		PORTA
#define	DATA_OUT		PORTC
#define DATA_IN			PINC
#define CONTROL			PORTD

/* Poner un byte en el bus de datos */
#define DATA_OUT_MODE	DDRC=0xFF
#define PUT_DATA(x)		DATA_OUT=x

/* Leer un byte del bus de datos */
#define DATA_IN_MODE	DDRC=0x00 ; PORTC=0xFF
#define GET_DATA		DATA_IN

/* Poner  la dirección en el bus de direcciones */
#define PUT_ADDR_L(l) 	ADDR_0_7 = l
#define PUT_ADDR_H(h)	ADDR_8_15 = h
#define PUT_ADDR(a)		PUT_ADDR_L((uint8_t)a); \
						PUT_ADDR_H((uint8_t)(a>>8))
/* Flash enable */
#define FLASH_CE		7
#define FLASH_ENABLE	PORTA &= ~(1<<FLASH_CE)
#define FLASH_DISABLE    PORTA |= (1<<FLASH_CE)


/* Lineas de control */
#define _RESET			6
#define _CE				5
#define _OE				4
#define _WE				3
#define RESET			PORTD &= ~(1<<_RESET)
#define NOT_RESET		PORTD |= (1<<_RESET)
#define CHIP_ENABLE		PORTD &= ~(1<<_CE)
#define CHIP_DISABLE	PORTD |= (1<<_CE)
#define OUTPUT_ENABLE	PORTD &= ~(1<<_OE)
#define	OUTPUT_DISABLE	PORTD |= (1<<_OE)
#define WRITE_ENABLE	PORTD &= ~(1<<_WE)
#define WRITE_DISABLE	PORTD |= (1<<_WE)

/* Modos de control */
#define READ_MODE		FLASH_ENABLE; CHIP_DISABLE; OUTPUT_ENABLE; WRITE_DISABLE
#define COMMAND_MODE	FLASH_DISABLE; CHIP_DISABLE; OUTPUT_DISABLE; WRITE_DISABLE

/* LED de status */
#define	STATUS			2
#define STATUS_ON		PORTD &= ~(1<<STATUS)
#define STATUS_OFF		PORTD |= (1<<STATUS)

/* valores para XTAL de 3.6864 MHz */
#define BAUDRATE_230400 0
#define BAUDRATE_115200 1
#define BAUDRATE_19200	11
#define BAUDRATE_9600	23

/* utilidades */
#define ZERO_TEMP		temp1=0; temp2=0	   /* pone a cero las temporales */
#define ZERO_PKT		pkt_type=0; pkt_data=0 /* ^-- etc... */

/******************************** UART ****************************************/
/* Inicia el uart */
void uart_init (unsigned int baud) ;

/* Recibe un byte del UART */
uint8_t uart_receive_byte(void) ;

/* Envia un byte al UART */
void uart_transmit_byte(uint8_t data) ;

/* Envia una cadena terminada en NULL al UART */
void uart_transmit_string(char* str) ;

/******************************** MBC *****************************************/

void mbc_set_rom_bank(uint16_t bank);
void mbc_set_ram_bank(uint8_t bank);
void mbc_enable_ram(void);
void mbc_disable_ram(void);

/******************************* FLASH ****************************************/

void flash_reset(void) ;
void flash_get_id(void) ;
uint8_t flash_erase(void) ;
uint8_t flash_program_byte(uint16_t addr, uint8_t data) ;
uint8_t flash_read_byte(uint16_t addr) ;

/******************************** RAM *****************************************/

uint8_t ram_write_byte(uint16_t addr, uint8_t data) ;
uint8_t ram_read_byte(uint16_t addr) ;

/****************************** SISTEMA ***************************************/

/* hace parpadear el led de status, indicando error */
void smsf_status_error(void) ;

/* Configura los puertos y los perifericos */
void smsf_init(void) ;

void smsf_send_packet(uint8_t type, uint8_t data) ;
void smsf_receive_packet(void) ;
void smsf_flash_info(void) ;
void smsf_header_read(void);
void smsf_system_status(void) ;
void smsf_flash_erase(void) ;
uint8_t smsf_flash_write(void) ;
uint8_t smsf_flash_read(void) ;
uint8_t smsf_ram_write(void) ;
uint8_t smsf_ram_read(void) ;
uint8_t smsf_ram_erase(void) ;

