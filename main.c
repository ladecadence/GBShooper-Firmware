/*
 * main.c
 *	GBShooper firmware
 *      Author: David Pello http://ladecadence.net
 *	Licensed under a GNU GPL v3 License. See LICENSE.
 */


#define F_CPU			3686400UL 	/* 3.6864 MHz */
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include "gbflasher.h"

/******************************************************************************/
/***************************** VARIABLES **************************************/
/******************************************************************************/

uint8_t pkt_type, pkt_data = 0; // Bytes del protocolo
uint8_t temp1, temp2 = 0;		// Temporales para varias operaciones
uint32_t temp_addr;			    // Dirección temporal para operaciones
uint8_t buffer[BUFFER_SIZE];
uint32_t segundos = 0;		    // 4294967296 segundos, 136 años!
uint32_t ahora;		            // Guarda el segundo para timeouts

/******************************************************************************/
/***************************** FUNCIONES **************************************/
/******************************************************************************/

/******************************* TIMER ****************************************/
/* Interrupción */
ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	cli();
	segundos++;
	reti();
}


/* Inicia el timer */
void timer_init()
{
	/* Empezamos en 0 */
	TCNT1=0;
	/* Interrupción de overflow activada */
	TIMSK = (1<<TOIE1);
	/* Valor de reset */
	OCR1A = 3600; /* ~ 1s */
	/* Modo CTC, 3.6864 Mhz preescaler = /1024 =~ 3600 Hz */
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);	
	/* Activamos interrupciones */
	sei();	
}


/******************************** UART ****************************************/
/* Inicia el uart */
void uart_init (unsigned int baud) {
  /* Set baud rate */
  UBRRH = (uint8_t)(baud>>8);
  UBRRL = (uint8_t)baud;
  /* Enable receiver and transmitter */
  UCSRB = (1<<RXEN)|(1<<TXEN);
  /* Set frame format: 8data, 2stop bit */
  UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
}

/* Recibe un byte del UART */
uint8_t uart_receive_byte(void) {
  /* Wait for data to be received */
  while (!(UCSRA & (1<<RXC)));

  return UDR;
}

/* Envia un byte al UART */
void uart_transmit_byte(uint8_t data) {
  /* Wait for empty transmit buffer */
  while ( !( UCSRA & (1<<UDRE)));

  /* Copy ninth bit to TXB8 */
  //UCSRB &= ~(1<<TXB8);
  //if ( data & 0x0100 )
    //UCSRB |= (1<<TXB8);
  /* Put data into buffer, sends the data */
  UDR = data;
}

/* Envia una cadena terminada en NULL al UART */
void uart_transmit_string(char* str) {
	while (*str!=0) {
		uart_transmit_byte(*str);
		str++;
	}

}

/******************************** MBC *****************************************/

void mbc_set_rom_bank(uint16_t bank) {
	COMMAND_MODE;
	DATA_OUT_MODE;
	CHIP_DISABLE;

	PUT_ADDR(0x2000);
	PUT_DATA((uint8_t) bank);
	WRITE_ENABLE;
	WRITE_DISABLE;

	/* check if we need to update high bit of rom banking */
	if (bank>255) {
		PUT_ADDR(0x3000);
		PUT_DATA((uint8_t) bank>>8);
		WRITE_ENABLE;
		WRITE_DISABLE;
	}

}

void mbc_set_ram_bank(uint8_t bank) {
	COMMAND_MODE;
	DATA_OUT_MODE;
	CHIP_DISABLE;

	PUT_ADDR(0x4000);
	PUT_DATA(bank);
	WRITE_ENABLE;
	WRITE_DISABLE;
}

void mbc_enable_ram() {
	COMMAND_MODE;
	DATA_OUT_MODE;
	CHIP_DISABLE;

	PUT_ADDR(0x0000);
	PUT_DATA(0x0A);
	WRITE_ENABLE;
	WRITE_DISABLE;
}

void mbc_disable_ram() {
	COMMAND_MODE;
	DATA_OUT_MODE;
	CHIP_DISABLE;

	PUT_ADDR(0x0000);
	PUT_DATA(0x00);
	WRITE_ENABLE;
	WRITE_DISABLE;
}


/******************************* FLASH ****************************************/

void flash_reset(void) {
	COMMAND_MODE;
	DATA_OUT_MODE;

	// comando reset
	//PUT_ADDR(0x0000);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xF0);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

}

void flash_get_id(void) {
	// condiciones iniciales
	COMMAND_MODE;
	DATA_OUT_MODE;

	// modo desbloqueo, parte 1
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xAA);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// modo desbloqueo, parte 2
	PUT_ADDR(0x02AA);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x55);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// comando lectura de ID
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x90);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// Leemos Device ID
	DATA_IN_MODE;

	PUT_ADDR(0x0000);
	READ_MODE;

	// dos ciclos para que se estabilizen las señales
	asm volatile(	"nop" "\n\t"
					"nop" "\n\t");
	temp1 = GET_DATA;

	PUT_ADDR(0x0001);
	asm volatile(	"nop" "\n\t"
					"nop" "\n\t");
	temp2 = GET_DATA;

	flash_reset();

}

uint8_t flash_erase(void) {
	// condiciones iniciales
	COMMAND_MODE;
	DATA_OUT_MODE;

	// modo desbloqueo, parte 1
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xAA);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// modo desbloqueo, parte 2
	PUT_ADDR(0x02AA);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x55);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// comando borrado 1
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x80);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// modo desbloqueo, parte 3
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xAA);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// modo desbloqueo, parte 4
	PUT_ADDR(0x02AA);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x55);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// Comando de borrado 2
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x10);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	DATA_IN_MODE;
	PUT_ADDR(0x0000);
	READ_MODE;


	/* Checkeamos D7 para ver si ha terminado el borrado */
	while(!(GET_DATA&BIT7)) {
		//if (GET_DATA&BIT5)			/* Si D5 es 1, ERROR */
		//	return STAT_ERROR;
	}
	return STAT_OK;
}

uint8_t flash_program_byte(uint16_t addr, uint8_t data) {
	// condiciones iniciales
	COMMAND_MODE;
	DATA_OUT_MODE;

	CHIP_ENABLE;		/* down _CE so MBC doesn't mess with us */

	// modo desbloqueo, parte 1
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xAA);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// modo desbloqueo, parte 2
	PUT_ADDR(0x02AA);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0x55);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;

	// comando escritura
	PUT_ADDR(0x0555);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(0xA0);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	FLASH_DISABLE;
	

	// Dato a escribir
	PUT_ADDR(addr);
	FLASH_ENABLE;		/* la dirección entra al chip */
	PUT_DATA(data);
	WRITE_ENABLE;		/* los datos entran al chip */
	WRITE_DISABLE;

	CHIP_DISABLE;

	DATA_IN_MODE;
	//PUT_ADDR(addr);
	READ_MODE;

	/* Checkeamos para ver si ha terminado la programación */
	while((GET_DATA)!=(data)) {
		//if (GET_DATA&BIT5)			/* Si D5 es 1, ERROR */
		//	return STAT_ERROR;
	}
	return STAT_OK;

}

uint8_t flash_read_byte(uint16_t addr) {
	 READ_MODE;
	 DATA_IN_MODE;

	 FLASH_ENABLE;
	 PUT_ADDR(addr);

	 // dos ciclos para que se estabilizen las señales
	 asm volatile(	"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t");

	 temp1 = GET_DATA;

	 FLASH_DISABLE;
	 return STAT_OK;

 }


/******************************** RAM *****************************************/

uint8_t ram_read_byte(uint16_t addr) {
	 READ_MODE;
	 DATA_IN_MODE;

	 PUT_ADDR(addr); 
	 FLASH_DISABLE;

	 CHIP_ENABLE;

	 // dos ciclos para que se estabilizen las señales
	 asm volatile(	"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t");

	 temp1 = GET_DATA;

	 CHIP_DISABLE;
	 return STAT_OK;

}

uint8_t ram_write_byte(uint16_t addr, uint8_t data) {
	 READ_MODE;
	 DATA_OUT_MODE;


	 PUT_ADDR(addr); /* keep flash disabled */
	 FLASH_DISABLE;

	 CHIP_ENABLE;

	 // dos ciclos para que se estabilizen las señales
	 asm volatile(	"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t"
					"nop" "\n\t");

	 PUT_DATA(data);
	 WRITE_ENABLE;
	 WRITE_DISABLE;

	 CHIP_DISABLE;
	 return STAT_OK;

}
/****************************** SISTEMA ***************************************/

/* hace parpadear rápido el led de status, indicando error */
void smsf_status_error(void) {
	int i;
	for(i=0; i<20; i++) {
		STATUS_ON;
		_delay_ms(125);
		STATUS_OFF;
		_delay_ms(125);;
	}
}


/* hace parpadear lento el led de status, indicando exito */
void smsf_status_ok(void) {
	int i;
	for(i=0; i<3; i++) {
		STATUS_ON;
		_delay_ms(500);
		STATUS_OFF;
		_delay_ms(500);;
	}
}

/* Configura los puertos y los perifericos */
void smsf_init(void) {

	/* TIMER */
    timer_init();

	/* UART */
	uart_init(BAUDRATE_230400);
	//uart_transmit_string(MSG_VERSION);
	//uart_transmit_string(MSG_READY);

	/* Bus de direcciones, DDRB => A0-A7, DDRA => A8-A15
	 * Salidas
	 */
	DDRA = 0xFF;
	DDRB = 0xFF;

	/* 0000h */
	PUT_ADDR(0x0000);
	
	/* Bus de datos, DDRC => D0-D7
	 * Entrada
	 */
	DATA_IN_MODE;

	/* Estatus inicial de las lineas de control */
	DDRD |= 0xFC;
	FLASH_ENABLE;
	CHIP_DISABLE;
	OUTPUT_ENABLE;
	WRITE_DISABLE;

	/* Out from reset */
	NOT_RESET;

	/* LED  de estatus */
	DDRD |= (1<<STATUS);	// Bit 2 salida
	STATUS_ON;

	temp1 = 0;
	temp2 = 0;

}

/* envia un paquete por el uart */
void smsf_send_packet(uint8_t type, uint8_t data) {
	_delay_ms(10);
	uart_transmit_byte(type);

	uart_transmit_byte(data);
	//_delay_ms(1);
}
void smsf_receive_packet(void) {
	ZERO_PKT;
	pkt_type = uart_receive_byte();
	//_delay_ms(1);
	pkt_data = uart_receive_byte();
}

/* devuelve la información del chip flash */
void smsf_flash_info(void) {
	STATUS_ON;
	ZERO_TEMP;
	/* leemos el id del chip flash */
	flash_get_id();

	/* temp1 = manufacturer ID, temp2 = Chip ID */
	/* los mandamos */
	_delay_ms(1);
	smsf_send_packet(PKT_DATA, temp1);
	_delay_ms(1);
	smsf_send_packet(PKT_DATA, temp2);
	STATUS_OFF;
}

/* gets ROM name */
void smsf_rom_name(char* name) {
	uint8_t	title_size, i;

	ZERO_TEMP;

	/* check for CGB */
	flash_read_byte(0x0143);
	if (temp1 == 0x80 || temp1 == 0xC0)
		title_size = 11;
	else
		title_size = 16;

	/* get title */
	for (i=0; i<title_size; i++)
	{
		flash_read_byte(0x0134+i);
		name[i] = temp1;
	}

}


/* gets header information */
void smsf_header_read(void) {
	char title[17];
	int i;

	STATUS_ON;
	ZERO_TEMP;

	/* read cart type */
	flash_read_byte(0x0147);
	smsf_send_packet(PKT_DATA, temp1);
	_delay_ms(1);
	flash_read_byte(0x0148);
	smsf_send_packet(PKT_DATA, temp1);
	_delay_ms(1);
	flash_read_byte(0x0149);
	smsf_send_packet(PKT_DATA, temp1);
	_delay_ms(1);

	smsf_rom_name(title);
	for (i=0; i<16; i++){
		smsf_send_packet(PKT_DATA, title[i]);
		_delay_ms(1);
	}

	STATUS_OFF;
}

/* devuelve información del sistema */
void smsf_system_status(void) {
	STATUS_ON;
	_delay_ms(1);
	smsf_send_packet(PKT_DATA, SMSF_ID);
	_delay_ms(1);
	smsf_send_packet(PKT_DATA, VER_MAYOR);
	_delay_ms(1);
	smsf_send_packet(PKT_DATA, VER_MINOR);
	STATUS_OFF;
}

/* borra la memoria flash */
void smsf_flash_erase(void) {
	STATUS_ON;
	temp1 = flash_erase();
	if (temp1 == STAT_OK) {
		smsf_send_packet(PKT_STAT, STAT_OK);
		smsf_status_ok();
	}
	else {
		smsf_send_packet(PKT_STAT, STAT_ERROR);
		smsf_status_error();
	}
	STATUS_OFF;
}


uint8_t smsf_flash_write(void) {
	uint16_t i;
	uint8_t bank;
	uint8_t check;
	uint16_t bytecount;
	STATUS_ON;

	/* Empezamos a grabar en 0000h */
	temp_addr=0x0000;
	bank=0x00;
	bytecount = 0;

	/* bank 0 */
	mbc_set_rom_bank(bank);

	/* mandamos un ack, listos pa escribir */
	smsf_send_packet(PKT_STAT, STAT_OK);

	/* condición inicial */
	pkt_data = CMD_PRG_FLASH;
	while (pkt_data == CMD_PRG_FLASH) {
		check = 0;
		/* recibimos un bloque de bytes en el buffer */
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			temp1 = uart_receive_byte();
			buffer[i] = temp1;
		}

		/* programamos el bloque de bytes */
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			flash_program_byte(temp_addr, buffer[i]);
			temp_addr++;
			/* acumulamos el dato para el test */
			check+=buffer[i];
		}

		/* store flashed bytes */
		bytecount += BUFFER_SIZE;
		/* check if we need to change bank */
		if (bytecount>=16384) {
			bytecount = 0;
			temp_addr = 0x4000;
			bank++;
			mbc_set_rom_bank(bank);
		}

		/* enviamos la comprobación */
		smsf_send_packet(PKT_DATA, check);
		/* recibimos la respuesta, ¿seguimos grabando? */
		smsf_receive_packet();
	}

	STATUS_OFF;
	smsf_status_ok();
	return STAT_OK;

}

uint8_t smsf_flash_read(void) {
	uint16_t i;
	uint8_t check;
	uint8_t bank;
	uint16_t bytecount;
	
	STATUS_ON;

	/* Empezamos a leer en 0000h */
	temp_addr=0x0000;
	bank=0x00;
	bytecount = 0;

	/* bank 0 */
	mbc_set_rom_bank(bank);

	/* condición inicial */
	while (pkt_data == CMD_READ_FLASH) {
		check=0;
		/* leemos buffer bytes y calculamos la comprobacion */
		for (i=0; i<=255; i++) {
			flash_read_byte(temp_addr);
			buffer[i] = temp1;
			check+=temp1;
			temp_addr++;
		}

		/* los enviamos */
		for (i=0; i<=255;i++) {
			uart_transmit_byte(buffer[i]);
		}

		/* esperamos la comprobación */
		smsf_receive_packet();
		if (pkt_data != check) {
			smsf_send_packet(PKT_COMMAND, CMD_END);
			smsf_status_error();
			return STAT_ERROR;
		}

		/* store flashed bytes */
		bytecount += BUFFER_SIZE;
		/* check if we need to change bank */
		if (bytecount>=16384) {
			bytecount = 0;
			temp_addr = 0x4000;
			bank++;
			mbc_set_rom_bank(bank);
		}

		smsf_send_packet(PKT_STAT, STAT_OK);
		smsf_receive_packet();
	}

	smsf_status_ok();
	STATUS_OFF;
	return STAT_OK;
}

uint8_t smsf_ram_write(void){
	uint16_t i;
	uint8_t bank;
	uint8_t check;
	uint16_t bytecount;
	
	STATUS_ON;

	/* Empezamos a grabar en 0000h */
	temp_addr=0x0000;
	bank=0x00;
	bytecount = 0;

	/* enable ram, bank 0 */
	mbc_set_ram_bank(bank);
	mbc_enable_ram();

	/* mandamos un ack, listos pa escribir */
	smsf_send_packet(PKT_STAT, STAT_OK);

	/* condición inicial */
	pkt_data = CMD_PRG_RAM;
	while (pkt_data == CMD_PRG_RAM) {
		check = 0;
		/* recibimos un bloque de bytes en el buffer */
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			temp1 = uart_receive_byte();
			buffer[i] = temp1;
		}

		/* programamos el bloque de bytes */
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			ram_write_byte(temp_addr, buffer[i]);
			temp_addr++;
			/* acumulamos el dato para el test */
			check+=buffer[i];
		}

		/* store written bytes */
		bytecount += BUFFER_SIZE;
		/* check if we need to change bank */
		if (bytecount>=8192) {
			bytecount = 0;
			temp_addr = 0x0000;
			bank++;
			mbc_set_ram_bank(bank);
		}

		/* enviamos la comprobación */
		smsf_send_packet(PKT_DATA, check);
		/* recibimos la respuesta, ¿seguimos grabando? */
		smsf_receive_packet();
	}

	mbc_disable_ram();
	STATUS_OFF;
	smsf_status_ok();
	return STAT_OK;
}

uint8_t smsf_ram_read(void){
	uint16_t i;
	uint8_t bank;
	uint8_t check;
	uint16_t bytecount;

	STATUS_ON;

	/* Empezamos a leer en 0000h */
	bank=0x00;
	bytecount = 0;
	temp_addr=0x0000;

	/* enable ram, bank 0 */
	mbc_set_ram_bank(bank);
	mbc_enable_ram();

	/* condición inicial */
	while (pkt_data == CMD_READ_RAM) {
		check=0;
		/* leemos  bytes y calculamos la comprobacion */
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			ram_read_byte(temp_addr);
			buffer[i] = temp1;
			check+=temp1;
			temp_addr++;
		}

		/* los enviamos */
		for (i=0; i<=BUFFER_SIZE-1;i++) {
			uart_transmit_byte(buffer[i]);
		}

		/* esperamos la comprobación */
		smsf_receive_packet();
		if (pkt_data != check) {
			mbc_disable_ram();
			smsf_send_packet(PKT_COMMAND, CMD_END);
			smsf_status_error();
			return STAT_ERROR;
		}
		
		/* store read bytes */
		bytecount += BUFFER_SIZE;
		/* check if we need to change bank */
		if (bytecount>=8192) {
			bytecount = 0;
			temp_addr = 0x0000;
			bank++;
			mbc_set_ram_bank(bank);
		}

		smsf_send_packet(PKT_STAT, STAT_OK);
		smsf_receive_packet();
	}

	mbc_disable_ram();
	smsf_status_ok();
	STATUS_OFF;
	return STAT_OK;

}

uint8_t smsf_ram_erase(){
	uint16_t i;
	uint8_t bank;
	uint16_t bytecount;
	
	STATUS_ON;

	/* Empezamos a grabar en 0000h */
	temp_addr=0x0000;
	bank=0x00;
	bytecount = 0;

	/* enable ram, bank 0 */
	mbc_set_ram_bank(bank);
	mbc_enable_ram();

	/* mandamos un ack, listos pa escribir */
	smsf_send_packet(PKT_STAT, STAT_OK);

	/* condición inicial */
	pkt_data = CMD_ERASE_RAM;
	while (pkt_data == CMD_ERASE_RAM) {

		/* borramos el bloque de bytes con 0x00*/
		for (i=0; i<=BUFFER_SIZE-1; i++) {
			ram_write_byte(temp_addr, 0x00);
			temp_addr++;
		}

		/* store written bytes */
		bytecount += BUFFER_SIZE;
		/* check if we need to change bank */
		if (bytecount>=8192) {
			bytecount = 0;
			temp_addr = 0x0000;
			bank++;
			mbc_set_ram_bank(bank);
		}

		/* enviamos el ACK */
		smsf_send_packet(PKT_STAT, STAT_OK);
		/* recibimos la respuesta, ¿seguimos grabando? */
		smsf_receive_packet();
	}

	mbc_disable_ram();
	STATUS_OFF;
	smsf_status_ok();
	return STAT_OK;
}

/******************************************************************************/
/************************* PROGRAMA PRINCIPAL *********************************/
/******************************************************************************/
int main(void) {
	smsf_init();
	_delay_ms(100);
	STATUS_OFF;
	
	for(;;) {

		/* a la espera de un paquete */
		smsf_receive_packet();

		switch (pkt_type) {
			case PKT_COMMAND:
				switch (pkt_data) {
					case CMD_ID:
						smsf_flash_info();
						break;
					case CMD_ERASE_FLASH:
						smsf_flash_erase();
						break;
					case CMD_PRG_FLASH:
						smsf_flash_write();
						break;
					case CMD_READ_FLASH:
						smsf_flash_read();
						break;
					case CMD_READ_HEADER:
						smsf_header_read();
						break;
					case CMD_PRG_RAM:
						smsf_ram_write();
						break;
					case CMD_READ_RAM:
						smsf_ram_read();
						break;
					case CMD_ERASE_RAM:
						smsf_ram_erase();
						break;

				}
				break;
			case PKT_INFO:
				smsf_system_status();
				break;
			default:
				break;
		}
	}
}
