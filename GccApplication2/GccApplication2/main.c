/*
* GccApplication1.c
*
* Created: 31.10.2016 13:14:31
* Author : Tuomas
*/

#define F_CPU 3686400L
#define BAUD 9600
#define BRC (F_CPU/16/BAUD-1)
#define BUF_SIZE 64
#define PIN_GSMRST PINC5
#define PIN_OUT PINB5
#define PIN_LED PIND3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/sleep.h>

void init_AVR();
void send_data(const char* data);
void send_char(const char data);
char get_status(char* msg);
char get_ctrlz();
char get_response(char* response);
void clear_buffer();
void timer1_mode(const char mode);
char send_SMS();
char get_sms();
void avr_sleep();

//global variables
char* phone_num="+358505532822";
char set_output=0;

//volatiles for ISR
volatile char rx_buffer[BUF_SIZE];
volatile unsigned char data_index;
volatile unsigned char command_ready;
volatile unsigned int timer_count;

//setup commands
//AT+CMGF=1 :SMS text mode


//AT+IFC=1,1 software flow control
//send_char(19); XOFF
//send_char(17); XON

int main(void)
{
	init_AVR();
	
	send_char(27);

	do{
		send_data("AT+CREG?\r");
		_delay_ms(2000);
	} while(!get_response("0,1"));
	
	
	EIMSK &= ~(1 << INT0); // Disable INT0
	EIMSK |= (1 << INT0);  // Enable INT0

	send_data("AT+CMGF=1\r");
	get_response("OK");

	send_data("AT+CNMI=2,2,0,0,0\r");
	get_response("OK");

	send_data("AT+CSCLK=2\r");
	get_response("OK");

	send_data("AT+CNETLIGHT=0\r");
	get_response("OK");


	PORTC |= (1 << PIN_GSMRST); //disable gsm reset
	PORTD |= (1 << PIN_LED);

	/* MAIN LOOP */
	while (1)
	{
		wdt_reset();

		if (command_ready){
			command_ready=0;

			if (strstr((char*)rx_buffer,"RING")){
				PORTB |= (1 << PIN_OUT);
				timer1_mode(1);
				set_output=1;
			}

			if (strstr((char*)rx_buffer,"+CMT")){
				PORTB |= (1 << PIN_OUT);
			}
		}

		if (set_output==1){
			if (timer_count==2){
				send_data("ATH\r\n");
				if (get_response("OK")) timer_count++; //exit if condition
				EIMSK |= (1 << INT0); // Enable INT0
			}

			if (timer_count>12){
				PORTB &= ~(1 << PIN_OUT);
				timer1_mode(1);
				set_output=0;
			}
		}
		else{
			if (timer_count>4) avr_sleep();
		}
	}
}

void avr_sleep(){
	PORTD &= ~(1 << PIN_LED);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	cli();

	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();

	sei();

	timer1_mode(1);
	PORTD |= (1 << PIN_LED);
}

char get_sms(){

	char cmd_buffer[BUF_SIZE];
	strcpy(cmd_buffer,(const char*) rx_buffer);
	data_index=0;

	send_data(cmd_buffer);

	//char* ptr=strchr(cmd_buffer,'\n')+1; // find ',' from +CMTE: 0,26.17

	return 1;

	//+CMT:"+358505532822","","16/12/06,22:44:42+08"\nNumberi
}

void clear_buffer(char* buffer){
	memset((char*)&buffer[0], 0, sizeof(*buffer));
}

void send_char (char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) ); // Wait for empty transmit buffer
	UDR0 = data;  // Send data
}

void send_data(const char *data){
	while(*data) send_char(*data++);
}

ISR(USART_RX_vect){
	//wait for data
	while ( !(UCSR0A & (1<<RXC0)) );

	if (!(UCSR0A & ((1<<FE0) | (1<<DOR0) | (1<<UPE0)))) //check errors for frame, overrun, parity
	{

		if (data_index>BUF_SIZE-1) data_index=0;

		rx_buffer[data_index] = UDR0;

		if (rx_buffer[data_index]=='\r'){
			command_ready=1;
		}
		else{
			data_index++;
		}
	}
}

ISR(TIMER1_OVF_vect) {
	timer_count++;
}

ISR(INT0_vect) {
	PORTD |= (1 << PIND3);
	EIMSK &= ~(1 << INT0); // Disable INT0
}

char get_response(char* response){

	unsigned int timeout=0;

	do{
		_delay_ms(500);
		if (++timeout>8 || strstr((char*)rx_buffer,"ERROR")) return 0;
	}
	while(!strstr((char*)rx_buffer,response));

	data_index=0;

	return 1;
}


char send_SMS(){
	char status_msg[30];
	get_status(status_msg);

	char msg[30];
	strcpy(msg,"AT+CMGS=\"");
	strcat(msg, phone_num);
	strcat(msg, "\"");
	
	send_data(msg);
	send_data("\r\n");
	get_response(">");

	send_data(status_msg);

	send_char(27);

	return get_response("OK");
}


char get_status(char* msg){

	send_data("AT+CMTE?\r\n");
	char check=get_response("OK");

	if (check==0) return 0;

	char temp_str[3]="";
	char volt_str[3]="";
	char* ptr=strchr((const char*)rx_buffer,',')+1; // find ',' from +CMTE: 0,26.17

	strncpy(temp_str, ptr, 2); //get temp string

	//get ADC
	ADMUX=0x00; //PC0?

	ADCSRA |= (1<<ADSC);

	while(ADCSRA & (1<<ADSC));

	int voltage=ADCW; //12.5V => 490 adc value
	itoa(voltage,volt_str,10);

	char buf[30];
	strcpy(buf,"Lampotila: ");
	strcat(buf, temp_str);
	strcat(buf,"c \nAkun jannite: ");
	strcat(buf, volt_str);

	strcpy(msg,buf);

	return 1;
}



void init_AVR(){

	/* INIT WDT	*/
	/*
	//set up WDT interrupt
	WDTCSR = (1<<WDCE)|(1<<WDE);
	//Start watchdog timer with 4s prescaler
	WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3); //(1<<WDP0)
	*/
	MCUSR = 0; // clear reset flags
	wdt_disable();
	
	cli(); // disable global interrupts

	//wdt_enable(WDTO_4S);

	/* INIT PORTS */

	//DDRx: 1 output, 0 input

	//B: 5 OUTPUT
	DDRB = (1 << PIN_OUT);
	PORTB = 0x00;

	//C: 0 MEAS(in), 1 IN_SIG(in), 5 GSM_RST(out)
	DDRC = (1 << PIN_GSMRST);
	PORTC = 0x00;
	
	//D: 3 TEST_LED(out), 2 RING(in)
	//D3
	DDRD = (1 << PIN_LED);
	
	//D2 (ring interrupt)
	DDRD &= ~(1 << PIND2); //set to input
	PORTD |= (1 << PIND2); //set to 0: tristate
	
	EIMSK |= (1 << INT0);     // Enable INT0
	// trigger in low level, ISC0:00


	/* INIT ADC */

	ADMUX = (1<<REFS0); // AREF = AVcc

	// ADC Enable and prescaler of 128
	// 3.6MHz/32 = 112500Hz
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);


	/* INIT TIMER */

	TIMSK1 = (1 << TOIE1); // enable Timer1 overflow interrupt:
	timer1_mode(1);


	/* INIT USART */

	//Set baud rate
	UBRR0H = (BRC>>8);
	UBRR0L = BRC;

	//Enable transmitter and receiver and RX complete interrupt
	UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);

	//Set frame format: 8N1
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

	sei(); // enable global interrupts
}


//0: off & reset, 1: on
void timer1_mode(const char mode){
	timer_count=0;
	TCCR1A = 0;
	TCCR1B = 0; //stop timer
	TCNT1 = 0; //reset timer

	if (mode==1)
	{
		TCCR1B |= (1 << CS10) | (1 << CS11); //start timer
		//duration: 1/(f/prescaler)*2^timer_bits
	}
}


//UCSR0 register:
/*
UCSR0A:
7 RXC0: receive complete
6 TXC0: transmit complete
5 UDRE0: data register empty
*/

/*
UCSR0B:
7-3: enable interrupts RXCIE0/TXCIE0, enable TXEN0/RXEN0
2: UCSZ02: character size
*/

/*
UCSR0C:
NEED:
2: UCSZ01: character size
1: UCSZ00: character size

NO NEED:
7-6 UMSEL0n: opearation mode
5-4: UMP0n: parity mode
3: USBS0 stop bit select
0: UCPOL0: clock polarity
*/

//TIMER0 register:
/*
TCCR1B:
NEED:
0: CS10 clock bits
1: CS11
2: CS12

*/