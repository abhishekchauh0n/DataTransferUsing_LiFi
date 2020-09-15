#define F_CPU 2000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include<string.h> 

#define LCD_DATA PORTB // In my case PORTB is the PORT from which I send data to my LCD
#define Control_PORT PORTA // In my case PORTC is the PORT from which I set the RS , R/W and En
#define En PORTA7 // Enable signal
#define RW PORTA6 // Read/Write signal
#define RS PORTA5 // Register Select signal

void init_LCD();// Function to initialize the LCD
void LCD_cmd(unsigned char cmd);// Function to send command to th LCD
void LCD_write(unsigned char data);// Function to display character on LCD

unsigned int value;
float data;
unsigned char a=0,b=0,x,y,z;
int cor,n;




void usart_init(void)
{
	UCSRB |= (1 << RXEN) | (1 << RXCIE) | ( 1<<TXEN );// Turn on the transmission and reception 
	UCSRC = ( 1<<UCSZ1 ) | ( 1<<UCSZ0 ) | ( 1<<URSEL );
	UBRRL = 0X0C; //  FOR 2000000MHZ   9600 br
}


void usart_send(unsigned char data)
{

	while(! (UCSRA & (1<<UDRE)));
	UDR = data;
	_delay_ms(5);
}

void usart_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)		/* Send string till null */
	{
		_delay_ms(20);
		usart_send(str[j]);
		j++;
	}
}





void LCD_cmd(unsigned char cmd)
{
LCD_DATA=cmd;
Control_PORT =(0<<RS)|(0<<RW)|(1<<En);	// RS and RW as LOW and EN as HIGH
_delay_ms(10);
Control_PORT =(0<<RS)|(0<<RW)|(0<<En);	// RS, RW , LOW and EN as LOW
_delay_ms(10);
return;
}
 
void LCD_write(unsigned char data)
{
LCD_DATA= data;
Control_PORT = (1<<RS)|(0<<RW)|(1<<En);	// RW as LOW and RS, EN as HIGH
_delay_ms(10);
Control_PORT = (1<<RS)|(0<<RW)|(0<<En);	// EN and RW as LOW and RS HIGH
_delay_ms(10);	// delay to get things executed
return ;
}
 
void init_LCD()
{

LCD_cmd(0X38);
_delay_ms(10);
LCD_cmd(0X01);
_delay_ms(10);
LCD_cmd(0X0C);
_delay_ms(10);
LCD_cmd(0X80);
_delay_ms(10);

}
 
void LCD_write_string(unsigned char *str) //take address vaue of the string in pionter *str
{
int i=0;
while(str[i]!='\0') // loop will go on till the NULL charaters is soon in string 
{
LCD_write(str[i]); // sending data on CD byte by byte
i++;
}
return;
}
 



int main(void)
{
	
DDRA=0xf0;
DDRB=0xff;
DDRC=0X00;	
DDRD=0xff;

PORTD |= (1<<PD7);  /// set high

UCSRB = ( 1<<TXEN ) | ( 1<<RXEN ) ;
UCSRC = ( 1<<UCSZ1 ) | ( 1<<UCSZ0 ) | ( 1<<URSEL );
UBRRL = 0X0C; //9600 br

_delay_ms(2000);

LCD_cmd(0X38);
_delay_ms(10);
LCD_cmd(0X01);
_delay_ms(10);
LCD_cmd(0X0C);
_delay_ms(10);
LCD_cmd(0X80);
_delay_ms(10);


LCD_write_string("LiFi Transmitter");
_delay_ms(2000);

LCD_cmd(0X38);
_delay_ms(10);
LCD_cmd(0X01);
_delay_ms(10);
LCD_cmd(0X0C);
_delay_ms(10);
LCD_cmd(0X80);
_delay_ms(10);

while(1)
{

PORTD |= (1<<PD7);  /// set high

while(! (UCSRA & (1<<RXC)));
cor = UDR ;
_delay_ms(1000);

PORTD &= ~(1<<PD7);  // set low
_delay_ms(5);
PORTD |= (1<<PD7);  /// set high
_delay_ms(5);
PORTD &= ~(1<<PD7);  // set low
_delay_ms(5);
PORTD |= (1<<PD7);  /// set high
_delay_ms(5);
PORTD &= ~(1<<PD7);  // set low
_delay_ms(5);
PORTD |= (1<<PD7);  /// set high
_delay_ms(5);
PORTD &= ~(1<<PD7);  // set low
_delay_ms(5);
PORTD |= (1<<PD7);  /// set high
_delay_ms(5);



usart_send(cor);
_delay_ms(10);

LCD_write(cor);
_delay_ms(1000);


if(cor == '#' )

{
LCD_cmd(0X38);
_delay_ms(10);
LCD_cmd(0X01);
_delay_ms(10);
LCD_cmd(0X0C);
_delay_ms(10);
LCD_cmd(0X80);
_delay_ms(10);


LCD_write_string("Enter New message");
_delay_ms(2000);

LCD_cmd(0X38);
_delay_ms(10);
LCD_cmd(0X01);
_delay_ms(10);
LCD_cmd(0X0C);
_delay_ms(10);
LCD_cmd(0X80);
_delay_ms(10);
}

else

{

}






}

return 0;
}
