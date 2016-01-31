/****************************************************************************************************
	LED pin configuration 
		|  |  |   |
		|  |  |   |
		|  |  |   |
		   |  |
		      |
	    B  G  +   R
	  27  18  28  17

ADC Connection:
ACD CH.	   PORT	   Sensor
0			PF0		Battery Voltage
1			PF1		White line sensor 3 right
2			PF2		White line sensor 2 center
3			PF3		White line sensor 1 left
9			PK1		Sharp IR range sensor 1  left 
11			PK3		Sharp IR range sensor 3  front
13			PK5		Sharp IR range sensor 5  right


LCD Display interpretation:
****************************************************************************
 BATTERY VOLTAGE		LEFT SHARP DIS   FRONT SHARP DIS    RIGHT SHARP DIS
 color detected			LEFT WL SENSOR	 CENTER WL SENSOR	RIGHT WL SENSOR		
****************************************************************************

room numbering															  _________
																		 |		   |
 ________________________________________________________________________|		   |
|__________________________________   ___________________________________		   |
								   | |									 |		   |
blue		     red    		   | |			green					 |_________|
								   | |
								   | |
								   | |
								   |_|		 ___________________________
____________________________				|							|
			_____			|				|							|
		   |_   _|			|	  			|	 __		room1	  __	|	
			 | |							|	|  |_____________|  |   |
			 | |____________				|	|  	_____	_____	|	|
	room4	 |	____________|				|	|__|	 | |	 |__|	|
			 | |							|			 | |			|		
			_| |_    						|			 | |			|	
		   |_____|			|				|			 | |			|
____________________________|		 _		|_____       |_|	   _____|
									| |		  12.5		  40		12.5|			
									| |									|			
							   _____| |_____							|			
							  |_____ H _____|							|											 
									| |									|
									| |									|
									|_|									|
_____         _        _____				 ___________________________|
			 | |			|				|		  room 2			|
			 | |			|				|		  _____				|
			 | |			|				 		 |_   _|			|
	__		 | |	  __	|				 		   | |				|
   |  |______| |_____|	|	|				 __________| |				|
   |   ______________	|	|				|__________	 |		    	|
   |__|				 |__|	|						   | |				|
							|						  _| |_				|
		  room 3			|				|		 |_____|			|
							|				|							|										 
****************************************************************************************************/
/*************incomplete tasks************************
pickup_service(); //this function will be different for vip,rooms without garbage can  && regular rooms with garbage 
				  //for the vip room and picking the service for next room would ve different paths without garbage 



*******************************************************/
//#define __OPTIMIZE__ -O0
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

int line_sensor_distance=200;
int threshold_line_sensor_value=70;
int Csensor_pos=50;   //distance of color sensor from the left sharp sensor
int max_speed=150,turn_speed=100;



volatile unsigned long int ShaftCountLeft = 0; 		
volatile unsigned long int ShaftCountRight = 0; 	
volatile unsigned int Degrees;					 	
long int pulse = 0; 				
long int red;       				
long int blue;      				
long int green;     				
long int threshold=900;			//threshold value to decide the color
char color;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char distance, adc_reading;
float BATT_Voltage;
int sharp_left=0,sharp_right=0,sharp_front=0,count,sharp_left_diff,sharp_right_diff,sharp_front_diff;

int left_line=0,center_line=0,right_line=0,line_conf;
int pref[5]={0,0,0,0,0};
char orders[5];
int sorted_rooms[5]={0,0,0,0,0};
int current_room;


//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; 
	PORTC = PORTC & 0x80; 
}


void lcd_cursor_char_print(char row,char column,char letter)
{       
	lcd_cursor (row,column);
	lcd_wr_char(letter);
}
	
int mod(int a)
{   
	if (a<0)
	a= a*(-1);
	return a;
}


void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}



void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row and Column Location
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{	int sharp_error;
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	if (distance<=140)
	sharp_error = 15;
	else if (distance<=250)
	sharp_error = 20;
	else if (distance<=360)
	sharp_error = 35;
	else if (distance<=525)
	sharp_error = 45;
	else if (distance<=800)
	sharp_error = 50;
	else
	sharp_error=0;
	distanceInt = (int)distance - sharp_error;
	if(distanceInt>800)
		distanceInt=800;
	return distanceInt;
}


void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void GPIO_pin_config(void)
{
	DDRL = DDRL | 0xC3;      
	PORTL = PORTL | 0xC3;	 
}

void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}

void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

//ISR for color sensor
ISR(INT0_vect)
{
	pulse++;
}

//Color Sensing Scaling
void color_sensor_scaling()		//This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
	//Output Scaling 20% from datasheet
	//PORTD = PORTD & 0xEF;
	PORTD = PORTD | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTD = PORTD | 0x20; //set S1 high
}

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD & 0x7F; //set S3 low
	pulse=0; 
	_delay_ms(100); 
	red = pulse;  
	
	}

void green_read(void) 
{
	PORTD = PORTD | 0x40; //set S2 High
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	green = pulse;  
	
	}

void blue_read(void) 
{
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	blue = pulse;  
	}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   
	PORTL = PORTL | 0x18; 
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;
}

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
	motion_set(0x06);
}

void back (void) 
{
	motion_set(0x09);
}

void left (void) 
{
	motion_set(0x05);
}

void right (void) 
{
	motion_set(0x0A);
}


void stop (void)
{
	motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); 
	angle_rotate(Degrees);
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  
 PORTB = PORTB | 0x20; 
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  
 PORTB = PORTB | 0x40; 
}



 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}



void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void velocity (unsigned char left_motor, unsigned char right_motor)   
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}
 
void clip_close(void)
{	for(int i=0;i<270;i++)
	{
		servo_1(i);
		_delay_ms(0);
		servo_2(270-i);
		_delay_ms(0);
	}
}
void clip_open(void)
{	for (int i=270;i>0;i--)
	{	
		servo_1(i);
		_delay_ms(0);
		servo_2(270-i);
		_delay_ms(0);
	}
}

void blink_red()
{
	 PORTL = PORTL & 0x7F;
	 _delay_ms(1000);   
	 PORTL = PORTL | 0xC3;
}
 
 void blink_green()
 {
	 PORTL = PORTL & 0xBF;
	 _delay_ms(1000);
	 PORTL = PORTL | 0xC3;
 }
 
 void  blink_blue()
 {
	 PORTL = PORTL & 0xFD;
	 _delay_ms(1000);
	 PORTL = PORTL | 0xC3;
 }
 
 void judge_color(long int red,long int green,long int blue)
 {
	 if(red<threshold && green<threshold && blue<threshold)
		 color = 'K';
	 else
	 {
		 if (red>green && red >blue)
		 {
			 color = 'R';
			 blink_red();
		 }
		 else if (green>red && green > blue)
		 {
			 color = 'G';
			 blink_green();
		 }
		 else if (blue>red && blue>green)
		 {
			 color = 'B';
			 blink_blue();
		 }
		 else
		 {
			 color = 'E';
			 color_detect();
		 }
	 }
 }
	
	  
 
 void color_detect()
 {
	
	red_read(); 
	_delay_ms(500);
	red_read();
	lcd_print(1,1,red,5);
	_delay_ms(1000);
	
	green_read(); 
	_delay_ms(500);
	green_read();
	lcd_print(1,7,green,5);
	_delay_ms(1000);
	
	blue_read(); 
	_delay_ms(500);
	blue_read();
	lcd_print(2,1,blue,5);
	_delay_ms(1000);
	
	judge_color(red,green,blue);
 }
 
 char judge_order(char room1,char room2)
 {
	 char order1;
	 if (room1 != 'K')
	 {
		 if (room1 == room2)
		 {
			 order1 = room1;
			 pref[current_room] = 1;
		 }
		 else if (room2 == 'K')
		 {
			 order1 = room1;
			 pref[current_room]=0;
		 }
		 else
		 order1 = 'E';  //E as error we need to detect the color again
	 }
	 
	 else
	 {
		 if (room2 == 'K')
		 {
			 order1 = 'N';    //N for Do Not Disturb Room
			 pref[current_room] = (-1);
		 }
		 else
		 {
			 order1 = room2;
			 pref[current_room] = 0;
		 }
	 }
	 
	 return order1;
 }
 
 void sort_orders()
 {  int j=2;
	 for(int i=1;i<5;i++)
	 {
		 if(pref[i]==1)
		 sorted_rooms[1] =i;
		 else if (pref[i]==0)
		 {
			 sorted_rooms[j]=i;
			 j++;
		 }
	 }
	 
 }
 
 
 
 void print_line_sensor()
 {
	 if (ADC_Conversion(3)>threshold_line_sensor_value)    //to print left line sensor detection W-white B-black
	 {	lcd_cursor_char_print(2,5,'B');
		left_line=1;
	 }
	 else
	 {	lcd_cursor_char_print(2,5,'W');
		left_line=0;
	 }
	 
	 if (ADC_Conversion(2)>threshold_line_sensor_value)	  //to print center line sensor detection W-white B-black
	 {	lcd_cursor_char_print(2,7,'B');
		center_line=1;
	 }
	 else
	 {	lcd_cursor_char_print(2,7,'W');
		center_line=0;
	 }
	 
	 if (ADC_Conversion(1)>threshold_line_sensor_value)	  //to print right line sensor detection
	 {	lcd_cursor_char_print(2,9,'B');
		right_line=1;
	 }
	 else
	 {	lcd_cursor_char_print(2,9,'W');
		right_line=0;
	 }
	 
	 line_conf = 100*left_line + 10*center_line +right_line;
 }
 
 /*void sharp_testing(){
	 int value,sharp;
    value = ADC_Conversion(11);
    lcd_print(1,6,value,4);
	sharp=Sharp_GP2D12_estimation(value);
    lcd_print(1,11,sharp,3);
 }*/
 
 
 void print_sharp_sensor()

 {  int sharp,value;
	 
	 value = ADC_Conversion(9);
	 sharp =Sharp_GP2D12_estimation(value);
	 sharp_left_diff = sharp_left - sharp;
	 sharp_left=sharp;
	 lcd_print(1,6,sharp,3);
	 value = ADC_Conversion(11);
	 sharp=Sharp_GP2D12_estimation(value);
	 sharp_front_diff = sharp_front - sharp;
	 sharp_front=sharp;
	 lcd_print(1,10,sharp,3);
	 value = ADC_Conversion(13);
	 sharp=Sharp_GP2D12_estimation(value);
	 sharp_right_diff = sharp_right - sharp;
	 sharp_right=sharp;
	 lcd_print(1,14,sharp,3);
 }
 
 
 
 void auto_follow()
 {
	 int P,I,D,Kp=1,Ki=0,Kd=0,correction,L_speed,R_speed,error,prev_error,speed=0,max_speed=160,turn_speed=120,line_conf;
	 if (count==0)
	 {
		 P=0;
		 I=0;
		 D=0;
		 correction = 0;
		 error=0;
		 prev_error=0;
		 count++;
	 }
	 print_line_sensor();
	 print_sharp_sensor();
	 line_conf = 100*left_line + 10*center_line +right_line;
	 if (line_conf == 111)
	 stop();
	 else if (line_conf == 101)
	 {
		forward();
		velocity(turn_speed,turn_speed);
	 }
	 else if (line_conf == 000)
	 {
		 L_speed = turn_speed + 20 *error;
		 R_speed = turn_speed - 20 *error;
		 forward();
	 velocity(L_speed,R_speed);
	 }
	 
	 else 
	 {
		 if (line_conf == 100)
		 {
			error = -2;
			speed = turn_speed;
		 }
		 else if(line_conf == 110)
		 {
			error = -1;
			speed = max_speed - 20;
		 }
		 else if(line_conf == 010)
		 {
			error = 0;
			speed = max_speed;
		 }
		 else if(line_conf == 011)
		 {
			error = 1;
			speed = max_speed -20;
		 }
		 else if(line_conf == 110)
		 {
			error = 2;
			speed = turn_speed;
		 }
		 
		 P = error;
		 I = I + error;
		 D = prev_error - error;
		 correction = Kp*P + Ki*I + Kd*D;
		 L_speed = speed + correction;
		 R_speed = speed - correction;
		 prev_error = error;
		 forward();
		 velocity(L_speed,R_speed);
	 }
 }



void print_battery_voltage()
{
	BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;
	lcd_print(1,1,BATT_Voltage,4);
}


void take_order(){
	
	char room1,room2;
	print_sharp_sensor();
	while(sharp_left_diff < 300)
	{
		forward();
		velocity(turn_speed,turn_speed);
		print_sharp_sensor();
	}
	stop();
	left_degrees(90);
	forward_mm(185);
	stop();
	right_degrees(90);
	color_detect();
	while (color=='E')
	{
		color_detect();
	}
	room1 = color;
	//forward_mm(150);
	print_sharp_sensor();
	while(sharp_front >= 150)
	{
		forward();
	    velocity(turn_speed,turn_speed);
		print_sharp_sensor();
	}
	stop();
	
	_delay_ms(300);
	forward_mm(60);
	color_detect();
	while (color=='E')
	{
		color_detect();
	}
	room2=color;
	
	while (judge_order(room1,room2)=='E')		//detecting the indicators again if we cant judge the final order from available color pair like red and green 
	{
		 back_mm(425);			//going back in front of the previous indicator
		 
		 color_detect();
		 
		 room1 = color;
		 //forward_mm(150);
		 
		 while(sharp_front >= 150)
		 {
			 print_sharp_sensor();
			 forward();
			 velocity(turn_speed,turn_speed);
		 }
		 stop();
		 
		 _delay_ms(300);
		 forward_mm(70);
		 color_detect();
		 room2=color;
	}
	orders[current_room]=judge_order(room1,room2);
	
	if (current_room!=4)
	{
	
		right_degrees(180);
		print_line_sensor();
		while(line_conf != 111)
		{
			forward();
			velocity(max_speed,max_speed);
			print_line_sensor();
		}
		stop();
	
		_delay_ms(300);
		left_degrees(90);
		print_line_sensor();
	
		while(line_conf != 111)
		{
		auto_follow ();
		}
		count=0;
	
		stop();
		_delay_ms(100);
		forward_mm(200);
		stop();
		_delay_ms(100);
		print_line_sensor();
	
		while(line_conf != 010)
		{
			left_degrees(1);
			print_line_sensor();
		}
		stop();
	}  
	else if (current_room==4)
	{	
		forward_mm(200);
		right_degrees(90);
		print_line_sensor();
		while(line_conf != 111)
		{
			forward();
			velocity(turn_speed,turn_speed);
		}
		stop();
		forward_mm(line_sensor_distance-5);
		left_degrees(90);
		while(line_conf != 111)
		{
			auto_follow();
		}
		count = 0;
		stop();
	}
	
}

 
 //Initialize the ports
 void port_init(void)
 {
	 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	 motion_pin_config(); //robot motion pins config
	 left_encoder_pin_config(); 
	 right_encoder_pin_config(); 
	 color_sensor_pin_config();
	 GPIO_pin_config();	//GPIO pins config for LEDs to glow 
	 buzzer_pin_config(); 
	 lcd_port_config();		
	 adc_pin_config();
 }
 
 //Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	timer1_init();   //PWM for servo pins
	timer5_init();	 //PWM for velocity of bot or DC motors
	color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	lcd_set_4bit();
	lcd_init();
	color_sensor_scaling();
	sei();   // Enables the global interrupt
}
 
 
 

 int main(void)
 {  
	 init_devices();
	 
   /*for (current_room=1;current_room<=4;current_room++)
	 {
		 take_order();
	 }
	 sort_orders();			//this will save the sequence of the rooms according to their preferences in a array rooms_sorted[] from 1 to 4 otherwise zero
	 
	 */ 
	 //print_battery_voltage();
	 while(1)
	 {		print_line_sensor();
		while (line_conf!=010)
		{
			print_line_sensor();
			left_degrees(2);
		}
			stop();
			_delay_ms(1000);
			right_degrees(90);
		/*clip_close();
		lcd_wr_char('C');
		_delay_ms(2000);
		clip_open();
		lcd_wr_char('O');
		_delay_ms(2000);*/
	
		//color_detect();
		//lcd_cursor_char_print(1,10,color_detect());
		//_delay_ms(1000);
		//sharp_testing();
		//auto_follow();
		//follow_line();
		//print_line_sensor();
		//print_sharp_sensor();
	 }
 }
