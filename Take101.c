/****************************************************************************************************
LED pin configuration
|	|	|   |
|	|	|   |
|	|	|   |
	|	|
		|
B	G	 +   R
27  18  28  17
ADC Connection:
ADC CH.	   PORT	   Sensor
0			PF0		Battery Voltage
1			PF1		White line sensor 3 right
2			PF2		White line sensor 2 center
3			PF3		White line sensor 1 left
4			PF4		IR Proximity analog sensor 1
5			PF5		IR Proximity analog sensor 2
6			PF6		IR Proximity analog sensor 3
7			PF7		IR Proximity analog sensor 4
8			PK0		IR Proximity analog sensor 5
9			PK1		Sharp IR range sensor 1  left
11			PK3		Sharp IR range sensor 3  front
13			PK5		Sharp IR range sensor 5  right
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading) {	
	int sharp_error;
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
unsigned int side_Sharp_GP2D12_estimation(unsigned char adc_reading) {	
	int sharp_error;
	float distance;
	unsigned int distanceInt;
	distance = (int) 4795.2296*(pow((float)adc_reading,-0.925180938));
		if (distance<=60)
		sharp_error = 5;
		else if (distance<=212 && distance>205)
		sharp_error = -4;
		else if (distance<280 && distance>230)
		sharp_error = -6;
		else if (distance>314 && distance<=333)
		sharp_error = 10;
		else if (distance<430 && distance >370)
		sharp_error = 15;
		else if (distance<500 && distance >470)
		sharp_error= -15;
		else if (distance<700 && distance >500)
		sharp_error=35;
		else if(distance <800 && distance>700)
		sharp_error = 80;
		else if (distance <920 && distance > 800)
		sharp_error=135;
		else
		sharp_error=0;
	
	distanceInt = (int)distance - sharp_error;
	if(distanceInt>800)
	distanceInt=800;
	return distanceInt;
}
int main()
{
	init_devices();
	PORTG = PORTG | 0x04;		//to turn off sharp sensor234 and white line sensor leds
	while (1)
	{
		PORTH = PORTH | 0x04;		//off sharp 1 and 5
		PORTH = PORTH | 0x08;		//turn off proximity sensor leds
		lcd_cursor_char_print(1,1,'O');
		lcd_cursor_char_print(1,2,'N');
		lcd_cursor_char_print(1,3,' ');
		print_sensor(2,7,13);
		print_sensor(2,1,9);
		print_sensor(1,4,4);	//left proximity
		print_sensor(1,8,6);	//front proximity
		print_sensor(1,12,8);	//right proximity
		_delay_ms(1000);
		PORTH = PORTH & 0xFB;		//on sharp 1 and 5
		PORTH = PORTH & 0xF7;		//turn on proximity sensor leds
		lcd_cursor_char_print(1,1,'O');
		lcd_cursor_char_print(1,2,'F');
		lcd_cursor_char_print(1,3,'F');
		print_sensor(2,7,13);
		print_sensor(2,1,9);
		print_sensor(1,5,4);	
		print_sensor(1,9,6);
		print_sensor(1,13,8);
		_delay_ms(1000);
	}
}
void MOSFET_switch_config (void)
	{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}
void lcd_port_config (void){
	DDRC = DDRC | 0xF7; 
	PORTC = PORTC & 0x80; 
}
void lcd_cursor_char_print(char row,char column,char letter){       
	lcd_cursor (row,column);
	lcd_wr_char(letter);
}

void print_battery_voltage(){
	BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;
	lcd_print(1,1,BATT_Voltage,4);
}
void print_sensor(char row, char coloumn,unsigned char channel) {
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
****************************************************************************************************/
/*
*
* Team Id: 			eYRC-HS#3528
* Author List: 		Kiran Dhamane,Ayush Sawarni,Rishabh Joshi,Siddharth
* Filename: 		Take101
* Theme: 			Hotel Guest Service
* Functions: 		motion_set(),forward(),back(),left(),right(),stop(),linear_distance_mm(int),forward_mm(int),back_mm(int),
*						angle_rotate(int),left_degrees(int),right_degrees(int),servo1_pin_config(),servo2_pin_config(),servo3_pin_config(),
*						servo_1(int),servo_2(int),servo_3(int),servo_1_free(),servo_2_free(),servo_3_free(),color_sensor_pin_interrupt_init(),
*						left_encoder_pin_config(),right_encoder_pin_config(),left_position_encoder_interrupt_init(),
*						right_position_encoder_interrupt_init(),timer1_init(),timer5_init(),velocity(int,int),print_line_sensor(),take_order1(),
*						enter_room(),dump_garbage(),return_home(),buzzer_pin_config(),buzzer_on(),buzzer_off(),GPIO_pin_config(),
*						find_line(),slow_follow_line(int),follow_line(int),turn_on_line(char),adc_pin_config(),adc_init(),ADC_Conversion(int),
*						color_sensor_pin_config(),color_sensor_scaling(),motion_pin_config(),clip_close(),clip_open(),color_detect(),
*						judge_order(char,char),sort_orders(),pickup_service_dumping_section(char),pickup_service_Shome(char),delivery(),
*						init_devices(),main().
* Global Variables:	threshold,ShaftCountLeft,ShaftCountRight,pulse,red,blue,green,ADC_Value,adc_reading,line_conf,shaft,garbage_rank,
*						color,pref[5],orders[5],sorted_rooms[6],current_room,position,count1
*
*/

//every function must have this comment before its declaration
/*
*
* Function Name: 	<Function Name>
* Input: 		<Inputs (or Parameters) list with description if any>
* Output: 		<Return value with description if any>
* Logic: 		<Description of the function performed and the logic used in the function>
* Example Call:		<Example of how to call this function>
*
*/



#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

const int threshold=900;	//threshold value to decide the color 550 in the night and 800 in the daylight

volatile unsigned long int ShaftCountLeft = 0;
volatile unsigned long int ShaftCountRight = 0;
//volatile unsigned int value;
volatile unsigned long int pulse = 0;
volatile unsigned long int  red;
volatile unsigned long int  blue;
volatile unsigned long int  green;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
volatile unsigned char adc_reading;

int line_conf=0;
int shaft=0;					//Every time we use follow_line(0) shaft is an approx shaft count after which line ends
int garbage_rank=0;				//the rank of present garbage in dumping section
char color;
int pref[5]={-1,-1,-1,-1,-1};			//the type of room is saved sequentially 1->vip	0->regular	(-1)->DND room
char orders[5];					//the orders of the rooms 1234 in sequence 2nd position has order room1's order
int sorted_rooms[6]={0,0,0,0,0,0};		//the final sequence of rooms bot has to provide service
int current_room=1;
char position='S';
int count1=1;


/*siddharth*/void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
/*siddharth*/void forward (void) 
{
	motion_set(0x06);
}
/*siddharth*/void back (void)
{
	motion_set(0x09);
}
/*siddharth*/void left (void) 
{
	motion_set(0x05);
}
/*siddharth*/void right (void)
{
	motion_set(0x0A);
}
/*siddharth*/void stop (void)
{
	motion_set(0x00);
}
/* siddharth*/void linear_distance_mm(unsigned int DistanceInMM)
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
/*siddharth*/void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}
/*siddharth*/void back_mm(unsigned int DistanceInMM) 
{
	back();
	linear_distance_mm(DistanceInMM);
}
/*siddharth*/void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/3.60351 ; //was 4.090 division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); /*Stop robot*/}
/*siddharth*/void left_degrees(unsigned int Degrees) 
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	
	angle_rotate(Degrees);
	
}
/*siddharth*/void right_degrees(unsigned int Degrees) 
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right();
	angle_rotate(Degrees);
}
/*siddharth*/void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  
 PORTB = PORTB | 0x20; 
}
/*siddharth*/void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  
 PORTB = PORTB | 0x40; 
}
/*siddharth*/void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
/*siddharth*/void servo_1(unsigned char degrees) 
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}
/*siddharth*/void servo_2(unsigned char degrees) 
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}
/*siddharth*/void servo_3(unsigned char degrees) 
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}
/*siddharth*/void servo_1_free (void) 
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}
/*siddharth*/void servo_2_free (void) 
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}
/*siddharth*/void servo_3_free (void) 
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}





/*kiran*/void color_sensor_pin_interrupt_init(void)
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}
/*kiran*/ISR(INT0_vect) 
{
	pulse++;
}
/*kiran*/void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}
/*kiran*/void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
/*kiran*/void left_position_encoder_interrupt_init (void)
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt}
/*kiran*/void right_position_encoder_interrupt_init (void)
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}
/*kiran*/ISR(INT5_vect) 
{
	ShaftCountRight++;
}
/*kiran*/ISR(INT4_vect)
{
	ShaftCountLeft++;
} 
/*kiran*/void timer1_init(void) 
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0X03;
 OCR1CL = 0XFF;
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
/*kiran*/void timer5_init()
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
/*kiran*/void velocity (unsigned char left_motor, unsigned char right_motor) 
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
	//lcd_print(2,1,left_motor,3);
	//lcd_print(2, 5, right_motor, 3);
}	
/*kiran*/int print_line_sensor()
{
	 int left_line=0,center_line=0,right_line=0;
	 
	 if (ADC_Conversion(3)>32)    	//to print left line sensor detection W-white B-black
		 left_line=1;
	 if (ADC_Conversion(2)>32)	  //to print center line sensor detection W-white B-black
		center_line=1;
	 if (ADC_Conversion(1)>32)	  //to print right line sensor detection
	 	right_line=1;
	 line_conf = 100*left_line + 10*center_line +right_line;
	 //lcd_print(1,1,line_conf,3);
	 return line_conf;}
/*kiran*/void take_order1()
{
	char IA1,IA2;
	current_room++;
	shaft=15;
	slow_follow_line(0);
	forward();
	timer5_init();
	while(1)
	{
		if(ADC_Conversion(9)>40)
		{
			print_sensor(1,5,9);
			forward_mm(50);
			stop();
			break;
		}
	}
	IA1=color_detect();// add the the color servo functionality
	forward();
	timer5_init();
	if (current_room!=5)
	{
		while (1)
		{
			if ((int) ADC_Conversion(6)<30)
			{
				stop();
				break;
			}
		}
	}
	else{
		forward_mm(150);
		forward();
		while (1)
		{
			if ((int) ADC_Conversion(9)>40)
			{
				stop();
				break;
			}
		}
		forward_mm(40);
	}
	IA2=color_detect();
	if (judge_order(IA1,IA2)=='E')
	{
		IA2=color_detect();
		velocity(251,255);
		back_mm(150);
		_delay_ms(1000);
		back();
		while (1)
		{
			if (ADC_Conversion(9)>40)
			{
				stop();
				break;
			}
		}
		back_mm(50);
		IA1=color_detect();
		judge_order(IA1,IA2);
		if (current_room==5)
			forward_mm(620);
	}
	timer5_init();
	if (current_room!=5)
	{
			back();
			velocity(252,255);
			while (1)
			{
				if (print_line_sensor()==111)
				{
					stop(); 
					break;
				}
			}
		forward_mm(70);
		right();
		velocity(120,120);
		_delay_ms(300);
		turn_on_line('r');
		back_mm(50);
		find_line();
	}	
	else
	{	
		
		PORTH= PORTH & 0xCF;	 //turn off color sensor vcc and servo3
		PORTH= PORTH | 0x20;	 //turn on servo vcc
		timer1_init();
		servo3_pin_config();
		servo_3(150);		////////////////was 100
		_delay_ms(1000);
		servo_3_free();
		forward_mm(40);
		find_line();
		follow_line(111);
		forward_mm(72);
	}
	
	if (current_room!=5)
	take_order1();
	return;
}
/*kiran*/void enter_room(int room)
{
	if(room!=4)
	{
		print_line_sensor();
		shaft=56;
		follow_line(0);
		velocity(200,200);
		forward_mm(630);
		find_line();
		slow_follow_line(111);
		velocity(150,150);
		if (room!=2)
		{
			forward_mm(75);
			if(room==1)
				left();
			else
				right();
			velocity(150,150);
			_delay_ms(400);
			velocity(100,100);
			print_line_sensor();
			while (line_conf!=10)
				print_line_sensor();
			stop();
		}
		shaft=18;		
		slow_follow_line(0);
		forward();
		velocity(250,250);
		ShaftCountLeft=0;
		while(ShaftCountLeft<=70);
		stop();
		velocity(150,150);
		left_degrees(90);
	}		
	else 
	{
		shaft=56;
		follow_line(0);
		velocity(200,200);
		forward_mm(290);
		right_degrees(90);
	}
	forward_mm(145);
	find_line();
	follow_line(111);
	velocity(150,150);
	stop();
	forward_mm(50);
}
/*kiran*/void dump_garbage(int room)
{
	char turn='r';
	int garbage=1;		
	//dumping garbage will always initiate from cross inside the room i.e. room home
	if(ADC_Conversion(9)-ADC_Conversion(13)>15)
			turn='r';
					//garbage is on left side
	else if(ADC_Conversion(13)-ADC_Conversion(9)>15)
			turn='l';	//garbage is on right
	else 
		{
			turn='r'; 
			garbage=0;
		}
	if (garbage==1)
	{
		garbage_rank++;
		position='D';
	}
	else
	position='S';
	velocity(150,150);
	forward_mm(21);//was 25
	//velocity(100,100);
	
	turn_on_line(turn);
	forward_mm(40);
	clip_open();
	//_delay_ms(1000);
	back_mm(40);	
	if (garbage!=0)
		{
			if(turn=='r'){
				turn='l';
				velocity(200,200);
				left_degrees(90);}
			else{
				turn='r';
				velocity(200,200);
			right_degrees(90);}
			turn_on_line(turn);
			forward_mm(35);
			clip_close();
			//_delay_ms(1000);
			back_mm(35);
			turn_on_line(turn);
		}
	else
		turn_on_line(turn);
	shaft=56;
	follow_line(0);
	velocity(150,150);
	//timer5_init();
	forward_mm(269);////////////was 255
	if(sorted_rooms[count1+1]==0 && garbage==0)
	{
		right_degrees(90);
		forward_mm(300);
		find_line();
		//velocity(100,100);
		slow_follow_line(111);
		//velocity(100,100);
		//timer5_init();
		//forward_mm(75);
		buzzer_on();
		_delay_ms(5000);
		buzzer_off();
		return;
	}
	if (room!=4)
	{
		right_degrees(90);
		forward_mm(300);
		find_line();
		//velocity(100,100);
		slow_follow_line(111);
		velocity(100,100);
		//timer5_init();
		forward_mm(75);
		
		if(room!=2)
		{
			if(room==1)
				turn_on_line('r');
			else
				turn_on_line('l');
		}
		shaft=18;
		slow_follow_line(0);
		_delay_ms(200);
		velocity(200,200);
		forward_mm(650);
	}
	else 
	{
		left_degrees(90);
		forward_mm(260);
	}
	find_line();
	follow_line(111);
	forward_mm(60);
	if (garbage==1)
		turn_on_line('r');
	else
		return;
	ShaftCountLeft=0;
	while (1)
	{
		follow_line(111);
		if(ShaftCountLeft>=110)
			break;
	}
	//blink_red();/////////////////////////////////////////////////////////////////////////////////////////////////////
	if (garbage_rank==1)
	{
		forward_mm(50);
		left_degrees(45);
		clip_open();
		//_delay_ms(10);
		left_degrees(110);
		//forward_mm(70);	
	}
	else if (garbage_rank==2)
	{
		forward_mm(50);
		right_degrees(45);
		clip_open();
		//_delay_ms(1000);
		right_degrees(120);
		//forward_mm(70);
	}
	else if (garbage_rank==3)
	{
		//forward_mm(120);
		left_degrees(30);
		clip_open();
		//_delay_ms(1000);
		left_degrees(120);
		find_line();
		//turn_on_line('l');
	}
	else if (garbage_rank==4)
	{
		//forward_mm(120);
		right_degrees(30);
		clip_open();
		//_delay_ms(1000);
		right_degrees(100);
		find_line();
		//turn_on_line('r');
	}
	if(sorted_rooms[count1+1]==0 && garbage!=0){
		return_home();
	}
	find_line();
	//slow_follow_line(111);
	//back_mm(50);
	return;
}




/*rishabh*/void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
/*rishabh*/void buzzer_on (void) 
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}
/*rishabh*/void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
/*rishabh*/void GPIO_pin_config(void)
{
	DDRL = DDRL | 0xC3;   
	//DDRD = DDRD & 0x0F;  
	PORTL = PORTL | 0xC3;
	DDRH= DDRH | 0x30;
	//PORTH= PORTH | 0x30;	 //turn on color sensor vcc and servo3
	PORTH= PORTH & 0xCF;	 //turn off color sensor vcc and servo3
	//PORTH= PORTH | 0x20;	 //turn on servo3 vcc
	//PORTH= PORTH | 0x10;	 //turn on color sensor vcc
}
/*rishabh*/void find_line()
{
	print_line_sensor();
	if(line_conf==0)
	{
		ShaftCountRight=0;
		//left();
		right();
		velocity(120,120);
		while(ShaftCountRight<12)
		{
			print_line_sensor();
			if(line_conf!=0)
			{
				stop();
				return;
			}
		}
			ShaftCountRight=0;
			//right();
			left();
			velocity(120,120);
			while(ShaftCountRight<24)
			{
				print_line_sensor();
				if(line_conf!=0)
				{
					stop();
					return;
				}
			}
		
		//if no line found			
			ShaftCountRight=0;
			//left();
			right();
			velocity(120,120);
			while(ShaftCountRight<12)
				print_line_sensor();
			stop();
			forward();
			while (1)
			{
				print_line_sensor();
				if (line_conf!=0)
				return;
			}
			
			return;
	}
	
	else if(line_conf!=0)
	return;}
/*rishabh*/void slow_follow_line(int RqrdLineConf) 
{
	int last_line_conf=0;
	ShaftCountRight=0;
	print_line_sensor();
	forward();
	while(1)
	{
		if(line_conf==100)
		velocity(50,150);
		else if(line_conf==110)
		velocity(70,100);
		else if(line_conf==1)
		velocity(150,50);
		else if(line_conf==11)
		velocity(100,70);
		else if (line_conf==10)
		velocity(120,120);
		else if(line_conf==111)
		velocity(100,100);
		else if(line_conf==0)
		{
			if (last_line_conf>10)
			velocity(0,80);
			else if(last_line_conf<10)
			velocity(80,0);
			else
			velocity(100,100);
			line_conf=last_line_conf;
		}
		else
		velocity(100,100);
		last_line_conf=line_conf;
		print_line_sensor();
		if (line_conf==RqrdLineConf)
		{
			if (RqrdLineConf==111)
			break;
			else if(ShaftCountRight>=shaft)
			break;
		}
	}
	stop();
	velocity(100,100);
	return;
}
/*rishabh*/void follow_line(int RqrdLineConf)
{
	int last_line_conf=0;
	ShaftCountRight=0;//,ShaftCountLeft=0;   ///////////////////////////////////////////////////////////////////////////////////
	print_line_sensor();
	forward();
	//back();
	while(1)
	{	
		if(line_conf==100)
		velocity(130,200);
		else if(line_conf==110)
		velocity(150,200);
		else if(line_conf==1)
		velocity(200,130);
		else if(line_conf==11)
		velocity(200,150);
		else if (line_conf==10)
		velocity(220,220);
		else if(line_conf==111)
		velocity(220,220);
		else if(line_conf==0)
		{
			if (last_line_conf>10)
			velocity(0,100);
			else if(last_line_conf<10)
			velocity(100,0);
			else
			velocity(200,200);
			line_conf=last_line_conf;
		}
		else
		velocity(200,200);
		last_line_conf=line_conf;
		print_line_sensor();
		if (line_conf==RqrdLineConf)
		{
			if (RqrdLineConf==111)
				break;
			else if(ShaftCountRight>=shaft)// | ShaftCountLeft>=shaft)   ////////////////////////////////////////////////////////
				break;
		}
	}
	stop();
	velocity(200,200);
	return;
}
/*rishabh*/void turn_on_line(char direction)
{
	velocity(100,100);
	if(direction=='r')
		right();
	else
		left();
	_delay_ms(1000);
	
	while(1)
		if(ADC_Conversion(2)>32)
		break;
	stop();
	return;
}
/*rishabh*/void adc_pin_config (void) 
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}
/*rishabh*/void adc_init() 
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
/*rishabh*/unsigned char ADC_Conversion(unsigned char Ch)
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
/*rishabh*/void color_sensor_pin_config(void) 
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}
/*rishabh*/void color_sensor_scaling() 
{
	//Output Scaling 20% from datasheet
	//PORTD = PORTD & 0xEF;
	PORTD = PORTD | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTD = PORTD | 0x20; //set S1 high
}
/*rishabh*/void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   
	PORTL = PORTL | 0x18; 
}
/*rishabh*/void return_home()
{
	if(position=='D')
	{
		follow_line(111);
		forward();
		_delay_ms(1000);
		follow_line(111);
		turn_on_line('l');
	}
	shaft=56;
	follow_line(0);
	velocity(200,200);
	forward_mm(630);
	find_line();
	follow_line(111);
	stop();
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();}




/*ayush*/void clip_close(void) 
{	
	for (int i=0;i<181;i++)
		{
			servo_1(i);
			servo_2(180-i);
			_delay_ms(10);
		}
	_delay_ms(800);
	servo_1_free();
	servo_2_free();
}
/*ayush*/void clip_open(void)
{
		for (int i=0;i<181;i++)
		{
			servo_2(i);
			servo_1(180-i);
			
		}
		_delay_ms(800);
		servo_1_free();
		servo_2_free();
}
/*ayush*/char color_detect() 
{
	//lcd_wr_command(0x01);
	//red
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD & 0x7F; //set S3 low
	pulse=0; 
	_delay_ms(100); 
	red = pulse-300;
	//lcd_print(1,1,red,5);
	_delay_ms(100);	
	
	//green
	PORTD = PORTD | 0x40; //set S2 High
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	green = pulse; 
	//lcd_print(1,7,green,5);
	_delay_ms(100);	
	
	//blue
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	blue = pulse;
	
	//lcd_print(2,1,blue,5);
	_delay_ms(100);	
	if(red<threshold+50 && green<threshold && blue<threshold)
	    color = 'K';
	else
	{
		if (red>green && red >blue)
		{
			color = 'R';
			//blink_red();
			 PORTL = PORTL & 0x7F;
	 		_delay_ms(1000);   
	 		PORTL = PORTL | 0xC3;
		}
		else if (green>red && green > blue)
		{
			color = 'G';
			//blink_green();
			PORTL = PORTL & 0xBF;
			 _delay_ms(1000);
	 		PORTL = PORTL | 0xC3;
 
		}
		else if (blue>red && blue>green)
		{
			color = 'B';
			//blink_blue();
			PORTL = PORTL & 0xFD;
			 _delay_ms(1000);
	 		PORTL = PORTL | 0xC3;
		}
		else
		{
			color = 'E';
			color_detect();
		}
	}
	//lcd_cursor_char_print(2,10,color);
	//lcd_wr_command(0x01); //Clear the LCD
	return color;
}
/*ayush*/char judge_order(char room1,char room2)
{
	 char order1;
	 if (room1 != 'K') // K is black
	 {
		 if (room1 == room2)
		 {
			 order1 = room1;
			 orders[current_room-1] = room1;
			 pref[current_room-1] = 4;
		 }
		 else if (room2 == 'K')
		 {
			 
			 order1=room1;
			 orders[current_room-1] = room1;
			 if(room1=='G') pref[current_room-1]=3;
			 else if(room1=='R') pref[current_room-1]=2;
			 else if(room1=='B') pref[current_room-1]=1;
		 }
		 else
		 order1 = 'E';  //E as error we need to detect the color again
	 }
	 
	 else
	 {
		 if (room2 == 'K')
		 {
			 order1 = 'N';    //N for Do Not Disturb Room
			 orders[current_room-1] = 'N';
			 pref[current_room-1] = 0;
		 }
		 else
		 {
			 order1 = room2;
			 orders[current_room-1] = room2;
			  if(room2=='G') pref[current_room-1]=3;
			  else if(room2=='R') pref[current_room-1]=2;
			  else if(room2=='B') pref[current_room-1]=1;
		 }
	 }
	 lcd_cursor_char_print(2,1,order1);
	 return order1;
}
/*ayush*/void sort_orders()
{
	int max=0; 
	int max_room=0;
	for(int j=1;j<5;j++)
	{ 
		 for(int i=1;i<5;i++)
		 {
			if (pref[i]>max) {max=pref[i]; max_room=i;}
		 }
		if(max>0){
			sorted_rooms[j]=max_room;
			pref[max_room]=0;
		}
		max_room=0;
		max=0;
	}	
}
/*ayush*/void pickup_service_dumping_section(char current_service)
{
	int cross,tempv=0,ret=1;
	
	if (current_service=='R')
		cross=3;
	else if (current_service=='G')
		cross=1;
	else if	(current_service=='B')
		{
			cross=4;
			ret=2;
		}
	while(1)
	{
		follow_line(111);
		//print_sharp_sensor();
		tempv++;
		if(tempv>=cross) 
		break;
		velocity(150,150);
		forward();
		_delay_ms(500);
		stop();
		
	}
	stop();
	//_delay_ms(1000);
	
	//move certain distance forward###################
	velocity(100,100);
	forward_mm(50);
	stop();
	//_delay_ms(1000);
	
	//turn_on_line('r');
	
	//determine whether the service is on left or right
	char turn;
	if((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) 
		turn='r';
	else 
		turn='l';
	
	//turn accordingly
	turn_on_line(turn);
	clip_close();
	//use servo to pickup###################
	//_delay_ms(2000);
	
	//go to service line
	if (current_service=='G'){
		if (turn=='r')
			turn_on_line('l');
		else 
			turn_on_line('r');
	}
	else 
		turn_on_line(turn);
	
	tempv=0;
	while(1)
	{
		follow_line(111);
		//print_sharp_sensor();
		tempv++;
		if(tempv>=ret) 
			break;
		velocity(200,200);
		forward();
		_delay_ms(300);
		stop();
	}
	velocity(100,100);
	forward_mm(60);
	if(current_service=='G') 
		turn_on_line('l');
	else 
		turn_on_line('r');
	return;
}
/*ayush*/void pickup_service_Shome(char current_service)
{
	//the center point of the two wheels is exactly on service home
	char turn;
	//velocity(100,100);
	//forward_mm(70);
	if(current_service=='G')
	{
		turn_on_line('r');
		follow_line(111);
		velocity(150,150);
		forward_mm(60);
		if((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) 
			turn='r';
		else 
			turn='l';
		turn_on_line(turn);
		//_delay_ms(100);
		clip_close();
		//_delay_ms(2000);
		//servo operation
		turn_on_line(turn);
		follow_line(111);
		velocity(150,150);
		forward_mm(60);
		turn_on_line('l');
	}
	else
	{
		turn_on_line('l');
		follow_line(111);
		if(current_service=='B')
		{
			velocity(200,200);
			forward();
			_delay_ms(300);
			follow_line(111);
		}
		
		velocity(150,150);
		forward_mm(60);
		if((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) 
			turn='r';
		else 
			turn='l';
		turn_on_line(turn);
		clip_close();
		//_delay_ms(200);//servo operation
		turn_on_line(turn);
		follow_line(111);
		if(current_service=='B')
		{
			velocity(200,200);
			forward();
			_delay_ms(300);
			follow_line(111);
		}
		velocity(150,150);
		forward_mm(60);
		turn_on_line('r');
	}
}
/*ayush*/void delivery()
{	
	//position can be only dumping section or service home
	//for service home position=s
	//for dumping area position=D	
	position='S';
		for(count1=1;count1<5;count1++)
		{
			int room=sorted_rooms[count1];
			if(room!=0)
			{
				char service=orders[room];
				if(position=='D')
				pickup_service_dumping_section(service);		//bot has picked up the service and came to service and facing to the center
				else
				pickup_service_Shome(service);
				enter_room(room);		//bot will enter and stop at the room center where line_conf=111
				dump_garbage(room);
			}
		}		//it will detect the garbage, put the service at empty space and pick up the garbage and dump it and wait at dumping section otherwise home
}
/*ayush*/void init_devices()
{
	cli();									//Clears the global interrupt
	motion_pin_config();							//robot motion pins config
	left_encoder_pin_config();
	right_encoder_pin_config();
	color_sensor_pin_config();
	GPIO_pin_config();							//GPIO pins config for LEDs to glow
	buzzer_pin_config();
	//lcd_port_config();
	adc_pin_config();
	//MOSFET_switch_config();						//control switch for ir and line sensor leds
	timer5_init();								//PWM for velocity of bot or DC motors
	color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	//lcd_set_4bit();
	//lcd_init();
	color_sensor_scaling();
	sei();									// Enables the global interrupt}
/*ayush*/int main()
{
	init_devices();
	PORTH= PORTH | 0x10;	 		//turn on color sensor vcc and servo3
	take_order1();
		servo1_pin_config();							//Configure PORTB 5 pin for servo motor 1 operation
		servo2_pin_config();							//Configure PORTB 6 pin for servo motor 2 operation
	sort_orders();
	delivery();
	while (1);
}
