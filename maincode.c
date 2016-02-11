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

required functions-
1.take order: will judge the orders and go to service home
2.delivery :function(which can deliver the service to the room from service home or dumping section 
						moreover dump the garbage and stay at dumping section if any otherwise will come to home)






****************************************************************************************************/
//#define __OPTIMIZE__ -O0
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"
//constants
const int line_sensor_distance=200;
const int threshold_line_sensor_value=70;
const int Csensor_pos=50;   //distance of color sensor from the left sharp sensor
const int max_speed=150,turn_speed=130;
const int threshold=700;	//threshold value to decide the color

//volatile variables
volatile unsigned long int ShaftCountLeft = 0,shaftleft=0;
volatile unsigned long int ShaftCountRight = 0,shaftright=0;
volatile unsigned int Degrees,sharp,value;
volatile unsigned long int pulse = 0;
volatile unsigned long int  red;
volatile unsigned long int  blue;
volatile unsigned long int  green;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
volatile unsigned char adc_reading;
volatile unsigned int sharp_left=0,sharp_right=0,sharp_front=0,sharp_left_diff,sharp_right_diff,sharp_front_diff;
//range of the sharp sensor is 10cm to 80cm
int left_line=0,center_line=0,right_line=0;
int line_conf=0;
int ShortLeft=0,ShortFront=0,ShortRight=0;	//proximity sensors analog values for distance ranges 0 to 10cms ONLY

char color;
int count;
int KLp,KLi=0,KLd=0,KWp=20,KWi=10,KWd=10; //kLp is proportionality constant for auto line follower and kwp for wall following
//int distance;
float BATT_Voltage;
int pref[5];								//the type of room is saved sequentially 1->vip	0->regular	(-1)->DND room
char orders[5];							//the orders of the rooms 1234 in sequence 2nd position has order room1's order
int sorted_rooms[5]={0,0,0,0,0};		//the final sequence of rooms bot has to provide service
int current_room=1;


//SENSOR CONFIGURATION AND PREDEFINED FUNCTIONS
void lcd_port_config (void){
	DDRC = DDRC | 0xF7; 
	PORTC = PORTC & 0x80; 
}
void linear_distance_mm(unsigned int DistanceInMM) {
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
void adc_pin_config (void) {
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}
void adc_init() {
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
unsigned char ADC_Conversion(unsigned char Ch) {
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
/*unsigned int Left_Sharp_GP2D12_estimation(unsigned char adc_reading) {	int sharp_error;
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
}*/
void buzzer_pin_config (void){
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
void GPIO_pin_config(void) {
	DDRL = DDRL | 0xC3;   
	//DDRD = DDRD & 0x0F;  
	PORTL = PORTL | 0xC3;	 
}
void color_sensor_pin_config(void) {
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}
void color_sensor_pin_interrupt_init(void) {
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}
ISR(INT0_vect) {
	pulse++;
}
void color_sensor_scaling()	{
	//Output Scaling 20% from datasheet
	//PORTD = PORTD & 0xEF;
	PORTD = PORTD | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTD = PORTD | 0x20; //set S1 high
}
void motion_pin_config (void){
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   
	PORTL = PORTL | 0x18; 
}
void left_encoder_pin_config (void) {
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}
void right_encoder_pin_config (void) {
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void left_position_encoder_interrupt_init (void) {
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}
void right_position_encoder_interrupt_init (void) {
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}
ISR(INT5_vect) {
	ShaftCountRight++;
	shaftright++;
}
ISR(INT4_vect){
	ShaftCountLeft++;
	shaftleft++;
} 
void motion_set (unsigned char Direction){
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
void servo1_pin_config (void) {
 DDRB  = DDRB | 0x20;  
 PORTB = PORTB | 0x20; 
}
void servo2_pin_config (void) {
 DDRB  = DDRB | 0x40;  
 PORTB = PORTB | 0x40; 
}
void servo3_pin_config (void) {
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
void timer1_init(void) {
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
void timer5_init() {
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
void velocity (unsigned char left_motor, unsigned char right_motor) {
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
void blink_red(){
	 PORTL = PORTL & 0x7F;
	 _delay_ms(1000);   
	 PORTL = PORTL | 0xC3;
}
void blink_green() {
	 PORTL = PORTL & 0xBF;
	 _delay_ms(1000);
	 PORTL = PORTL | 0xC3;
 }
void blink_blue() {
	 PORTL = PORTL & 0xFD;
	 _delay_ms(1000);
	 PORTL = PORTL | 0xC3;
 }
void print_battery_voltage(){
	BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;
	lcd_print(1,1,BATT_Voltage,4);
}
void servo_1(unsigned char degrees) {
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}
void servo_2(unsigned char degrees) {
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}
void servo_3(unsigned char degrees) {
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}
void servo_1_free (void) {
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}
void servo_2_free (void) {
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}
void lcd_cursor_char_print(char row,char column,char letter){       
	lcd_cursor (row,column);
	lcd_wr_char(letter);
}
void forward (void) {
	motion_set(0x06);
	//velocity(252,255);
}
void back (void) {
	motion_set(0x09);
	//velocity(252,255);
}
void left (void) {
	motion_set(0x05);
	//velocity(252,255);
}
void right (void) {
	motion_set(0x0A);
	//velocity(252,255);
}
void stop (void) {
	motion_set(0x00);
}
void angle_rotate(unsigned int Degrees) {
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
	stop(); //Stop robot
}
void forward_mm(unsigned int DistanceInMM) {
	forward();
	linear_distance_mm(DistanceInMM);
}
void back_mm(unsigned int DistanceInMM) {
	back();
	linear_distance_mm(DistanceInMM);
}
void left_degrees(unsigned int Degrees) {
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	
	angle_rotate(Degrees);
	
}
void right_degrees(unsigned int Degrees) {
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right();
	angle_rotate(Degrees);
}
void print_sensor(char row, char coloumn,unsigned char channel) {
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
void AlignColorSensor(){
		servo_3(95);
		_delay_ms(1000);
		int i=0;
		while (i<=95)
		{
			servo_3(95-i);
			_delay_ms(10);
			i++;
		}
		_delay_ms(1000);
}
void ResetColorSensor(){
	servo_3(95);
	_delay_ms(2000);
}
//OUR MAIN FOCUS WOULD BE THESE FUNCTIONS
void clip_close(void) {	
	for (int i=0;i<200;i++)
		servo_1(i);
	
	
	//wait();
	//servo_2(0);
	//wait();
	_delay_ms(2000);
}
void clip_open(void) {
		servo_1(0);
		//wait();
		//servo_2(180);
		//wait();
		_delay_ms(2000);
}
void buzzer_on (void) {
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}
void buzzer_off (void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
char color_detect() {
	AlignColorSensor();
	red_read();
	lcd_print(1,1,red,5);
	_delay_ms(1000);

	green_read();
	lcd_print(1,7,green,5);
	_delay_ms(1000);

	blue_read();
	lcd_print(2,1,blue,5);
	_delay_ms(1000);
	
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
	
	return color;
 }
char judge_order(char room1,char room2){
	 char order1;
	 if (room1 != 'K') // K is black
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
void red_read(void) {
	//Red
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD & 0x7F; //set S3 low
	pulse=0; 
	_delay_ms(100); 
	red = pulse;  
	
	}
void green_read(void) {
	PORTD = PORTD | 0x40; //set S2 High
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	green = pulse;  
	
	}
void blue_read(void) {
	PORTD = PORTD & 0xBF; //set S2 low
	PORTD = PORTD | 0x80; //set S3 High
	pulse=0; 
	_delay_ms(100); 
	blue = pulse;  
	}
int print_line_sensor(){		
	 
	 if (ADC_Conversion(3)>32)    //to print left line sensor detection W-white B-black
	 {	
		 left_line=1;
	 }
	 else
	 {	
		left_line=0;
	 }
	 
	 if (ADC_Conversion(2)>32)	  //to print center line sensor detection W-white B-black
	 {	
		center_line=1;
	 }
	 else
	 {	
		center_line=0;
	 }
	 
	 if (ADC_Conversion(1)>32)	  //to print right line sensor detection
	 {	
		right_line=1;
	 }
	 else
	 {	
		right_line=0;
	 }
	 
	 line_conf = 100*left_line + 10*center_line +right_line;
	 lcd_print(1,1,line_conf,3);
	 return line_conf;
	}
void print_sharp_sensor(){
 	unsigned int sharp,value;
	 
	 value = ADC_Conversion(9);
	 sharp =side_Sharp_GP2D12_estimation(value);
	 sharp_left_diff = sharp_left - sharp;
	 sharp_left=sharp;
	 lcd_print(2,1,value,3);
	 lcd_print(1,1,sharp,3);
	 value = ADC_Conversion(11);
	 sharp=Sharp_GP2D12_estimation(value);
	 sharp_front_diff = sharp_front - sharp;
	 sharp_front=sharp;
	 lcd_print(1,5,sharp,3);
	 lcd_print(2,5,value,3);
	 value = ADC_Conversion(13);
	 sharp=side_Sharp_GP2D12_estimation(value);
	 sharp_right_diff = sharp_right - sharp;
	 sharp_right=sharp;
	 lcd_print(1,9,sharp,3);
	 lcd_print(2,9,value,3);
	 //lcd_print(1,13,sharp,3);
	 
 }
void auto_line_follow(int required_line_conf){
	 int I=0,correction=0,L_speed=0,R_speed=0,error=0,prev_error=0,speed=0;
	 
	 
	 //lcd_print(2,1,KLp,3);
	 print_line_sensor();
	 while (1)
	 {
		 
	 //print_line_sensor();
	 
	 if (line_conf == 111)
	 _delay_ms(10);
	 //stop();
	 
	 else if (line_conf == 101)
	 {
		forward();
		velocity(turn_speed,turn_speed);
	 }
	 else if (line_conf == 0)
	 {
		 forward();
		 if (error>0)
		 velocity(turn_speed+30,0);
		 else if (error<0)
		 velocity(0,turn_speed+30);
		 }
	 
	 else 
	 {
		 if (line_conf == 100)
		 {
			error = -2;
			speed = max_speed;
		 }
		 else if(line_conf == 110)
		 {
			error = -1;
			speed = turn_speed - 40;
		 }
		 else if(line_conf == 10)
		 {
			error = 0;
			speed = max_speed;
		 }
		 else if(line_conf == 11)
		 {
			error = 1;
			speed = max_speed -40;
		 }
		 else if(line_conf == 1)
		 {
			error = 2;
			speed = turn_speed;
		 }
		 
		 I = I + error;
		 correction = KLp*error + KLi*I + KLd*(prev_error - error);
		 L_speed = speed + correction;
		 R_speed = speed - correction;
		 prev_error = error;
		 forward();
		 velocity(L_speed,R_speed);
	 }
	 //_delay_ms(100);
	 print_line_sensor();
	 if (line_conf==required_line_conf)
	 {
		stop();
		break;
	 }
	 }	 
 }
void take_order() {		
	//we have add a feature of recheck if by mistake it is detecting more than one vip rooms
	
	char room1,room2;
	print_line_sensor();			//final function for turning left or right till line sensor detects the line
	left();
	_delay_ms(200);
	velocity(90,90);
	ShaftCountLeft=0;
	while (1)
	{
		print_line_sensor();
		if (line_conf == 10)
		{
			stop();
			break;
		}
	}							
	red=ShaftCountLeft+5;
	forward_mm(80);
	_delay_ms(100);
	ShaftCountLeft=0;
	right();
	velocity(turn_speed,turn_speed);
	while (1)
	{
		if (ShaftCountLeft>=red)
		{
			stop();
			break;
		}
		
	}
	//buzzer_on();
	//_delay_ms(100);
	//buzzer_off();
	//_delay_ms(10000);
	//right_degrees(90);
	
	while(current_room<4)
	{
		//print_sharp_sensor();
		_delay_ms(100);
		forward_mm(200);
		//while(sharp_left > 100)
		//print_sharp_sensor();
		stop();
		//_delay_ms(20000);
		//room1=color_detect();
		//print_sharp_sensor();
		/*while(sharp_front >= 200)
			follow_right_wall(200);
		stop();*/
		//we have to check if the bot is straight and close enough to second indicator
		
		_delay_ms(3000);
		
		ShaftCountRight = 0;
		forward();
		while(1)
		{	
			print_sharp_sensor();
			if(ShaftCountRight >= 94)
			{
				stop();
				break;
			}				
		}
		stop();	
		_delay_ms(2000);
		//room2=color_detect();
		//right_degrees(180);
		//_delay_ms(1000);
		back();
		velocity(115,120);
		//check velovity
		//ShaftCountRight=0;
		/*while(1)
		{
			//print_sharp_sensor();
			if(ShaftCountRight >= 94)
			{
				stop();
				break;
			}
		}
		_delay_ms(100);*/
		//stop();
		//back();
		//velocity(110,120);
		print_line_sensor();
		while (1)
		{
			print_line_sensor();
			if (line_conf==111)
			{
				stop();
				break;
			}
			
		}
		forward_mm(170);
		right_degrees(90);
		buzzer_on();
		_delay_ms(500);		//testing
		buzzer_off();
		_delay_ms(20000);
		while (judge_order(room1,room2)=='E')		//detecting the indicators again if we cant judge the final order from available color pair like red and green 
		{
			 back_mm(425);			//going back in front of the previous indicator
		 
			 room1=color_detect();
		 
			 while(sharp_front >= 200)
			 {
				 print_sharp_sensor();
				 forward();
				 velocity(turn_speed,turn_speed);
			 }
			 stop();
		 
			 _delay_ms(300);
			 forward_mm(60);
			 room2=color_detect();
		}
	
		orders[current_room]=judge_order(room1,room2);
	
		if (current_room<4)
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
			stop();
		}  
		else if (current_room==4)
		{	
			forward();
			velocity(turn_speed,turn_speed);
			print_line_sensor();
			while(line_conf != 111)
			{
				print_line_sensor();
			}
			stop();
			_delay_ms(200);
			forward_mm(line_sensor_distance);
			stop();
		}
		current_room++;
		}
}
void sort_orders(){ 
	 int j=2;
	 for(int i=1;i<5;i++)
	 {
		 if(pref[i]==1)
		 	sorted_rooms[1] = i;
		 else if (pref[i]==0)
		 {
			 sorted_rooms[j]=i;
			 j++;
		 }
	 }
	 
 }
void follow_right_wall(){
		 
			//int error;
		 	//value = ADC_Conversion(13);
	     	//sharp=side_Sharp_GP2D12_estimation(value);
	 	 	//sharp_right_diff = sharp_right - sharp;
			//sharp_right=sharp;
			//lcd_print(1,14,sharp,3);
			
			//_delay_ms(100);
			     
		 if(value>10)
		 {	
			 if (value<45)
				 velocity(90,80);
			 else if(value>55)
				 velocity(80,90);
			 else
				velocity(80,80);
			
			// _delay_ms(100);
			//velocity(L_speed,R_speed);
		 _delay_ms(200);
		 }
		 else
		 	stop();
		 
}
void follow_left_wall(int required_distance){
		 int I,correction,L_speed,R_speed,error,prev_error,speed=0,k=2; //k is speed proportionality constant
		 
		 	value = ADC_Conversion(9);
	     	sharp=Sharp_GP2D12_estimation(value);
	 	 	sharp_left_diff = sharp_left - sharp;
			 sharp_left=sharp;
		 //lcd_print(1,6,sharp,3);     
		 if(sharp_left!=800)
		 {
		 	 error = sharp_left - required_distance;
			 //P = error;
			 //I = I + error;
			 //D = prev_error - error;
			 correction = KWp*error + KWi*(I + error)+ KWd*(prev_error - error);
			 L_speed = k*max_speed - correction;
			 R_speed = k*max_speed + correction;
			 prev_error = error;
			 forward();
			 velocity(L_speed,R_speed);
		 }
		 else
		 {
		 	stop();
		 }
		 
}
void pickup_service_dumping_section(char current_service){		
	int cross,tempv=0;
	if (current_service=='R')
	cross=2;
	else if (current_service=='G')
	cross=1;
	else if	(current_service=='B')
	cross=3;
	while(tempv==cross)
	{
	auto_line_follow(111);
	print_sharp_sensor();
	if(sharp_left<600)			//to rule out the possibility line_conf becoming 111 at service since we dont have to consider it as a cross
	tempv++;
	}
	stop();
	return;
}
void pickup_service_Shome(char current_service){			
	//the centre point of the two wheels is exactly on service home
	int cross,tempv=0;
	if(current_service=='G')
	{
		while(line_conf!=010)
		{
			right();
			velocity(turn_speed,turn_speed);
		}
		stop();
	}
	else
	{
		print_line_sensor();
		while(line_conf!=010)
		{
			print_line_sensor();
			left();
			velocity(turn_speed,turn_speed);
		}
		stop();
		if (current_service=='R')
			cross=1;
		else if	(current_service=='B')
			cross=2;
		while(tempv==cross)
		{
			auto_line_follow(111);
			tempv++;
		}
			stop();
	}
}
void dump_garbage(current_room){		
	//dumping garbage will always initiate from cross inside the room i.e. room home
	if(current_room!=4)
	{
		while(sharp_front>200)
		{
			forward();
			velocity(max_speed,max_speed);
			print_sharp_sensor();
		}
		stop();
		right_degrees(90);
		print_sharp_sensor();
		while(abs(sharp_left_diff)<300)
		{
			follow_left_wall(120);
			print_sharp_sensor();
		}
		stop();
		if(current_room==2)
		{	
			print_sharp_sensor();
			while(sharp_right>400)
			{
				forward();
				velocity(turn_speed,turn_speed);
				print_sharp_sensor();
			}
			stop();
		}
		else
		{
			print_line_sensor();
			while(line_conf!=111)
			{
				forward();
				velocity(turn_speed,turn_speed);
				print_line_sensor();
			}
			forward_mm(line_sensor_distance);
			stop();
			print_line_sensor();
			while(line_conf!=010)
			{	
				if (current_room==1)
				right();
				else
				left();
				velocity(turn_speed/2,turn_speed/2);
				print_line_sensor();
			}
			stop();
			print_sharp_sensor();
			while(sharp_right>400)
			{
				forward();
				velocity(turn_speed,turn_speed);
			}
			stop();
		}
		print_sharp_sensor();
		while(abs(sharp_right_diff)<=250)
		{
			follow_right_wall(150);
			print_sharp_sensor();
		}
		stop();
		print_line_sensor();
		while(line_conf!=111)
		{
			forward();
			velocity(turn_speed,turn_speed);
			print_sharp_sensor();
		}
		forward_mm(line_sensor_distance);
		while(line_conf!=010)
		{
			right();
			velocity(turn_speed,turn_speed);
			print_line_sensor();
		}
		stop();
		print_line_sensor();
		//ShaftCountRight = 0;
		//distance
		while(line_conf!=111 )			//use shaft count for detection of second cross
		{	
			print_line_sensor();
		}
	}
}
void enter_room(int room){
	int flag=0;
	if(room!=4){
		forward();
		print_line_sensor();
		
		while(line_conf!=111){
			print_line_sensor();
			}
		stop();
		forward_mm(200);	//to get the wheels centre at the home centre
		if(room!=2){
			if(room==1)
			left();
			else if(room==3)
			right();
		}
		ShaftCountRight=0;
		//print_sharp_sensor();
		while(ShaftCountRight<=90)//will have to exact shaft count to reach at the doorstep the room from home
		{
			print_sharp_sensor();
			follow_right_wall(125);
			
		}
		stop();
		left_degrees(90);
	}		
	else if(room==4){
		forward_mm(600);
		right_degrees(90);
	}
	forward_mm(145);
	print_line_sensor();
	if(line_conf==0)
	{
		ShaftCountRight=0;
		left();
		velocity(90,90);
		while(ShaftCountRight<15)
		{
			print_line_sensor();
			if(line_conf!=0)
			{
				stop();
				flag=1;
				break;
			}
		}
		if (flag==0)
		{
			ShaftCountRight=0;
			right();
			velocity(90,90);
			while(ShaftCountRight<30)
			{
				print_line_sensor();
				if(line_conf!=0)
				{
					stop();
					flag=1;
					//follow_line(111);
					break;
				}
			}
		}
		
		if(flag==0)
		{
			ShaftCountRight=0;
			left();
			velocity(100,100);
			while(ShaftCountRight<15);
			stop();
			forward();
			velocity(100,100);
			//print_line_sensor();
			while(1)
			{
				print_line_sensor();
				if(line_conf==111)
				{
					stop();
					break;
				}
			}
		}
		
		if (flag==1)
		follow_line(111);
		
	}
	
	else if(line_conf!=0)
	follow_line(111);
	
	stop();
	forward_mm(80);
}
void delivery(char service,int room,char position){
		//position can be only dumping section or service home 
		//for service home position=s
		//for dumping area position=D
		if(position=='D')
			pickup_service_dumping_section(service);		//bot has picked up the service and came to service and facing to the center
		else 
			pickup_service_Shome(service);		

		enter_room(room);		//bot will enter and stop at the room center where line_conf=111
		dump_garbage(room);		//it will detect the garbage, put the service at empty space and pick up the garbage and dump it and wait at dumping section otherwise home
}
void calibrate(){
	int left1=ShaftCountLeft,right1=ShaftCountRight;

	if (right1>left1)
	{
		int diff=(right1-left1);
		forward();
		velocity(100,0);
		ShaftCountRight=0;
		ShaftCountLeft=0;
		while (1)
		{
			if(ShaftCountLeft >= ShaftCountRight)
			{
				velocity(100,100);
				return;
			}
		}
		
	}
	else if (right1<left1)
	{
		forward();
		velocity(0,100);
		while (1)
		{
			if(ShaftCountRight >= ShaftCountLeft)
			{
				velocity(100,100);
				return;
			}
		}
	}
	else return;
}
void follow_line(int RqrdLineConf){
	int last_line_conf=0;
	print_line_sensor();
	//forward();
	back();
	while(line_conf!=RqrdLineConf)
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
		}
		else
		velocity(100,100);
		last_line_conf=line_conf;
		print_line_sensor();
		//_delay_ms(50);
	}
	stop();
	return;
}
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}


//initialization functions
void port_init(){
	 //servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	 //servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	 //servo3_pin_config(); //servo3
	 motion_pin_config(); //robot motion pins config
	 left_encoder_pin_config(); 
	 right_encoder_pin_config(); 
	 //color_sensor_pin_config();
	 //GPIO_pin_config();	//GPIO pins config for LEDs to glow 
	 //buzzer_pin_config(); 
	 lcd_port_config();		
	 adc_pin_config();
	 MOSFET_switch_config(); //control switch for ir and line sensor leds
 }
void init_devices(){
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	//timer1_init();   //PWM for servo pins
	timer5_init();	 //PWM for velocity of bot or DC motors
	//color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	lcd_set_4bit();
	lcd_init();
	//color_sensor_scaling();
	sei();   // Enables the global interrupt
}
/*int main(void){  		//testing function to move the bot in alleys
	init_devices();	
	forward_mm(300);
	forward();
	ShaftCountLeft=0;
	ShaftCountRight=0;
	while (ADC_Conversion(6)>=100)
	{
		value = ADC_Conversion(13);
		print_sensor(1,1,13);
		if(value>10)
		{
			if (value<43)
			velocity(110,90);
			else if (value<46)
			velocity(100,90);
			else if(value>53)
			velocity(90,110);
			else if (value>51)
			velocity(90,100);
			else
			velocity(100,100);
		}
		else
		{
			stop();
			break;
			
		}		
	}
		forward();
		if ((int) ShaftCountRight>(int) ShaftCountLeft)
		{
			velocity(100,0);
			while (1)
				if(ShaftCountLeft>=ShaftCountRight)
					break;
			
		}
		else if ((int) ShaftCountRight<(int) ShaftCountLeft)
		{
			velocity(0,100);
			shaftright=0;
			shaftleft=0;
			while (1)
				if(ShaftCountRight>=ShaftCountLeft)
					break;
		}
		lcd_print(2,1,ShaftCountLeft,4);
		lcd_print(2,7,ShaftCountRight,4);
		stop();
	while (1);	
}*/

int main(){
	init_devices();
	forward();
	//velocity(120,120);
	while(1){
	if(ADC_Conversion(9)>40)
		{stop();
		forward();
		_delay_ms(400);
		stop();
		break;}
	}	
	//color sensing
	_delay_ms(1000);
	forward();
	while(1)	{
		
		if(ADC_Conversion(6)<60)
		break;
	}
	stop();
	//check right wall distance. if too less then turn the bot right and according to the distace move it towards the indicator.
	//color sensing
	_delay_ms(2000);
	
	back();
	while(1){
		print_line_sensor();
		if(line_conf==111)
		break;	
	}
	stop();
	forward();
	_delay_ms(560);
	stop();
	_delay_ms(2000);
	//right_degrees(30);
	//_delay_ms(1000);
	right();
	_delay_ms(100);
	velocity(90,90);
	while(1)
	{
		print_line_sensor();
		if (line_conf==10)
		{
			stop();
			break;
		}
		
	}
	_delay_ms(2000);
	print_line_sensor();
	back();
		//back();
		follow_line(111);
	stop();
	timer5_init();
	forward();
	while(1);
}
