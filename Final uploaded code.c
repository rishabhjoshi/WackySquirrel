/*
*
* Team Id: 				eYRC-HS#3528
* Author List: 			Kiran Dhamane, Ayush Sawarni, Rishabh Joshi, Siddharth Gupta
* Filename: 			GccApplication2.c
* Theme: 				Hotel Guest Service
* Functions: 			motion_set(), forward(), back(), left(), right(), stop(), linear_distance_mm(int), forward_mm(int), back_mm(int),
*						angle_rotate(int), left_degrees(int), right_degrees(int), servo1_pin_config(), servo2_pin_config(), servo3_pin_config(),
*						servo_1(int), servo_2(int), servo_3(int), servo_1_free(), servo_2_free(), servo_3_free(), color_sensor_pin_interrupt_init(),
*						left_encoder_pin_config(), right_encoder_pin_config(), left_position_encoder_interrupt_init(),
*						right_position_encoder_interrupt_init(), timer1_init() ,timer5_init() ,velocity(int,int) ,print_line_sensor() ,take_order1(),
*						enter_room(), dump_garbage(), return_home(), buzzer_pin_config(), buzzer_on(), buzzer_off(), GPIO_pin_config(),
*						find_line(), slow_follow_line(int), follow_line(int), turn_on_line(char), adc_pin_config(), adc_init(), ADC_Conversion(int),
*						color_sensor_pin_config(), color_sensor_scaling(), motion_pin_config(), clip_close(), clip_open(), color_detect(),
*						judge_order(char,char), sort_orders(), pickup_service_dumping_section(char), pickup_service_Shome(char), delivery(),
*						init_devices(), main().
* Global Variables:		const int threshold					-	threshold value of Black color, (550 - night, 800 - daylight)
*						volatile unsigned long ShaftCountLeft,ShaftCountRight	-	Variables which will hold the number of shaft counts that have passes (left/right),
*						volatile unsigned long pulse							-	Number of pulses while detecting a color using color sensor,
*						volatile unsigned long red,blue,green					-	Volatile variables to measure pulse count after setting the particular filters (r/b/g),
*						unsigend char ADC_Value 			-	Stores the value of ADC,
*						volatile unsigned char line_conf 	-	Three digit integer to specify the line configuration, eg = 100 means that the sequence of
*																colors detected is Black White White.
*						int shaft 							-	Every time we use follow_line(0) shaft is an approx shaft count after which line ends ,
*						int arbage_rank 					-	the rank of present garbage in dumping section,
*						char color 							-	Color detected,
*						int pref[5] 						-	Array storing room prefs, the type of room is saved sequentially 1->vip	0->regular	(-1)->DND room,
*						char orders[5] 						-	the orders of the rooms 1234 in sequence 2nd position has order room1's order,
*						int sorted_rooms[6]					-	the final sequence of rooms bot has to provide service,
*						int current_room 					-	The current room,
*						char position 						-	Current position. eg - S -> Service Home,
*						int count1							-	Works as array index.
*
*/#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"

const int threshold=900;	//threshold value to decide the color 550 in the night and 800 in the daylight

volatile unsigned long int ShaftCountLeft = 0;
volatile unsigned long int ShaftCountRight = 0;
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
char judge_order(char room1,char room2);

/*
*
*	Name:	Motion Set
*	Input:	A binary code telling about the direction of motion.
*	Output:	void
*	Logic:	Sets the desired configuration for the pins connected to the motor.
*	Example Call:	motion_set(0x06);
*
*/
void motion_set (unsigned char Direction)				//get checked
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}


/*
*
*	Name:	Forward
*	Input:	void
*	Output: void
*	Logic:	Sets the reuired input to motion_set for forward motion i.e. 0x06
*	Example call:	forward();
*
*/
void forward (void)
{
	motion_set(0x06);
}

/*
*
*	Name:	Back
*	Input:	void
*	Output:	void
*	Logic:	Sets the requires input to motion_set for backward motion i.e. 0x09
*	Example Call:	back();
*
*/
void back (void)
{
	motion_set(0x09);
}


/*
*
*	Name:	Left
*	Input:	void
*	Output:	void
*	Logic:	Sets the requires input to motion_set for left turn i.e. 0x05
*	Example Call:	left();
*
*/
void left (void)
{
	motion_set(0x05);
}


/*
*
*	Name:	Right
*	Input:	void
*	Output:	void
*	Logic:	Sets the requires input to motion_set for right turn i.e. 0x0A
*	Example Call: right();
*
*/
void right (void)
{
	motion_set(0x0A);
}


/*
*
*	Name:	Stop
*	Input:	void
*	Output:	void
*	Logic:	Sets the requires input to motion_set for ceasing motion i.e. 0x00
*	Example Call:	stop();
*
*/
void stop (void)
{
	motion_set(0x00);
}


/*
*
*	Name:	Linear_distance_mm
*	Input:	Distance in mm to be travelled by any motion set.
*	Output:	void
*	Logic:	Converts distance in mm to shaft count by dividing by 5.338. Then setting ShaftCountRight to 0. After that it
*			checks that the funciton is executed till the ReqdShaftCount is equal to ShaftCountRight.
*	Example call:	linear_distance_mm(45);
*
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;									// Shaft count that would equal the distance to be travelled.
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; 						// division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;										// Global variable. Updated automatically as the bot moves.
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)					// when distance travelled exceeds the required distance the
		// function breaks
		{
			break;
		}
	}
	stop(); 													//Stop robot
}


/*
*
*	Name:	forward_mm
*	Input:	Distance to be travelled in mm as integer.
*	Output:	void
*	Logic:	Uses forward() and linear_distance_mm() to move forward the specified distance.
*	Example Call:	forward_mm(60);
*
*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}


/*
*
* Function Name:	Back_mm
* Input:			Distance to be travelled in mm as integer.
* Output:			void
* Logic:			Uses back() and linear_distance_mm() to move backward the specified distance.
* Example Call:		back_mm(80);
*
*/
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}


/*
*
* Function Name:	angle_rotate
* Input:			Amount of degrees to be rotated
* Output:			void
* Logic:			Rotate the bot by converting degrees to appropriate shaft counts.
* 					Required shaft count = degrees/3.60351
* 					if any of the shaft counts exceeds required shaft count, the rotation stops.
* Example call:		angle_rotate(40);
*
*/
void angle_rotate(unsigned int Degrees)			// get checked
{
	float ReqdShaftCount = 0;					// represents the degrees in terms of shaft count
	unsigned long int ReqdShaftCountInt = 0;	// shaft count in integer

	ReqdShaftCount = (float) Degrees/3.60351 ; //was 4.090 division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;						// set both left and right shaft counts to 0
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop();										// stop robot
}


/*
*
* Function Name:	left_degrees
* Input:			Amount of degrees to be rotated
* Output:			void
* Logic:			Rotate the bot left using left() and angle_rotate();
* Example call:		left_degrees(40);
*
*/
void left_degrees(unsigned int Degrees)
{
	left(); 									//Initiate left turn motion
	angle_rotate(Degrees);						// stops the bot after required degrees
	
}


/*
*
* Function Name:	right_degrees
* Input:			Amount of degrees to be rotated
* Output:			void
* Logic:			Rotate the bot left using right() and angle_rotate();
* Example call: 	right_degrees(40);
*
*/
void right_degrees(unsigned int Degrees)
{
	right();									// Initiate right turn motion
	angle_rotate(Degrees);						// stops bot after required degrees
}

/*
*
* Function Name:	 servo1_pin_config
* Input:			void
* Output:			void
* Logic:
* Example call:		servo_pin_config(void);
*
*/
void servo1_pin_config (void)				// to be done
{
	DDRB  = DDRB | 0x20;
	PORTB = PORTB | 0x20;
}


/*
*
* Function Name:	servo2_pin_config
* Input:			voif
* Output:			void
* Logic:
* Example call:		servo2_pin_config();
*
*/
void servo2_pin_config (void)				// to be done
{
	DDRB  = DDRB | 0x40;
	PORTB = PORTB | 0x40;
}


/*
*
* Function Name:	serov3_pin_connfig
* Input:			void
* Output:			void
* Logic:
* Example call:		servo3_pin_config();
*
*/
void servo3_pin_config (void)				// to be done
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}


/*
*
* Function Name:	servo_1
* Input:
* Output:			void
* Logic:
* Example call:		servo1();
*
*/
void servo_1(unsigned char degrees)			// to be done
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


/*
*
* Function Name:	servo_2
* Input:
* Output:			void
* Logic:
* Example call:		servo_2();
*
*/
void servo_2(unsigned char degrees)			// to be done
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}


/*
*
* Function Name:	servo_3
* Input:
* Output:			void
* Logic:
* Example call:		servo_3();
*
*/
void servo_3(unsigned char degrees) 			// to be done
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}


/*
*
* Function Name:	servo_1_free
* Input:			void
* Output:			void
* Logic:
* Example call:		servo_1();
*
*/
void servo_1_free (void) 					// to be done
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}


/*
*
* Function Name:	servo_2_free
* Input:			void
* Output:			void
* Logic:
* Example call:		servo_2_free();
*
*/
void servo_2_free (void)
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

/*
*
* Function Name:	servo_3-free
* Input:			void
* Output:			void
* Logic:
* Example Call:		servo_3_free();
*
*/
void servo_3_free (void) //					to be done
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}

void color_sensor_pin_interrupt_init(void)
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}


/*
*
* Function Name: 	ISR(INT0_vect)
* Input: 			INT0_vect
* Output: 			NIL
* Logic: 			Increment pulse.
* Example Call:		/////////////////////
*
*/
ISR(INT0_vect)
{
	pulse++;
}


/*
*
* Function Name: 	left_encoder_pin_config
* Input: 			void
* Output: 			void
* Logic: 			//////////////////Increment pulse.
* Example Call:		left_encoder_pin_config();
*
*/
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}


/*
*
* Function Name: 	right_encoder_pin_config
* Input: 			void
* Output: 			void
* Logic: 			//////////////////Increment pulse.
* Example Call:		right_encoder_pin_config();
*
*/
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}


/*
*
* Function Name: 	left_position_encoder_interrupt_init
* Input: 			void
* Output: 			void
* Logic: 			//////////////////Increment pulse.
* Example Call:		left_position_encoder_interrupt_init();
*
*/
void left_position_encoder_interrupt_init (void)
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}


/*
*
* Function Name: 	right_position_encoder_interrupt_init
* Input: 			void
* Output: 			void
* Logic: 			//////////////////Increment pulse.
* Example Call:		right_position_encoder_interrupt_init();
*
*/
void right_position_encoder_interrupt_init (void)
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}


/*
*
* Function Name: 	ISR(INT5_vect)
* Input: 			INT5_vect
* Output: 			NIL
* Logic: 			Increment Shaft count right.
* Example Call:		/////////////////////
*
*/
ISR(INT5_vect)
{
	ShaftCountRight++;
}


/*
*
* Function Name: 	ISR(INT4_vect)
* Input: 			INT4_vect
* Output: 			NIL
* Logic: 			Increment shaft coutn left.
* Example Call:		/////////////////////
*
*/
ISR(INT4_vect)
{
	ShaftCountLeft++;
}



/*
*
* Function Name: 	timer1_init
* Input: 			void
* Output: 			void
* Logic: 			///////////////////Increment pulse.
* Example Call:		timer1_init();
*
*/
void timer1_init(void) 
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


/*
*
* Function Name: 	timer5_init
* Input: 			void
* Output: 			void
* Logic: 			///////////////////Increment pulse.
* Example Call:		timer5_init();
*
*/
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


/*
*
* Function Name: 	velocity
* Input: 			left_motor, right_motor (unsigned char) - //////////////////// 
* Output: 			void
* Logic: 			Initializes buzzer_pin by setting output as 0 then turning off buzzer ///////////////
* Example Call:		buzzer_pin_config(); //////////////////
*
*/
void velocity (unsigned char left_motor, unsigned char right_motor) 
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}	

/*
*
* Function Name: 	print_line_sensor
* Input: 			void
* Output: 			int - line configuration (3 digit number)
* Logic: 			Initializes buzzer_pin by setting output as 0 then turning off buzzer////////////////
* Example Call:		buzzer_pin_config();
*
*/
int print_line_sensor()
{
	 int left_line=0,center_line=0,right_line=0;
	 
	 if (ADC_Conversion(3)>32)    	//to print left line sensor detection W-white B-black
		 left_line=1;
	 if (ADC_Conversion(2)>32)	  //to print center line sensor detection W-white B-black
		center_line=1;
	 if (ADC_Conversion(1)>32)	  //to print right line sensor detection
	 	right_line=1;
	 line_conf = 100*left_line + 10*center_line +right_line;
	 return line_conf;
}


/*
*
* Function Name: 	take_order1
* Input: 			void
* Output: 			void
* Logic: 			Initializes buzzer_pin by setting output as 0 then turning off buzzer//////////////
* Example Call:		take_order1();
*
*/
void take_order1()
{
	char IA1,IA2; 								/*	IA1 for storing first indicator
													IA2 for storing second indicator	*/
	current_room++;	
	shaft=15;									// tells approx shaft count for follow line
	slow_follow_line(0);						// follow initial line
	forward();									
	timer5_init();		
	velocity(255,250);						
	while(1)
	{
		if(ADC_Conversion(9)>40)					//detect left wall 
		{
			forward_mm(50);							//move forward to detect first indicator
			stop();
			break;
		}
	}
	color_detect();
	IA1=color;
	forward();										
	timer5_init();	
		forward_mm(250);							
		forward();
		while (1)
		{
			if ((int) ADC_Conversion(9)>40)			// detect left wall
			{
				stop();
				break;
			}
		}
	color_detect();
	IA2=color;
										
	if (judge_order(IA1,IA2)=='E')						// if error in color detection
	{
		color_detect();
		IA2 = color;
		velocity(252,255);
		back_mm(150);								
		_delay_ms(1000);
		back();
		while (1)
		{
			if (ADC_Conversion(9)>40)				// detect presence of first indicator again
			{
				stop();
				break;
			}
		}
		back_mm(50);								// aligns with first indicator
		color_detect();
		IA1=color;							
		judge_order(IA1,IA2);						
		if (current_room==5)						// if last room. Move forward to get in same position.
			forward_mm(620);
	}
	
	timer5_init();
	if (current_room!=5)
	{
			back();
			velocity(251,255);
			while (1)
			{
				if (print_line_sensor()==111)		// detect line perpendicular to bot.
				{
					stop(); 
					break;
				}
			}
		
		// aligns bot to starting postion
		
		forward_mm(65);								
		right();
		velocity(119,120);
		_delay_ms(300);
		turn_on_line('r');
		back_mm(50);
		find_line();
	}	
	else											// if last room
	{	
		PORTH= PORTH & 0xCF;						//turn off color sensor vcc and servo3
		timer1_init();
		servo3_pin_config();
		servo_3(150);								//rotation angle for color sensor servo motor to close
		_delay_ms(1000);
		servo_3_free();
		
													// reach service home
		
		forward_mm(40);								
		find_line();
		follow_line(111);
		forward_mm(72);
	}
	
	if (current_room!=5)							//if not last room repeat function.
	take_order1();
	return;
}

/*
* Function Name: 	Enter_room
*Input:			Room Number as int
*Output:		void
* Logic:		Start position: Service center home.
				End Position:	Room center.
* Example Call:		enter_room(3);
*/
void enter_room(int room)	
{
	if(room!=4)	{		// if room != 4 then go to home 
		print_line_sensor();
		shaft=56;		// Gives approx shaft count for which folllow line 						// excutes
		follow_line(0);		// follow line till line ends
		velocity(200,200);
		forward_mm(630);	// approx distance to home center
		find_line();		
		slow_follow_line(111);	// find the exact center of home
		velocity(150,150);
		if (room!=2)	{	// if not room 4 and 2 then decide left or right turn for 						// room 1 and room 3
			forward_mm(75);	// forward to align the bot on center after 							// turning
			if(room==1)
				left();
			else
				right();
			velocity(150,150);
			_delay_ms(400);
					// after initiating turn, continues till line_conf==10 i.e.	 					// turns exactly
			velocity(100,100);
			print_line_sensor();	
			while (line_conf!=10)
				print_line_sensor();
			stop();
		}		
		shaft=18;		// slow follow line to end  of home
		slow_follow_line(0);
		forward();			//start moving forward till ShaftCountLeft <= 76
		velocity(250,250);
		ShaftCountLeft=0;
		while(ShaftCountLeft<=76);	// 76 shaft count aprroximately 40 cm
		stop();
		velocity(150,150);
		left_degrees(90);		// turn left
	}
	else	{ 				// if room number 4 then no need to go to 	
		shaft=56;
		follow_line(0);
		velocity(200,200);
		forward_mm(295);		// move about 30 cm and then turn
		right_degrees(90);
	}
						// now follow line to center of room
	forward_mm(180);
	find_line1();				// searches for line
	follow_line(111);			// follow line till center
	velocity(150,150);
	stop();
	forward_mm(50);
}


/*
*
* Function Name: 	dump_garbage
* Input: 			room (int) - Room number
* Output: 			void
* Logic: 			Initializes buzzer_pin by setting output as 0 then turning off buzzer  /////////////////
* Example Call:		dump_garbage(1);  ////////////////////////
*
*/
void dump_garbage(int room)
{
	char turn='r';
	int garbage=1;
	//dumping garbage will always initiate from cross inside the room i.e. room home
	if(ADC_Conversion(9)-ADC_Conversion(13)>15)
	turn='r';
	//garbage is on left side
	else if(ADC_Conversion(13)-ADC_Conversion(9)>15)
	turn='l';					//garbage is on right
	else
	{
		turn='r';
		garbage=0;					// no garbage
	}
	if (garbage==1)
	{
		garbage_rank++;
		position='D';					// for retruning back
	}
	else
	position='S';						//for initiating another service pickup
	velocity(150,150);
	forward_mm(21);//was 25
	turn_on_line(turn);					// turn towards free position
	forward_mm(40);						// move ahead to place service
	clip_open();

	back_mm(40);
	if (garbage!=0)
	{
		if(turn=='r'){
			turn='l';
			velocity(200,200);
		left_degrees(90);}		// for fast turning
		else{
			turn='r';
			velocity(200,200);
		right_degrees(90);}			// for fast turning
		turn_on_line(turn);			// turn slowly the rest of the 90 degree turn
		forward_mm(35);
		clip_close();				// pick up garbage
		//_delay_ms(1000);
		back_mm(35);
		turn_on_line(turn);			// align towards the line entering the room
	}
	else
	turn_on_line(turn);
	shaft=56;
	follow_line(0);						// follow line out of the room
	velocity(150,150);
	
	forward_mm(260);
	
	if(sorted_rooms[count1+1]==0 && garbage==0)	// if next room is 'Do Not Disturb' and 'No gargabe is present' go to home
	{
		right_degrees(90);
		forward_mm(300);
		find_line();
		slow_follow_line(111);					// follow line till the center of the home
		buzzer_on();							// sound buzzer for 5 seconds
		_delay_ms(5000);
		buzzer_off();
		return;
	}
	
	if (room!=4)								// if we were not in fourth room
	{
		
		// go to center
		right_degrees(90);
		forward_mm(300);
		find_line1();
		slow_follow_line(111);
		velocity(100,100);
		forward_mm(75);
		
		// turning left or right depending on the room number
		if(room!=2)
		{
			if(room==1)
			turn_on_line('r');
			else
			turn_on_line('l');
		}
		
		//exiting
		find_line();
		shaft=18;
		slow_follow_line(0);
		_delay_ms(200);
		velocity(200,200);
		forward_mm(650);
	}
	else					// if room number was 4. Going to service region.
	{
		left_degrees(90);
		forward_mm(260);
	}
	
	// going to service region home
	find_line();
	follow_line(111);
	forward_mm(60);
	
	if (garbage==1)			// if garbage is present
	turn_on_line('r');	// turn right to dump it.
	else
	return;				// else return to delivery to service other rooms
	ShaftCountLeft=0;
	// following shaftCountLeft to reach the dumping zone
	while (1)
	{
		follow_line(111);
		if(ShaftCountLeft>=110)
		break;
	}
	// deciding garbage_rank and thus the placement of the garbage
	if (garbage_rank==1)
	{
		forward_mm(50);
		left_degrees(45);
		clip_open();
		left_degrees(110);
	}
	else if (garbage_rank==2)
	{
		forward_mm(50);
		right_degrees(45);
		clip_open();
		right_degrees(120);
	}
	else if (garbage_rank==3)
	{
		left_degrees(30);
		clip_open();
		left_degrees(120);
		find_line();
	}
	else if (garbage_rank==4)
	{
		right_degrees(30);
		clip_open();
		right_degrees(100);
		find_line();
		
	}
	find_line();
	if (garbage==1 && sorted_rooms[count1+1]==0)
	{
		return_home();
		return;
	}
	
	return;
}

/*
*
* Function Name: 	buzzer_pin_config
* Input: 			void
* Output: 			void
* Logic: 			Initializes buzzer_pin by setting output as 0 then turning off buzzer
* Example Call:		buzzer_pin_config();
*
*/

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}





/*
*
* Function Name: 	buzzer_on
* Input: 			void
* Output: 			void
* Logic: 			Starts the buzzer by switching on the PC3 pin
* Example Call:		buzzer_on();
*
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;  	// To switch on the PC3 pin
	PORTC = port_restore;					// Switches on the PC3 pin
}


/*
*
* Function Name: 	buzzer_off
* Input: 			void
* Output: 			void
* Logic: 			Stops the buzzer by switching off the PC3 pin
* Example Call:		buzzer_off();
*
*/
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;  // To switch off the PC3 pin
	PORTC = port_restore;				 // Switches off the PC3 pin
}


/*
*
* Function Name: 	GPIO_pin_config
* Input: 			void
* Output: 			void
* Logic: 			Sets output to 4 GPIO pins and they are set true
* Example Call:		GPIO_pin_config();
*
*/
void GPIO_pin_config(void)
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


/*
*
* Function Name: 	find_line
* Input: 			void
* Output: 			void
* Logic: 			Sweeps some area to find the expected line
* Example Call:		find_line();
*
*/
void find_line(){
	print_line_sensor();
	if(line_conf==0)
	{
		ShaftCountRight=0;
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
	return;
}




/*
*
* Function Name: 	slow_follow_line
* Input: 			RqrdLineConf (int) - Required Configuration to stop following line
* Output: 			void
* Logic: 			Moves forward (slow speed) and adusts speed of the wheels so that the bot remains on the line. This is done till the required line configuration is detected
* Example Call:		slow_follow_line(111);
*
*/
void slow_follow_line(int RqrdLineConf)
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

/*
*
* Function Name: 	follow_line
* Input: 			RqrdLineConf (int) - Required Configuration to stop following line
* Output: 			void
* Logic: 			Moves forward (high speed) and adusts speed of the wheels so that the bot remains on the line. This is done till the required line configuration is detected
* Example Call:		follow_line(111);
*
*/
void follow_line(int RqrdLineConf)
{
	int last_line_conf=0;
	ShaftCountRight=0;
	print_line_sensor();
	forward();
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
			else if(ShaftCountRight>=shaft)
			break;
		}
	}
	stop();
	velocity(200,200);
	return;
}

/*
*
* Function Name: 	color_sensor_scaling
* Input: 			void
* Output: 			void
* Logic: 			Sets particular power to the color sensor
* Example Call:		color_sensor_scaling();
*
*/
void color_sensor_scaling()
{
	PORTD = PORTD | 0x10; //set S0 high
	PORTD = PORTD | 0x20; //set S1 high
}

/*
*
* Function Name: 	motion_pin_config
* Input: 			void
* Output: 			void
* Logic: 			Configures motion pins for the DC motors and the servo motors.
* Example Call:		motion_pin_config();
*
*/
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;  	// A -> DC motors pins
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   	// L -> servo motor pins
	PORTL = PORTL | 0x18;
}
/*
*
* Function Name: 	color_detect
* Input: 			void
* Output: 			char
* Logic: 			Returns the bot to home after servicing all rooms/////////////////
* Example Call:		color_detect();
*
*/
void color_detect() 
{
	red_read();									//read red
	_delay_ms(100);
	green_read();								//read green
	_delay_ms(100);
	blue_read();								//read blue
	_delay_ms(100);
	
	if(red<threshold+40 && green<threshold && blue<threshold)		// 'K' is for black. When all three pulses are less than threshold value.
	color = 'K';
	
	// Decides color on the basis of the strength of the pulse.
	
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
			color = 'E';									// 'E' means error in color detection. If 'E' is encountered. color detction is repeated.
			color_detect();
		}
	}
}

/*
*
* Function Name: 	adc_pin_config
* Input: 			void
* Output: 			void
* Logic: 			Initializes ADC channel pins by setting 0 to all the channel pins
* Example Call:		adc_pin_config();
*
*/
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

/*
*
* Function Name: 	adc_init
* Input: 			void
* Output: 			void
* Logic: 			Initilizes ADCs pins
* Example Call:		adc_init();
*
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


/*
*
* Function Name: 	ADC_Conversion
* Input: 			Ch (unsigned char) - Channel Number
* Output: 			unsigned char - integer value between 0 and 255, as a hex value.
* Logic: 			The channel number is converted to its corresponding Analog Value
* Example Call:		if(ADC_Conversion(2) > 32);
*
*/
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


/*
*
* Function Name: 	color_sensor_pin_config
* Input: 			void
* Output: 			void
* Logic: 			Initializes color sensor pins
* Example Call:		color_sensor_pin_config();
*
*/
void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}

/*
* Function Name:	pickup_service_dumping_section
* Input:			character value informing about color of service
* Output:		void
* Logic:			Picking up service after dumping garbage 
			End position:	service home.
* Example call:		pickup_service_dumping_section('G');
*/	

void pickup_service_dumping_section(char current_service)
{
	int cross, tempv=0, retc=75, shaftc=0;		
						// shaftc : stores apporx shaft count to move for different colors
						//retc:	shaft count fot returning to service home
	if (current_service=='R') // if service required is red
		{		
		shaftc=132;				
		follow_line_shaft(shaftc);	// follow line till shaftc counts
		follow_line(111);		// then find the center for the service
		}		
	else if	(current_service=='B')		// if service required is blue	
	{		shaftc=206;
			follow_line_shaft(shaftc);
			retc=123;;
			follow_line(111);
	}
	else if(current_service=='G')	// if service required is green
		{						
		slow_follow_line(111);
		}
	velocity(100,100);
	forward_mm(62);		// move forward to align after turning
	stop();	
					//determine whether the service is on left or right
	char turn;
	if ((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) 
		turn='r';
	else 
		turn='l';
	turn_on_line(turn);		//turn accordingly
	clip_close();			//use servo to pickup
					//go to service line
	if (current_service=='G') 
		if (turn=='r')
			turn_on_line('l');
		else 
			turn_on_line('r');
	else 
		turn_on_line(turn);

	follow_line_shaft(retc);		// follow line till return count shaft
	if(current_service=='G') 	// if service is green turn left else turn right to enter the 					// arena
		turn_on_line('l');
	else 
		turn_on_line('r');
	find_line();			
	return;
}


/*
*
* Function Name: 	turn_on_line(char)
* Input: 			char
* Output: 			void
* Logic: 			turns the bot according to input character till the middle WL sensor detects black
* Example Call:		turn_on_line('l');
*
*/
void turn_on_line(char direction)
{
	
	if(direction=='r')		//if the direction is 'r'which means right direction
	{
		velocity(200,200);
		right_degrees(45);	//turn by 45(certain) degrees to leave the current line
		velocity(100,100);	//decrease	the speed
		right();			//motion set
	}
	else{
		velocity(200,200);
		left_degrees(45);	//turn by 45(certain) degrees in left direction to leave the current line
		velocity(100,100);	//decrease the speed
		left();
	}
	while(1)
	if(ADC_Conversion(2)>32)	//if center wl sensor detects black color  break the while loop
	break;						//break the while loop
	stop();					//stop the bot
	return;
}


/*
*
* Function Name: 	clip_close
* Input: 			void
* Output: 			void
* Logic: 			Returns the bot to home after servicing all rooms///////////////////////////
* Example Call:		clip_close();
*
*/
void clip_close(void) {
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

/*
*
* Function Name: 	clip_open
* Input: 			void
* Output: 			void
* Logic: 			Returns the bot to home after servicing all rooms/////////////////////////////
* Example Call:		clip_open();
*
*/
void clip_open(void) {
	for (int i=0;i<181;i++)
	{
		servo_2(i);
		servo_1(180-i);
		
	}
	_delay_ms(800);
	servo_1_free();
	servo_2_free();
}

/*
*
* Function Name: 	judge_order
* Input: 		2 characters corresponding to the colours of the service request panels - room1 and room2
* Output: 		Return the value of order
* Logic: 		Assigns values to the array pref and orders and calculates using the inputs given to the array
* Example Call:		judge_order('G','K')
*
*/
char judge_order(char room1,char room2)
{
	char order1;
	if (room1 != 'K')				//first indicator is not black
	{
		if (room1 == room2)			//both have same color i.e. VIP room
		{
			order1 = room1;
			orders[current_room-1] = room1;
			pref[current_room-1] = 4; 	//highest pref of 4 given to the room
		}
		else if (room2 == 'K')
		{
			
			order1=room1;
			orders[current_room-1] = room1;
			if(room1=='G') pref[current_room-1]=3; 	//if its green then a pref of 3 is given
			else if(room1=='R') pref[current_room-1]=2;	//if its red then a pref of 2 is given

			else if(room1=='B') pref[current_room-1]=1;	//if its blue then a pref of 0 is given

		}
		else
		order1 = 'E';
	}
	
	else
	{
		if (room2 == 'K')
		{
			order1 = 'N';   	 //N for Do Not Disturb Room
			orders[current_room-1] = 'N';
			pref[current_room-1] = 0;
		}
		else
		{
			order1 = room2;
			orders[current_room-1] = room2;
			if(room2=='G') pref[current_room-1]=3;	//similar to above comments the pref value is decided according to the value of color
			else if(room2=='R') pref[current_room-1]=2;
			else if(room2=='B') pref[current_room-1]=1;
		}
	}
	return order1;
}

/*
*
* Function Name: 	sort_orders
* Input: 			void
* Output: 			void
* Logic: 		Sorts the rooms according to their prefrence order. i.e(the corresponding values in pref appay)
* Example Call:		sort_orders();
*
*/
void sort_orders()
{
	int max=0; 		//max variable stores the max prefrence values in array
	int max_room=0;	//this variable stores the room no. of the room with the highest prefrence
	for(int j=1;j<5;j++)		//sort the rooms according to the prefrence value
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

/*
* Function Name:	pickup_service_Shome
* Input:			character value telling which service to be picked
* Output:		void
* Logic:			Picking up service from service center.
			The center point of the two wheels is exactly on service home.
			Takes color of service and picks it up by using follow_line_shaft.
* Example Call:		pickup_service_Shome('R');
*/
void pickup_service_Shome(char current_service)	{
	char turn;
	if(current_service=='G')				// if service is green turn right AND follow line to reach the service 
	{
		turn_on_line('r');
		follow_line(111);
		velocity(150,150);
		forward_mm(60);
		if((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) // check whick side the service is present
			turn='r';
		else 
			turn='l';
		turn_on_line(turn);
		clip_close();				// pickup service
		turn_on_line(turn);
		follow_line_shaft(75);			// reach near service center
		turn_on_line('l');			// turn left
	}
	else						// if service if red or blue
	{
		turn_on_line('l');
		follow_line(111);			// reach center of red
		if(current_service=='B')			// if service is blue
		{	
			velocity(200,200);
			forward();			// move forward and follow line to center of blue service
			_delay_ms(300);
			follow_line(111);
		}
		velocity(150,150);
		forward_mm(60);			// move forward to align bot after turn
		if((int)ADC_Conversion(13)-(int)ADC_Conversion(9)>30) //check service is left or right 
			turn='r';
		else 
			turn='l';
		turn_on_line(turn);				// turn towards service
		clip_close();					// pickup service
		turn_on_line(turn);				
		follow_line(111);			// should reach service home if service is red else center of red
		if(current_service=='B')		// if service is blue move forward to reach service home
		{
			velocity(200,200);
			forward();
			_delay_ms(300);
			follow_line(111);
		}
		velocity(150,150);
		forward_mm(60);		
		turn_on_line('r');		// turn towards arena
	}
}

/*
*
* Function Name: 	delivery();
* Input: 		void
* Output: 		void
* Logic: 		it uses arrays sorted_rooms[],orders[] and functions enter_room(int room), piickup_service functions and dump_garbage(int room)
				and loops it so that it keeps on executing them in a cycle till there is no more room left for service 
* Example Call:		delivery();
*
*/
void delivery(){	
	//position can be only dumping section or service home
	//for service home position = S
	//for dumping area position = D	
	position='S';
		for(count1=1;count1<5;count1++)
		{
			int room=sorted_rooms[count1];						// sorted_rooms has room number of the room to be seviced	
			if(room!=0)											// room is not 'Do Not Disturb'
			{
				char service=orders[room];						// orders has the color of the service
				if(position=='D')
				pickup_service_dumping_section(service);		//bot has picked up the service and came to service and facing to the center
				else
				pickup_service_Shome(service);
				enter_room(room);		//bot will enter and stop at the room center where line_conf=111
				dump_garbage(room);
			}
		}		//it will 1detect the garbage, put the service at empty space and pick up the garbage and dump it and wait at dumping section otherwise home
}


void follow_line_shaft(int shaftc){
	forward();
	velocity(200,200);
	int last_line_conf=0;
	ShaftCountLeft=0;
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
		if(ShaftCountLeft>=shaftc)// | ShaftCountLeft>=shaft)   ////////////////////////////////////////////////////////
			break;
		
	}
	stop();
	velocity(200,200);
	return;
}
void return_home()
{
	follow_line(111);
	velocity(100,100);
	forward_mm(62);
	stop();
	follow_line_shaft(80);
	turn_on_line('l');
	shaft=56;
	follow_line_shaft(shaft);
	velocity(200,200);
	forward_mm(640);
	find_line();
	follow_line(111);
	stop();
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	exit(0);
}

void find_line1(){
	print_line_sensor();
	if(line_conf==0)
	{
		ShaftCountRight=0;
		left();
		//right();
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
		right();
		//left();
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
		left();
		//right();
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
	return;
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

/*
*
* Function Name: 	init_devices();
* Input: 		void
* Output: 		void
* Logic: 		 it initializes all the ports and configures the various sensors and actuators
* Example Call:		init_devices();
*
*/
void init_devices(){
	cli();											//Clears the global interrupt
	motion_pin_config();							//robot motion pins config
	left_encoder_pin_config();					//initializes left_encoder
	right_encoder_pin_config();					//initializes right_encoder
	color_sensor_pin_config();
	GPIO_pin_config();							//GPIO pins config for LEDs to glow
	buzzer_pin_config();
	adc_pin_config();
	timer5_init();								//PWM for velocity of bot or DC motors
	color_sensor_pin_interrupt_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	color_sensor_scaling();
	sei();										// Enables the global interrupt
}

/*
*
* Function Name: 	int main();
* Input: 			void
* Output: 			void
* Logic: 			it executes the theme solution with the use of take_order1(),delivery and sort_rooms()
* Example Call:		int main();
*
*/
int main()
{
	init_devices();
	PORTH= PORTH | 0x10;								 //turn on color sensor Vcc
	take_order1();										//detect color indicators of all the rooms
		servo1_pin_config();							//Configure PORTB 5 pin for servo motor 1 operation
		servo2_pin_config();							//Configure PORTB 6 pin for servo motor 2 operation
	sort_orders();
	delivery();	
	stop();				
	while (1);
}
