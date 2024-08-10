#include <lpc17xx.h>
#include <stdio.h>

#define PRESCALE (3000-1) // Assuming 25MHz PCLK for Timer

#define TRIG (1 << 0) // P0.0 for Ultrasonic Sensor TRIG
#define ECHO (1 << 1) // P0.1 for Ultrasonic Sensor ECHO
#define STEPPER_MOTOR (0x03C00000) // P1.22, P1.23, P1.24, P1.25 for Stepper Motor
#define BUZZER (1 << 25) // P3.25 for Buzzer
#define ULTRASONIC_THRESHOLD 10.0 // Threshold value for Ultrasonic Sensor in centimeters

#define RS_CTRL  0x02000000  //P3.25
#define RW_CTRL  0x04000000  //P3.26
#define EN_CTRL  0x10000000  //P4.28
#define DT_CTRL  0x00F00000  //P1.20 to P1.23



unsigned char str[100];
unsigned char flag=0; 
float distance;
unsigned long int temp1=0, temp2=0 , count1 =0 ,count2 = 0;
unsigned char Msg1[16],Msg2[16];
unsigned char int3_flag=0;


void initUART0(void);
void send_message(char ch);
void send_string(const char *str);

void Timer0_Init(void);
void delayUS(unsigned int microseconds);
void startTimer0(void);
unsigned int stopTimer0(void);
void delayMS(unsigned int milliseconds);

void initUltrasonicSensor(void);
float measureDistance(void);

void initStepperMotor(void);
void rotateStepperMotor(void);

void initBuzzer(void);
void activateBuzzer(void);

void initRTC(void);
void setRTC(void);
void getRTC(void);
void delay(void);

void lcd_init(void);
void wr_cn(void);
void clr_disp(void);
void delay_lcd(unsigned int);
void lcd_com(void);						   
void wr_dn(void);
void lcd_data(void);
void clear_ports(void);
void lcd_puts(unsigned char *);
void EINT3_IRQHandler(void);
void interrupt_init(void);

int main(void)
{

	
	lcd_init();				//initialise LCD
	delay_lcd(3200);

	temp1 = 0x80;			//1st message on LCD 1st line
	lcd_com();
	delay_lcd(800);	
	clr_disp();								// above registers, bit0-EINT0, bit1-EINT1, bit2-EINT2,bit3-EINT3	

    initUART0(); // Initialize UART0 for communication
    initUltrasonicSensor(); // Initialize Ultrasonic Sensor
    initStepperMotor();     // Initialize Stepper Motor
    initBuzzer();           // Initialize Buzzer
    initRTC();              // Initialize RTC
	interrupt_init();
	NVIC_EnableIRQ(EINT3_IRQn);			//core_cm3.h

    send_string("System Initialized\n\r\n\n");
	setRTC(); // Set RTC time
    while (1)
    {
	    getRTC(); // Get RTC time
        distance = measureDistance();
		sprintf(str,"Distance=%0.2f cm\n\r\n\n", distance);
		send_string(str);
	
	    if (distance <= ULTRASONIC_THRESHOLD && !int3_flag)// when interrupt is not pressed and ultra sonic sensor measures distance accordingly
	        {
				temp1 = 0x80;			//1st message on LCD 1st line
	         	lcd_com();
	            send_string("Vehicle Detected\n\r\n\n");
	            rotateStepperMotor(); // Rotate Stepper Motor
	            activateBuzzer();      // Activate Buzzer

				count1++;
			    sprintf(Msg1,"VEHICLE=%d",count1);		//when flag is '0' off the LED	// When interrupt is pressed off the led 
				lcd_puts(Msg1);
				delay_lcd(1000);
				temp1 = 0x80;			//1st message on LCD 1st line
	         	lcd_com();
	        }
             else if (int3_flag) 
	   {
		    int3_flag = 0x00;
		    if (flag == 0x00)
			 {
		        temp1 = 0xC0;
		        lcd_com();
		
		        // Emergency case
		        LPC_GPIO2->FIOCLR = 0x00000001;
		        send_string("EMERGENCY CASE: Vehicle Detected\n\r\n\n");
		        rotateStepperMotor(); // Rotate Stepper Motor
		        activateBuzzer();     // Activate Buzzer
		
		        count2++;
		        sprintf(Msg2, "VIP-VEHICLE=%d", count2);
		        temp1 = 0xC0;
		        lcd_com();
		        lcd_puts(Msg2);
	
		        delay_lcd(1000);
		
		        flag = 0xFF;
		
			//	LPC_GPIO0->FIOPIN = 0;
		        // Wait until switch (P0.26) is pressed
		        while ((LPC_GPIO0->FIOPIN & (1 << 26)) != 0) ;
			
	
	        // Wait until the switch is released
	        	while ((LPC_GPIO0->FIOPIN & (1 << 26)) == 0) ;
			
	            // Continue waiting for switch release
				LPC_GPIO3->FIOPIN &= ~BUZZER; // Deactivate the Buzzer
		      }
       		}

        else
	        {
				LPC_GPIO2->FIOSET = 0x00000001;			//when flag is FF on the LED	 // when interrupt is not pressed on the led 
	            send_string("Vehicle Not Detected\n\r\n\n");
				flag = 0x00;
	        }

        delayMS(1000); // Delay for 1 second
    }
}
void EINT3_IRQHandler(void)
{
	int3_flag = 0xff;
	LPC_SC->EXTINT = 0x00000008;	//cleares the interrupt

}

void initUART0(void) //PCLK must be = 25Mhz!
{
	LPC_PINCON->PINSEL0 |= (1<<4) | (1<<6); //Select TXD0 and RXD0 function for P0.2 & P0.3!
	LPC_UART0->LCR = 0x83 ; /* 8 bits, no Parity, 1 Stop bit & DLAB set to 1  */
	LPC_UART0->DLL = 12;
	LPC_UART0->DLM = 0;
	LPC_UART0->FCR |= 0x07;
	LPC_UART0->FDR = (15<<4) | 2; /* MULVAL=15(bits - 7:4) , DIVADDVAL=2(bits - 3:0)  */
	LPC_UART0->LCR = 0x03;

}

void send_message(char ch)
{
	while(!(LPC_UART0->LSR & 0x20));
	LPC_UART0->THR = ch;
}

void send_string(const char *str)
{
	while(*str!='\0')
	{
		send_message(*str++);	
	}
}

void Timer0_Init(void)
{
	LPC_TIM0->CTCR = 0x0000;		   //rising edge and 00 therefore in timer mode
	LPC_TIM0->PR = PRESCALE;			// put prescaler value in PR register
	
	LPC_TIM0->TCR = 0x02;				// RESET the timer once befor starting
}

void delayUS(unsigned int milliseconds)
{
	LPC_TIM0->TCR = 0x02;	  // RESET the timer once befor starting
	LPC_TIM0->TCR = 0x01;	 //Start timer

	while(LPC_TIM0->TC < milliseconds);
	
	LPC_TIM0->TCR = 0x00;
}


void startTimer0()
{
   	LPC_TIM0->TCR = 0x02;	  // RESET the timer once befor starting
	LPC_TIM0->TCR = 0x01;	 //Start timer
}

unsigned int stopTimer0(void)
{
	LPC_TIM0->TCR = 0x00; //Disable timer
	return LPC_TIM0->TC;
}


void delayMS(unsigned int milliseconds) //Using Timer0
{
	delayUS(milliseconds * 1000);
}

void initUltrasonicSensor(void)
{
    // Configure TRIG (P0.0) as output
    LPC_GPIO0->FIODIR |= TRIG;

    // Configure ECHO (P0.1) as input
    LPC_GPIO0->FIODIR &= ~ECHO;

    // Clear TRIG initially
    LPC_GPIO0->FIOCLR |= TRIG;
}

float measureDistance(void)
{
    unsigned int echoTime;
    float distance;

    // Trigger the Ultrasonic Sensor
    LPC_GPIO0->FIOPIN |= TRIG;
    delayUS(10); // Wait for 10 microseconds
    LPC_GPIO0->FIOCLR |= TRIG;

    // Wait for a HIGH on ECHO pin
    while (!(LPC_GPIO0->FIOPIN & ECHO));

    // Start the timer
    startTimer0();

    // Wait for a LOW on ECHO pin
    while (LPC_GPIO0->FIOPIN & ECHO);

    // Stop the timer and get the elapsed time
    echoTime = stopTimer0();

    // Calculate distance in centimeters using the speed of sound (approx. 0.0343 cm/us)
    distance = (0.0343 * echoTime) / 2;

    return distance;
}

void initStepperMotor(void)
{
    // Configure P1.22, P1.23, P1.24, P1.25 as output for the Stepper Motor
    LPC_PINCON->PINSEL3 &= ~(0xFF << 12);
    LPC_GPIO1->FIODIR |= STEPPER_MOTOR;

    // Initial state: Set P1.22 high and others low
    LPC_GPIO1->FIOPIN = 0x00400000;
}

void rotateStepperMotor(void)
{
 
      //LPC_GPIO1->FIOPIN = 0x00400000 ;		  // for 90 degree only rotate from 1 to 2
	  //LPC_GPIO1->FIOCLR = STEPPER_MOTOR;
	  delayMS(100);

	  LPC_GPIO1->FIOPIN = 0x00800000 ;
	  //LPC_GPIO1->FIOCLR = STEPPER_MOTOR;
	  delay();
	   delayMS(100);

	  LPC_GPIO1->FIOPIN = 0x01000000 ;
	  //LPC_GPIO1->FIOCLR = STEPPER_MOTOR;
	  delay();
	   delayMS(100);

	  LPC_GPIO1->FIOPIN = 0x02000000 ;
	  //LPC_GPIO1->FIOCLR = STEPPER_MOTOR; 
	  delay(); 
	  delayMS(100);
	  LPC_GPIO1->FIOPIN = 0x00400000; 
	  delayMS(100);                      // Add a delay for the motor to move to the next step
}

void initBuzzer(void)
{
    // Configure P3.25 as output for the Buzzer
    LPC_PINCON->PINSEL7 &= ~(0x3 << 18);
    LPC_GPIO3->FIODIR |= BUZZER;

    // Initial state: Clear P3.25 (Buzzer off)
    LPC_GPIO3->FIOPIN &= ~BUZZER;
}

void activateBuzzer(void)
{
    // Activate the Buzzer
    LPC_GPIO3->FIOPIN |= BUZZER;
    delayMS(100); // Add a delay for the buzzer to activate
    LPC_GPIO3->FIOPIN &= ~BUZZER; // Deactivate the Buzzer
	delayMS(100);	
}

void delay()
{
	int i;
	  for(i=0;i<2000;i++);	
}

void initRTC(void)
{
    // Enable power for RTC
    LPC_SC->PCONP |= (1 << 9);

    // Clear RTC control register
    LPC_RTC->CCR = 0;

    // Initialize RTC registers
    LPC_RTC->CIIR = 0; // Disable all periodic interrupts
    LPC_RTC->AMR = 0;  // Alarm mask register, no alarm
    LPC_RTC->ILR = 0;  // Clear interrupts

    // Set RTC to 24-hour mode
    LPC_RTC->CCR |= (1 << 4);

    // Enable RTC
    LPC_RTC->CCR |= (1 << 0);
}

void setRTC(void)
{
    // Set the RTC time to a predefined value
    LPC_RTC->SEC = 0;      // Seconds
    LPC_RTC->MIN = 1;      // Minutes
    LPC_RTC->HOUR = 12;    // Hours
    LPC_RTC->DOM = 1;      // Day of month
    LPC_RTC->DOW = 5;      // Day of week (Friday)
    LPC_RTC->DOY = 1;      // Day of year
    LPC_RTC->MONTH = 1;    // Month
    LPC_RTC->YEAR = 2024;  // Year

    // Start the RTC
    LPC_RTC->CCR |= (1 << 0);

    // Clear the interrupt flag
    LPC_RTC->ILR |= (1 << 0);
}

void getRTC(void)
{
	char time[200];
    // Read the RTC values
    unsigned int sec = LPC_RTC->SEC;
    unsigned int min = LPC_RTC->MIN;
    unsigned int hour = LPC_RTC->HOUR;
    unsigned int dom = LPC_RTC->DOM;
    unsigned int dow = LPC_RTC->DOW;
    unsigned int doy = LPC_RTC->DOY;
    unsigned int month = LPC_RTC->MONTH;
    unsigned int year = LPC_RTC->YEAR;

    // Print the RTC values
    sprintf(time,"RTC Time: %02u:%02u:%02u %02u/%02u/%04u\n\r\r\n\n", hour, min, sec, dom, month, year);
	send_string(time);
}
void interrupt_init(void)
{
	LPC_PINCON->PINSEL4 = 0x04000000;	//P2.13 as EINT3 and P2.0 - GPIO for LED1
	LPC_GPIO2->FIODIR = 0x00000001;		//P2.0 is assigned output
	LPC_GPIO2->FIOSET = 0x00000001;	//Initiall LED is kept on
	
	LPC_SC->EXTINT = 0x00000008;		//writing 1 cleares the interrupt, get set if there is interrupt
	LPC_SC->EXTMODE = 0x00000008;		//EINT3 is initiated as edge senitive, 0 for level sensitive
	LPC_SC->EXTPOLAR = 0x00000000;		//EINT3 is falling edge sensitive, 1 for rising edge
}

//lcd initialization
void lcd_init()
{
	/* Ports initialized as GPIO */
    LPC_PINCON->PINSEL3 &= 0xFFFF00FF;  //P1.20 to P1.23
	LPC_PINCON->PINSEL7 &= 0XFFF3FFFF;  //P3.25
    LPC_PINCON->PINSEL7 &= 0xFFCFFFFF;  //P3.26
	LPC_PINCON->PINSEL9 &= 0xFCFFFFFF;  //P4.28

	/* Setting the directions as output */
    LPC_GPIO1->FIODIR |= DT_CTRL;	// data lines - P1.20 to P1.23
	LPC_GPIO3->FIODIR |= RS_CTRL;	// RS - P3.25
    LPC_GPIO3->FIODIR |= RW_CTRL;	// RW - P3.26
	LPC_GPIO4->FIODIR |= EN_CTRL;	// P4.28 
        
    clear_ports();
	delay_lcd(3200);

	temp2=0x30;		   
	wr_cn();	   
	delay_lcd(30000); 
		
	temp2=0x30;
	wr_cn();
	delay_lcd(30000);	 
		
	temp2=0x30;
	wr_cn();
	delay_lcd(30000);

	temp2=0x20;
	wr_cn();
	delay_lcd(30000);

	temp1 = 0x28;
	lcd_com();
	delay_lcd(30000);
		
	temp1 = 0x0c;		
	lcd_com();
	delay_lcd(800);
	
	temp1 = 0x06;
	lcd_com();
	delay_lcd(800);
	
	temp1 = 0x01;
	lcd_com();
 	delay_lcd(10000);
	
	temp1 = 0x80;
	lcd_com();
	delay_lcd(800);
    return;
}

void lcd_com(void)
{
	temp2= temp1 & 0xf0;
	temp2 = temp2 << 16;				//data lines from 20 to 23
	wr_cn();
	temp2 = temp1 & 0x0f;
	temp2 = temp2 << 20; 
	wr_cn();
	delay_lcd(1000);
    return;
}

// command nibble o/p routine
void wr_cn(void)                        //write command reg
{ 	 
	clear_ports();
	LPC_GPIO1->FIOPIN = temp2;		// Assign the value to the data lines    
	LPC_GPIO3->FIOCLR = RW_CTRL;		// clear bit RW
    LPC_GPIO3->FIOCLR = RS_CTRL;		// clear bit RW
	LPC_GPIO4->FIOSET = EN_CTRL;   	// EN=1
	delay_lcd(25);
	LPC_GPIO4->FIOCLR  = EN_CTRL;		 		// EN =0
    return;
    
 }

// data o/p routine which also outputs high nibble first
// and lower nibble next
 void lcd_data(void)
{             
    temp2 = temp1 & 0xf0;
    temp2 = temp2 << 16;
    wr_dn();
    temp2= temp1 & 0x0f;	
    temp2= temp2 << 20;
    wr_dn();
    delay_lcd(1000);	
    return;
} 

// data nibble o/p routine
void wr_dn(void)
{  	  
	clear_ports();

	LPC_GPIO1->FIOPIN = temp2;			// Assign the value to the data lines    
	LPC_GPIO3->FIOSET = RS_CTRL;		// set bit  RS
	LPC_GPIO3->FIOCLR = RW_CTRL;		// clear bit  RW
	LPC_GPIO4->FIOSET = EN_CTRL;   	// EN=1
	delay_lcd(25);
	LPC_GPIO4->FIOCLR  = EN_CTRL;	// EN =0
    return;
 }

void delay_lcd(unsigned int r1)
{
  	unsigned int r;
  	for(r=0;r<r1;r++);
    return;
}

void clr_disp(void)
{
	temp1 = 0x01;
	lcd_com();
 	delay_lcd(10000);
    return;
}
void clear_ports(void)
{
    /* Clearing the lines at power on */
	LPC_GPIO1->FIOCLR = DT_CTRL; //Clearing data lines
	LPC_GPIO3->FIOCLR = RS_CTRL;  //Clearing RS line
    LPC_GPIO3->FIOCLR = RW_CTRL;  //Clearing RW line
	LPC_GPIO4->FIOCLR = EN_CTRL; //Clearing Enable line
        
    return;
}

void lcd_puts(unsigned char *buf1)
{
    unsigned int i=0;

    while(buf1[i]!='\0')
    {
        temp1 = buf1[i];
     	lcd_data();
		i++;
        if(i==16)
		{
           	temp1 = 0xc0;
			lcd_com();
		}
         
       }
    return;
}