/**
 * Header shit
 */
 #include "main.h"

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	gpioSetup();
	Clock_Init_48MHz();
	adcSetup();
	timerSetup();
	uFlag = 0;

    __enable_interrupts();
	while(1)
	{
	    if(uFlag == 1)
	        updateDisplay();
	    checkUpdates();

	}
}

/**
 * Checks for state of the inputs and based off the lookup table changes the state
 * 
 * Ice Sensor P3.5 (On off switch)
 * Humidity Potentiometer P3.7
 */
void checkUpdates(void)
{
    int temp = setpoint - 5;
    temp = (temp < 0) ? 0 : temp;
    l_hum = (humidity < temp) ? TRUE : FALSE;
    /**
     * If there is iced sensed, prioritize the ice input
     * otherwise check too see if we are currently at high or low humidity based 
     * off the user defined humidity settings.
     */
    if(iceMachineStatus)
    {
        event = e1;
    } else if(l_hum == TRUE)
    {
        event = e2;
    } else
    {
        event = e3;
    }
    current = stateUpdate(current, event);
}

/**
 * Updates the LCD Content
 */
void updateDisplay(void)
{
    LCD_clear();
    LCD_home();
    // First Line
    LCD_print_str("Temp:");
    LCD_goto_xy(7,0);
    LCD_print_udec5(temperature);

    /**
     * On boot the interrupt is triggered. Even with having the __enableInterrupt call
     * after every initialize...
     */
    if(initBoot)
    {
        setpoint = 0;
        iceMachineStatus = 0;
        initBoot = false;
    }
    // Second Line
    LCD_goto_xy(0,1);
    LCD_print_str("Setpnt:");
    LCD_goto_xy(9,1);
    LCD_print_udec3(setpoint);

    // Third Line
    LCD_goto_xy(0,2);
    LCD_print_str("Hum:");
    LCD_goto_xy(6,2);
    LCD_print_udec5(humidity);

    // Fourth Line
    LCD_goto_xy(0,4);
    LCD_print_str("Defrost:");
    LCD_goto_xy(8,4);
    if(iceMachineStatus == ON)
    {
        LCD_print_str("On");
    } else
    {
        LCD_print_str("Off");
    }

    uFlag = 0;
}

/**
 * Initiate GPIO for the system
 */
void gpioSetup(void)
{

	//--------------------------------------------------------------------------
	// Inputs
	//--------------------------------------------------------------------------

	/** 
     * Ice Sensor P3.5 (On off switch)
	 * Room Temperature Potentiometer P3.6
	 * Humidity Potentiometer P3.7
	 * 
	 * Utilizing Pull-up Resistors
	 * When !Pressed == 0 When Pressed == 1
	 * PxDIR = 0 PxREN = 1 PxOUT = 1
	 */
    P3->DIR &= ~(BIT5|BIT6|BIT7);	
	P3->REN |= (BIT5|BIT6|BIT7);	
	P3->OUT |= (BIT5|BIT6|BIT7);
	P3->IE |= BIT5;
	P3->IES |= BIT5;
	NVIC->ISER[1] |= (1<<5);

	/**
	 * Humidity setpoint Up: P1.6 Down: P1.7
	 * P1.6 UP
	 * P1.7 Down
	 * 
	 * Utilizing Pull-up Resistors
	 * When !Pressed == 0 When Pressed == 1
	 * PxDIR = 0 PxREN = 1 PxOUT = 1
	 */
	P1->SEL0 &= ~(BIT1|BIT4);  // select GPIO function on pins
	P1->SEL1 &= ~(BIT1|BIT4);  // select GPIO function on pins
    P1->REN |= (BIT1|BIT4);   // Pull up resistor
    P1->OUT |= (BIT1|BIT4);

    P1->IE |= (BIT1|BIT4); // enable interrupt on P1.1
    P1->IES |= ~(BIT1|BIT4); // falling edge
    NVIC->ISER[1] |= (1<<3);  // enable P1 interrupt
	//--------------------------------------------------------------------------
	// Outputs
	//--------------------------------------------------------------------------

	/**
	 * Fan Control P2.4
	 * Compressor P2.5
	 * 
	 * PxDir = 1 for output
	 */
	P2->DIR |= (BIT4|BIT5);

	 /**
	  * LCD Setup
	  */
	LCD_Config();
	LCD_contrast(48);
 	LCD_goto_xy(0,0);
 	LCD_print_str("Startup...");
}

/**
 * Set ups ADC for A5(P5.0) A3(P5.2)
 */
void adcSetup(void)
{
    /* Enable A/D channels A3 and A5 */
    P5->SEL1 |= (BIT0|BIT2);
    P5->SEL0 |= (BIT0|BIT2);

    ADC14->CTL0 |= ADC14_CTL0_SHT0_7 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_2
            | ADC14_CTL0_ON | ADC14_CTL0_MSC | ADC14_CTL0_CONSEQ_1;

    ADC14->CTL1 |= ADC14_CTL1_RES_2; // 12-bit conversion
    ADC14->CTL1 &= ~ADC14_CTL1_RES_1; // 12-bit conversion
    ADC14->CTL1 |= (1 << ADC14_CTL1_CSTARTADD_OFS); // start at MEM[0]
    ADC14->MCTL[1] |= ADC14_MCTLN_INCH_3; // input on A3
    ADC14->MCTL[2] |= ADC14_MCTLN_INCH_5; // input on A5
    ADC14->IER0 |= (ADC14_IER0_IE1 | ADC14_IER0_IE2); // enable interrupt

    NVIC->ISER[0] |=(1<<24);    // enable NVIC ADC interrupts
}

/** 
 * Setup timer 
 */
void timerSetup(void)
{
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_MC__UP
                       | TIMER_A_CTL_CLR | TIMER_A_CTL_IE;
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[0] = 15999; //CCR[0] is set for an 8th of a seconds triggering time
    NVIC->ISER[0] |= (1<<8);
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
}

/**
 * For this program handles interrupts for the Humidity set-point
 * Setpoint buttons are on the sides of the boards using onboard Buttons.
 */
void PORT1_IRQHandler(void)
{
    int temp = P1->IV;
    if(temp == 4) /* P1.1 -> Up*/
    {
        setpoint -=5;
        setpoint = (setpoint <= 0) ? 0 : setpoint;  // Ensuring no negatives break through
    }

    if(temp == 10) /* P1.4 -> Down */
    {
        setpoint += 5;
        setpoint = (setpoint > 100) ? 100 : setpoint;   // Ensure to not go past 100%
    }
}

/**
 * For this program handles interrupts for the ice sensor
 */
void PORT3_IRQHandler(void)
{
    if(P3->IV == 0x0C)  /* P3.5 */
    {
        /**
         * I wanted to utilize the button as a push for on then push again for off
         * so here is my toggle iceMachineStatus logic.
         */
        iceMachineStatus = (iceMachineStatus == ON) ? OFF : ON;
    }
}

/**
 * Timer interrupt handling the ADC Start conversion
 */
void TA0_0_IRQHandler(void)
{
    // Subsequent conversions enabled
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      // clear interrupt flag
    uFlag = 1;
}

/**
 * Triggered by conversion complete flag
 */
void ADC14_IRQHandler(void)
{
    int temp;
    // Temperature conversion complete flag for MEM[0]
    if(ADC14->IV == 0x0E)
    {
        temp = ADC14->MEM[1];
        // Normalizing temperature from adc to 50->100 degrees
        temperature = ((((double)temp/4095.0)*50.0)+50);
    }
    if(ADC14->IV == 0x10)
    {
        temp = ADC14->MEM[2];
        // Normalizing temperature from adc to 0->100%
        humidity = (((double)temp/4095.0)*100);
    }
}

/**
 * Provides direction for the unit once we head into the idle state
 * 1. Turns off the compressor (P2.5)
 * 2. Turns off the fan        (P2.4)
 */
void idleAction(void)
{
    P2->OUT &= ~(BIT4|BIT5);
}

/**
 * Provides direction for the unit once we head into the on state
 * 1. Turns on the compressor (P2.5)
 * 2. Turns on the fan        (P2.4)
 */
void onAction(void)
{
    P2->OUT |= (BIT4|BIT5);
}

/**
 * Provides direction for the unit once we head into the defrost state
 * 1. Turns off the compressor (P2.5)
 * 2. Turns on the fan        (P2.4)
 */
void defrostAction(void)
{
    P2->OUT |= (BIT4);
    P2->OUT &= ~(BIT5);
}
