#include "msp.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_clk.h"
#include <stdbool.h>

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0

typedef void (*fp)(void);

typedef enum {s_idle, s_on, s_defrost} StateName;

/**
 * e_1 = ice
 * e_2 = !ice && low hum
 * e_3 = !ice && high hum
 */
typedef enum {e1, e2, e3} Event;

typedef struct
{
    StateName nextState;
    fp action;
} StateElement;


int temperature;
int setpoint;
int humidity;
int iceMachineStatus;

/**
 * uFlag - Update Flag was created due to the seemingly fast update of the adc values
 * Because I was hitting my update function in main very fast the values would bounce around
 * and therefor flash too often. I had setup a timer already for the ADC start conversion, but 
 * then transitioned to multiple sequential conversion mode and so no longer needed the timer.
 * I kept it and triggered an update of the LCD using it. So the timer toggles the flag to 
 * limit LCD updates.
 */
int uFlag;  
bool initBoot = true;
int l_hum;
StateName current = s_idle;
Event event;

/**
 * State Callbacks
 */
void defrostAction(void);
void idleAction(void);
void onAction(void);

void checkUpdates(void);
void gpioSetup(void);
void adcSetup(void);
void timerSetup(void);
void updateDisplay(void);


/**
 * [current State][event]
 * current State is irrelivant, but kept in to show each row being a 
 * different state.
 */
StateElement stateTable [3][3] =
{
    {{s_defrost, defrostAction},{s_idle, idleAction},{s_on, onAction}},
    {{s_defrost, defrostAction},{s_idle, idleAction},{s_on, onAction}},
    {{s_defrost, defrostAction},{s_idle, idleAction},{s_on, onAction}},
};

/**
 * Transitions between states via the state transition lookup table
 */
StateName stateUpdate(StateName current, Event event)
{
    StateElement currentState = stateTable[current][event];
    (*currentState.action)();
    return currentState.nextState;
}

