#include "Button.h"

#define MOUSE_MULT_HIGH 4
#define MOUSE_MULT_LOW  2

// Only four pins with interrupts available on the 2.0
// https://www.pjrc.com/teensy/td_libs_Encoder.html
// AVOID PIN 11
#define V_A 5  // OC2 pin 6
#define V_B 6  // OC2 pin 8
#define H_A 7  // OC1 pin 6
#define H_B 8  // OC1 pin 8


#define BTNS 5 // Button Count
// Short-hand for the various arrays
#define _LEFT       0
#define _MID        1
#define _RIGHT      2
#define _SCROLL     3
#define _DPI        4

byte pattern_order[] = {_SCROLL, _DPI, _LEFT, _MID, _RIGHT};

// Avoid PIN 11 on 2.0
byte btn_pins[] = {
    21,  // L
    20,  // R
    19,  // M
    NULL, // Scroll (unused)
    18   // DPI (or unuse this one and swap NULL/17)
};

volatile unsigned long last_int[] = {
    0,
    0,
    0,
    0,
    0
};

Button btn_scroll(btn_pins[_SCROLL]);
Button btn_dpi(btn_pins[_DPI]);
Button btn_left(btn_pins[_LEFT]);
Button btn_mid(btn_pins[_MID]);
Button btn_right(btn_pins[_RIGHT]);

Button btns[] = {
    btn_left,
    btn_mid,
    btn_right,
    btn_scroll,
    btn_dpi
};

void setup_btns(){
    for(byte i=0; i<BTNS; i++){
        btns[i].begin();
    }
}

void read_btns(){
    for(byte i=0; i<BTNS; i++){
        btns[i].read();
    }
}

#define LAST_EVENT_TIMEOUT 60000
unsigned long last_event = 0;
bool in_pattern = false;
byte pattern_dir = 1;

inline void reset_event(){last_event = millis();}

//Helper for time delays without actually pausing execution
bool TimeElapsed(unsigned long ref, unsigned long wait)
{
	static unsigned long now = 0;
	static bool result;
	result = false;
	now = millis();

	if(now < ref || ref == 0) //for the 50 day rollover or first boot
		result = true;
	else if((now - ref) > wait)
		result = true;

	return result;
}
