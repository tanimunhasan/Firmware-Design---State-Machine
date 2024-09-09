/*
    * A simple state machine to extract button press and release
    * events from button position(open=0 or closed = 1) when polled
    * Includes "do-nothing" transitions for completeness.


*/

#include<stdio.h>

/* Set up the states */
enum buttonState{RELEASED,PRESSED};

/* Create a variable to hold state and initialize */
enum buttonState state = RELEASED;

/*  Executed every time the button is polled - first read
    * the button value and then execute the state machine
*/

unsigned button;
void signalRelease();
void signalPress();

void main()
{
    while(1)
        {

    switch(state){
        case RELEASED:
            if(button == 1){
                state = PRESSED;
                signalPress();
            }
            else{                   ///trigger: open
                                    /// do nothing
            }                       ///
            break;
        case PRESSED:                // PRESSED state
            if(button == 0){         // trigger: open
                state = RELEASED;   // next state
                signalRelease();    // transition action
            }
            else{                   /// trigger: closed
                                    /// do nothing

            }                       ///
            break;
        default:
            break;

                }
        }
}


void signalRelease()
{

}
void signalPress()
{

}
