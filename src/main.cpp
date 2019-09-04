/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include <string>
#include "Parser.h"
// TODO: insert other definitions and declarations here

int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

    // TODO: insert code here
    int ch;
    std::string GCode;
    Parser parser;
    while(1) {
        if ((ch = Board_UARTGetChar()) != EOF) {
        	if (ch == 10) {
        		parser.Parse(GCode);
        		Board_UARTPutSTR(parser.output.c_str());
        		Board_UARTPutSTR("\r\n");
        		Board_UARTPutSTR("OK\r\n");
        		GCode = "";
        	}
        	else GCode += (char) ch;
        }
    }
    return 0 ;
}
