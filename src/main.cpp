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
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <string>
#include "Parser.h"
#include "DigitalIoPin.h"

void motor(char);

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}

// TODO: insert other definitions and declarations here
static void calibrate(void *pvParameters) {
    DigitalIoPin laser(0, 12, DigitalIoPin::output, true);	//laser
	DigitalIoPin pen(0, 10, DigitalIoPin::output, true);	//pen

    DigitalIoPin ls1(1, 3, DigitalIoPin::pullup, true);	//up
	DigitalIoPin ls2(0, 0, DigitalIoPin::pullup, true);	//down
	DigitalIoPin ls3(0, 29, DigitalIoPin::pullup, true);	//right
	DigitalIoPin ls4(0, 9, DigitalIoPin::pullup, true); //left

	int topNum = 0;
	int rightNum = 0;
	int bottomNum = 0;
	int leftNum = 0;
	int xSize = 0;
	int ySize = 0;
	int backLS = 5;
	int delay = 10;

	laser.write(false);
	pen.write(true);
	//Delay for laser to actually turn off
	vTaskDelay(100);

	//LEFT until limit
	while (ls4.read()==false){
		motor('L');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		motor('R');
		vTaskDelay(delay);
	}
	//UP until limit
	while(ls1.read()==false){
		motor('U');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		motor('D');
		vTaskDelay(delay);
	}

//AT THE TOP LEFT CORNER

	//counting RIGHT till limit
	while(ls3.read()==false){
		topNum++;
		motor('R');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		topNum--;
		motor('L');
		vTaskDelay(delay);
	}
	//counting DOWN till limit
	while(ls2.read()==false){
		rightNum++;
		motor('D');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		rightNum--;
		motor('U');
		vTaskDelay(delay);
	}
	//counting LEFT till limit
	while(ls4.read()==false){
		bottomNum++;
		motor('L');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		bottomNum--;
		motor('R');
		vTaskDelay(delay);
	}
	//counting UP till limit
	while(ls1.read()==false){
		leftNum++;
		motor('U');
		vTaskDelay(delay);
	}
	//back-up to not hit the limit switch
	for(int i=0; i<backLS; i++){
		leftNum--;
		motor('D');
		vTaskDelay(delay);
	}

//ENDS UP IN TOP LEFT CORNER

	xSize = (topNum+bottomNum)/2;
	ySize = (rightNum+leftNum)/2;

	char buffer[10];
	int n;
	n=sprintf(buffer, "X: %d \r\n", xSize);
	Board_UARTPutSTR(buffer);
	n=sprintf(buffer, "Y: %d \r\n", ySize);
	Board_UARTPutSTR(buffer);
}

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
    prvSetupHardware();
	xTaskCreate(calibrate, "calibrate",
				configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	/* Start the scheduler */
	vTaskStartScheduler();

    int ch;
    std::string GCode;
    Parser parser;

    while(1) {
        if ((ch = Board_UARTGetChar()) != EOF) {
        	if (ch == 10) {
        		parser.Parse(GCode);
        		Board_UARTPutSTR(parser.output.c_str());
        		if (parser.output != "invalid code") {
            		Board_UARTPutSTR("\r\n");
            		Board_UARTPutSTR("OK\r\n");
        		}
        		GCode = "";
        	}
        	else GCode += (char) ch;
        }
    }
    return 0 ;
}

void motor(char direction){
	DigitalIoPin xDir(0, 28, DigitalIoPin::output, true); //down-up
	DigitalIoPin xMotor(0, 27, DigitalIoPin::output, true);
	DigitalIoPin yDir(1, 0, DigitalIoPin::output, true); //left-right
	DigitalIoPin yMotor(0, 24, DigitalIoPin::output, true);

		if (direction == 'R'){
			yMotor.write(false);
			yDir.write(false);
			yMotor.write(true);
		}
		else if (direction == 'D'){
			xMotor.write(false);
			xDir.write(true);
			xMotor.write(true);
		}
		else if (direction == 'L'){
			yMotor.write(false);
			yDir.write(true);
			yMotor.write(true);
		}
		else if (direction == 'U'){
			xMotor.write(false);
			xDir.write(false);
			xMotor.write(true);
		}
}
