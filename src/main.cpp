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
#include <cstring>
#include "Parser.h"
#include "DigitalIoPin.h"
#include <vector>
#include <cmath>
#include "event_groups.h"

/*Declaration of global variables*/
volatile uint32_t RIT_count;		//counter for RITimer
volatile bool vertical;
int xSize = 0;
int ySize = 0;
int xSizemm = 390;	//310 in simulator
int ySizemm = 430;	//380 in simulator

SemaphoreHandle_t sbRIT = NULL;		//Semaphore for the RITimer
SemaphoreHandle_t xSignal = NULL;  	//signal for the motor to move in the X direction
SemaphoreHandle_t ySignal = NULL;	//signal for the motor to move in the Y direction
SemaphoreHandle_t doneMoving = NULL;//signal to keep reading
QueueHandle_t xCmdQueue;
std::vector<DigitalIoPin> args;		//vector for the PINS
EventGroupHandle_t lsFlags;
TaskHandle_t UARTHandle;
TaskHandle_t flagHandle;

/*Declaration of data types*/
struct Coordinates {
	float currX;
	float currY;
	float tarX;
	float tarY;
};

enum vctPos {	limit1,
				limit2,
				limit3,
				limit4,
				laser,
				pen,
				xMotor,
				xDir,
				yMotor,
				yDir,
				sw1,
				sw2,
				sw3
			};

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
/*-------------------------------------------------------------------*/
/*Functions declaration*/
/*-------------------------------------------------------------------*/
extern "C" {

void RIT_IRQHandler(void){
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
	if(RIT_count > 0) {
		RIT_count--;
		// do something useful here...
		if (vertical == true) {
			xSemaphoreGiveFromISR(ySignal, &xHigherPriorityWoken);
		} else
			xSemaphoreGiveFromISR(xSignal, &xHigherPriorityWoken);
	}
	else {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

void RIT_start(int count, int us)
{
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us / 1000000;
	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	RIT_count = count;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if(xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	}
	else {
	// unexpected error
	}
}
/* Sets up system hardware */
static void prvSetupHardware(void){
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);	//pen

	Board_LED_Set(0, false);
}
void SCT_Init(void){
	Chip_SCT_Init(LPC_SCTLARGE0);

	LPC_SCTLARGE0->CONFIG |= (1 << 17); // two 16 bit timers, auto limit
	LPC_SCTLARGE0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock = 1 MHz
	LPC_SCTLARGE0->MATCHREL[0].L = 20000 -1;
	LPC_SCTLARGE0->MATCHREL[1].L = 1000;

	LPC_SCTLARGE0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all state
	LPC_SCTLARGE0->EVENT[0].CTRL = (1 << 12); // match 0 condition only

	LPC_SCTLARGE0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCTLARGE0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only

	LPC_SCTLARGE0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCTLARGE0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	LPC_SCTLARGE0->CTRL_L &= ~(1 << 2); // start timer
}
/*X is the running*/
/*Plotting line with 0 < abs(slope) < 1 (x is running), positive direction*/
void PlotLineLowPositive(int x0, int y0, int x1, int y1) {
	int delay = 100;
	int dx = x1 - x0;
	int dy = y1 - y0;
	if (dy < 0)
		dy = -dy;
	int D = 2*dy - dx;

	for (int i = x0;i < x1; i++) {
		vertical = false;
		RIT_start(2, delay);
		if (D > 0) {
			vertical = true;
			RIT_start(2,delay);
			D = D - 2*dx;
		}
		D = D + 2*dy;
	}
}
/*Plotting line with 0 < abs(slope) < 1 (x is running), negative direction*/
void PlotLineLowNegative(int x0, int y0, int x1, int y1) {
	int delay = 100;
	int dx = x0 - x1;
	int dy = y1 - y0;
	if (dy < 0)
		dy = -dy;
	int D = 2*dy - dx;

	for (int x = x0;x > x1; x--) {
		vertical = false;
		RIT_start(2, delay);
		if (D > 0) {
			vertical = true;
			RIT_start(2,delay);
			D = D - 2*dx;
		}
		D = D + 2*dy;
	}
}
/*Y is the running*/
/*Plotting line with abs(slope) > 1 (y is running), positive direction*/
void plotLineHighPositive(int x0, int y0, int x1, int y1) {
	int delay = 100;
	int dx = x1 - x0;
	int dy = y1 - y0;
	if (dx < 0)
		dx = -dx;
	int D = 2*dx - dy;

	for (int y = y0;y < y1; y++) {
		vertical = true;
		RIT_start(2, delay);
		if (D > 0) {
			vertical = false;
			RIT_start(2,delay);
			D = D - 2*dy;
		}
		D = D + 2*dx;
	}
}
/*Plotting line with abs(slope) > 1 (y is running), negative direction*/
void plotLineHighNegative(int x0, int y0, int x1, int y1) {
	int delay = 100;
	int dx = x1 - x0;
	int dy = y0 - y1;
	if (dx < 0)
		dx = -dx;
	int D = 2*dx - dy;

	for (int y = y0;y > y1; y--) {
		vertical = true;
		RIT_start(2, delay);
		if (D > 0) {
			vertical = false;
			RIT_start(2, delay);
			D = D - 2*dy;
		}
		D = D + 2*dx;
	}
}
/*Selecting which case to plot*/
void Plot(int x0, int y0, int x, int y) {
	if (abs(y - y0) < abs(x - x0)) {
		if (x < x0) {
			PlotLineLowNegative(x0, y0, x, y);
		}
		else PlotLineLowPositive(x0, y0, x, y);
	} else {
		if (y < y0) {
			plotLineHighNegative(x0, y0, x, y);
		} else {
			plotLineHighPositive(x0, y0, x, y);
		}
	}
}

/*-------------------------------------------------------------------*/
/*Task declaration*/
/*-------------------------------------------------------------------*/
/* Task for UART communication
 * - Take the GCode from the serial port sent by mDraw
 * - Parse the Gcode and execute the commands accordingly
 * 		- Send the reply on Information Querry
 * 		- Set the pen to up/down (either directly or calling from another class)
 * 		- Send the coordinate data (raw) to the queue for the controller
 * */
static void vUARTCommTask(void *pvParameters) {
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();

	vTaskDelay(100);
    int ch;
    std::string GCode;
    Parser parser;
    Coordinates cmd;
    //Set the initial position to be at the top left
	cmd.currX = 1;
	cmd.currY = 1;
	char buffer[50];	//used for printing

	int penUpValue = 160;
	int penDownValue = 90;
	int ls1 = 0;
	int ls2 = 0;
	int ls3 = 0;
	int ls4 = 0;
	int xDirection = 0;
	int yDirection = 0;
	int plottingSpeed = 80;

	vTaskSuspend(UARTHandle);
	while (1) {
		if ((ch = Board_UARTGetChar()) != EOF) {
			if (ch == 10) {
				std::string output(parser.Parse(GCode));
				//SET PEN POSITION
				if(output == "M1"){
					std::string value = "";
					value = value.append(GCode.begin()+3, GCode.end());
					if (value == std::to_string(penUpValue)){
						LPC_SCT0->MATCHREL[1].L = 1100;				//up 1600 in simulator
					}
					else if (value == std::to_string(penDownValue)){
						LPC_SCT0->MATCHREL[1].L = 1600;				//down 1100 in simulator
					}
					Board_UARTPutSTR("OK\r\n");
				}

				//SAVE PEN UP/DOWN POSITION
				else if(output == "M2"){
					std::string uVal = "";
					std::string dVal = "";
					//skip "M2 U"
					int i = 4;

					//get up value
					while(GCode.at(i) != ' '){
						uVal = uVal+GCode.at(i);
						i++;
					}
					penUpValue = atoi(uVal.c_str());

					//skip space and character "D" from GCode
					i = i+2;

					//get down value
					while(GCode.at(i) != ' '){
						dVal = dVal+GCode.at(i);
						i++;
					}
					penDownValue = atoi(dVal.c_str());
					Board_UARTPutSTR("OK\r\n");
				}
				//SET LASER OUTPUT
				else if(output == "M4"){
					std::string value = "";
					value = value.append(GCode.begin()+3, GCode.end());
					if (atoi(value.c_str())){
						(it+4)->write(true);
					}
					else {
						(it+4)->write(false);
					}
					//small delay for the laser to turn off
					vTaskDelay(100);
					Board_UARTPutSTR("OK\r\n");
				}

				//SAVE STEPPER DIRECTION, AREA, SPEED
				else if(output == "M5"){
					//M5 A0 B0 H310 W380 S80
					std::string xVal = "";
					std::string yVal = "";
					std::string hVal = "";
					std::string wVal = "";
					std::string sVal = "";
					//skip "M5 A"
					int i = 4;

					//get xVal
					while(GCode.at(i) != ' '){
						xVal = xVal+GCode.at(i);
						i++;
					}
					xDirection = atoi(xVal.c_str());
					//skip space and character "B" from GCode
					i = i+2;
					//get yVal
					while(GCode.at(i) != ' '){
						yVal = yVal+GCode.at(i);
						i++;
					}
					yDirection = atoi(yVal.c_str());
					//skip space and character "H" from GCode
					i = i+2;
					//get hVal
					while(GCode.at(i) != ' '){
						hVal = hVal+GCode.at(i);
						i++;
					}
					ySizemm = atoi(hVal.c_str());
					//skip space and character "W" from GCode
					i = i+2;
					//get wVal
					while(GCode.at(i) != ' '){
						wVal = wVal+GCode.at(i);
						i++;
					}
					xSizemm = atoi(wVal.c_str());
					//skip space and character "S" from GCode
					i = i+2;
					//get sVal
					while(GCode.at(i) != ' '){
						sVal = sVal+GCode.at(i);
						i++;
					}
					plottingSpeed = atoi(sVal.c_str());

					Board_UARTPutSTR("OK\r\n");
				}
				//SEND THE START UP REPLY
				else if(output == "M10"){
					//sprintf(buffer, "M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\n");
					sprintf(buffer, "M10 XY %d %d 0.00 0.00 A%d B%d H0 S%d U%d D%d\r\n",
							ySizemm, xSizemm, xDirection, yDirection, plottingSpeed, penUpValue, penDownValue);
					Board_UARTPutSTR(buffer);
					Board_UARTPutSTR("OK\r\n");
				}

				//LIMIT SWITCH STATUS QUERY
				else if(output == "M11"){
					//reply order: L4(left), L3(right), L2(down), L1(up)
					//1 means open switch, 0 means closed
					if((it+limit1)->read()==true){	//limit switch 1
						ls1 = 0;
					}else{
						ls1 = 1;
					}
					if((it+limit2)->read()==true){	//limit switch 2
						ls2 = 0;
					}else{
						ls2 = 1;
					}
					if((it+limit3)->read()==true){	//limit switch 3
						ls3 = 0;
					}else{
						ls3 = 1;
					}
					if((it+limit4)->read()==true){	//limit switch 4
						ls4 = 0;
					}else{
						ls4 = 1;
					}
					sprintf(buffer, "M11 %d %d %d %d \r\n", ls4, ls3, ls2, ls1);
					Board_UARTPutSTR(buffer);
					Board_UARTPutSTR("OK\r\n");
				}
				//GET COORDINATES
				else if(output == "G1"){
					std::string xVal = "";
					std::string yVal = "";
					//skip "G1 X"
					int i = 4;

					//get xVal
					while(GCode.at(i) != ' '){
						xVal = xVal+GCode.at(i);
						i++;
					}
					cmd.tarX = atof(xVal.c_str());

					//skip space and character "Y" from GCode
					i = i+2;

					//get yVal
					while(GCode.at(i) != ' '){
						yVal = yVal+GCode.at(i);
						i++;
					}
					cmd.tarY = atof(yVal.c_str());
					//send the command to a queue
					xQueueSend(xCmdQueue, &cmd, portMAX_DELAY);
					//update the current position
					cmd.currX = cmd.tarX;
					cmd.currY = cmd.tarY;
					//wait for the motors to stop moving then
					if (xSemaphoreTake(doneMoving, portMAX_DELAY) == pdTRUE) {
						Board_UARTPutSTR("OK\r\n");
					}
				}

				//GO TO ORIGIN
				else if(output == "G28"){
					//xVal is 0
					//yVal is 0
					cmd.tarX = 0;
					cmd.tarY = 0;
					xQueueSend(xCmdQueue, &cmd, portMAX_DELAY);
					//update the current positions
					cmd.currX = cmd.tarX;
					cmd.currY = cmd.tarY;
					//wait for the motors to stop moving then
					if (xSemaphoreTake(doneMoving, portMAX_DELAY) == pdTRUE) {
						Board_UARTPutSTR("OK\r\n");
					}
				}
				GCode = "";
			}
			else GCode += (char) ch;
		}
	}
}
/* Task for controlling the X direction motor
 * Monitor a semaphore: xSignal
 * Each call sends 1 pulse;
 * */
static void vMotorXTask(void *pvParameters) {
	//ls3 = 2
	//ls4 = 3
	//xMotor = 6
	//xDir = 7
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();
	bool signal = true;
	vTaskDelay(100);

	while(1) {
		//normal movement used in the task
		if (xSemaphoreTake(xSignal, portMAX_DELAY) == pdTRUE) {
			if ((it+limit3)->read() == false && (it+limit4)->read() == false) {
				(it+xMotor)->write(signal);
				signal = !signal;
			}
		}
	}
}
/* Task for controlling the Y direction motor
 * Monitor a semaphore: ySignal
 * Each call sends 1 pulse;
 * */
static void vMotorYTask(void *pvParameters) {
	//ls1 = 0
	//ls2 = 1
	//yMotor = 8
	//yDir = 9
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();
	bool signal = true;
	vTaskDelay(100);

	while(1) {
		//normal movement used in the task
		if (xSemaphoreTake(ySignal, portMAX_DELAY) == pdTRUE) {
			if ((it+limit1)->read() == false && (it+limit2)->read() == false) {
				(it+yMotor)->write(signal);
				signal = !signal;
			}
		}
	}
}
/* Calibration task. Called once and Deleted
 * - Run to each Limit switch and record the number of steps
 */
static void vCalibrationTask(void *pvParameters) {
	LPC_SCTLARGE0->MATCHREL[1].L = 1100;	//set pen up

    DigitalIoPin limSW1(1, 3, DigitalIoPin::pullup, true);		//up
	DigitalIoPin limSW2(0, 0, DigitalIoPin::pullup, true);		//down
	DigitalIoPin limSW3(0, 9, DigitalIoPin::pullup, true);		//right
	DigitalIoPin limSW4(0, 29, DigitalIoPin::pullup, true); 	//left

	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();
	int delay = 50;
	int count = 0;

	(it+laser)->write(false);	//laser
	//Delay for laser to actually turn off
	vTaskDelay(100);
	/* Calibration plan:
	 * - Runs in the circle: +X +Y -X -Y
	 * - Counter with increment value 0 -> 4 (the number of switches read)
	 * - When a switch is read, change the value in the vector based on what we have
	 * - When a switch is read, it changes a bit in the event group
	 * - Clear bit on every cycle
	 * */
	while (1) {
		EventBits_t uxBits = xEventGroupGetBits(lsFlags);
		switch (count) {
		case 0:										//no switches hit, +X, expecting Limit 4
			if ((uxBits & 0x0F) == 0) {	//& 0x0F == 0 meaning no bit were set
				vertical = false;
				(it+xDir)->write(true);
				RIT_start(2,delay);
			} else
			if (((uxBits & 0x0F) != 0)) {			//(at least) 1 bit is set but the switch is not saved
				if ((uxBits & BIT_0) == BIT_0) {	//find out which switch is hit
					*(it+limit4) = limSW1;
				}
				if ((uxBits & BIT_1) == BIT_1) {
					*(it+limit4) = limSW2;
				}
				if ((uxBits & BIT_2) == BIT_2) {
					*(it+limit4) = limSW3;
				}
				if ((uxBits & BIT_3) == BIT_3) {
					*(it+limit4) = limSW4;
				}
				while ((it+limit4)->read() == true) {
					(it+xDir)->write(false);				//Move back to not hit the switch anymore
					(it+xMotor)->write(true);
					RIT_start(1, delay);
					(it+xMotor)->write(false);
					RIT_start(1, delay);
				}
				xEventGroupClearBits(lsFlags, 0x0F);//clear the bits
				count++;
			}
			break;
		case 1:										//1 switch found, +Y, expecting Limit 2
			if ((uxBits & 0x0F) == 0) {	//& 0x0F == 0 meaning no bit were set
				vertical = true;
				(it+yDir)->write(true);
				RIT_start(2,delay);
			} else
			if (((uxBits & 0x0F) != 0)) {			//(at least) 1 bit is set but the switch is not saved
				if ((uxBits & BIT_0) == BIT_0) {	//find out which switch is hit
					*(it+limit2) = limSW1;
				}
				if ((uxBits & BIT_1) == BIT_1) {
					*(it+limit2) = limSW2;
				}
				if ((uxBits & BIT_2) == BIT_2) {
					*(it+limit2) = limSW3;
				}
				if ((uxBits & BIT_3) == BIT_3) {
					*(it+limit2) = limSW4;
				}
				while ((it+limit2)->read()) {
					(it+yDir)->write(false);				//Move back to not hit the switch anymore
					(it+yMotor)->write(true);
					RIT_start(1, delay);
					(it+yMotor)->write(false);
					RIT_start(1, delay);
				}
				xEventGroupClearBits(lsFlags, 0x0F);//clear the bits
				count++;
			}
			break;
		case 2:										//2 switches found, -X expecting Limit 3
			if ((uxBits & 0x0F) == 0) {	//& 0x0F == 0 meaning no bit were set
				xSize++;
				vertical = false;
				(it+xDir)->write(false);
				RIT_start(2,delay);
			} else
			if (((uxBits & 0x0F) != 0)) {			//(at least) 1 bit is set but the switch is not saved
				if ((uxBits & BIT_0) == BIT_0) {	//find out which switch is hit
					*(it+limit3) = limSW1;
				}
				if ((uxBits & BIT_1) == BIT_1) {
					*(it+limit3) = limSW2;
				}
				if ((uxBits & BIT_2) == BIT_2) {
					*(it+limit3) = limSW3;
				}
				if ((uxBits & BIT_3) == BIT_3) {
					*(it+limit3) = limSW4;
				}
				while ((it+limit3)->read() == true) {
					xSize--;
					(it+xDir)->write(true);				//Move back to not hit the switch anymore
					(it+xMotor)->write(true);
					RIT_start(1, delay);
					(it+xMotor)->write(false);
					RIT_start(1, delay);
				}
				xEventGroupClearBits(lsFlags, 0x0F);//clear the bits
				count++;
			}
			break;
		case 3:										//3 switches found, -Y, expecting Limit 1
			if ((uxBits & 0x0F) == 0) {	//& 0x0F == 0 meaning no bit were set
				ySize++;
				vertical = true;
				(it+yDir)->write(false);
				RIT_start(2,delay);
			} else
			if (((uxBits & 0x0F) != 0)) {			//(at least) 1 bit is set but the switch is not saved
				if ((uxBits & BIT_0) == BIT_0) {	//find out which switch is hit
					*(it+limit1) = limSW1;
				}
				if ((uxBits & BIT_1) == BIT_1) {
					*(it+limit1) = limSW2;
				}
				if ((uxBits & BIT_2) == BIT_2) {
					*(it+limit1) = limSW3;
				}
				if ((uxBits & BIT_3) == BIT_3) {
					*(it+limit1) = limSW4;
				}
				while ((it+limit1)->read() == true) {
					ySize--;
					(it+yDir)->write(true);				//Move back to not hit the switch anymore
					(it+yMotor)->write(true);
					RIT_start(1, delay);
					(it+yMotor)->write(false);
					RIT_start(1, delay);
				}
				xEventGroupClearBits(lsFlags, 0x0F);//clear the bits
				count++;
			}
			break;
		case 4:										//all limit switches found
			xSize/=2;
			ySize/=2;
			char buffer[10];
			int n;
			n=sprintf(buffer, "X: %d \r\n", xSize);
			Board_UARTPutSTR(buffer);
			n=sprintf(buffer, "Y: %d \r\n", ySize);
			Board_UARTPutSTR(buffer);
			vTaskSuspend(flagHandle);
			vTaskResume(UARTHandle);
			vTaskDelete(flagHandle);
			vTaskDelete(NULL);
			break;
		}
	}
//ENDS UP AT (0,0)
//	LPC_SCTLARGE0->MATCHREL[1].L = 1600;	//set pen up
}

/* Controller task for the motor
 * - Monitors a queue and take the values from there
 * - Calculate the ratios and send to Plot function to execute
 * */
static void vControllerTask(void *pvParameters) {
	//xDir = 7
	//yDir = 9
	Coordinates cmd;
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();
	vTaskDelay(100);
	//convert the value to steps;
	//X size = 380, Y size = 310

	while(1) {
		/*Take command from a queue*/
		if (xQueueReceive(xCmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
			//calculate the current number of steps from the origin
			float x,y;
			//Extract the coordinate and calculate the number of steps needed
			x = (cmd.tarX-cmd.currX);
			y = (cmd.tarY-cmd.currY);
			std::string debugMsg;
			debugMsg += std::to_string(cmd.currY);
			debugMsg += " ";
			debugMsg += std::to_string(cmd.tarY);
			debugMsg += " \r\n";
			Board_UARTPutSTR(debugMsg.c_str());
			debugMsg = "";
			//set the direction of the motor based on the value of x and y
			if (x>=0)
				(it+xDir)->write(true);
			else (it+xDir)->write(false);

			if (y>=0)
				(it+yDir)->write(true);
			else (it+yDir)->write(false);

			//set the scaling value: determining the steps/pixel ratio
			x = xSize/xSizemm;
			y = ySize/ySizemm;
//			x=1;
//			y=1;
			/*algorithm: Bresenham
			 * - Determine either x or y is the running variable (the large of the 2)
			 * - Based on the slope of the line (y-y0)/(x-x0),
			 *   determine whether to increase the follow variable
			 * - Call RIT_start accordingly
			 * */
			Plot(round(cmd.currX * x), round(cmd.currY * y), round(cmd.tarX * x), round(cmd.tarY * y));
			xSemaphoreGive(doneMoving);
		}
	}
}

static void vSetLsFlagTask(void *pvParameters) {
    DigitalIoPin limSW1(1, 3, DigitalIoPin::pullup, true);		//up
	DigitalIoPin limSW2(0, 0, DigitalIoPin::pullup, true);		//down
	DigitalIoPin limSW3(0, 9, DigitalIoPin::pullup, true);		//right
	DigitalIoPin limSW4(0, 29, DigitalIoPin::pullup, true); 	//left

	while (1) {
		if (limSW1.read()) xEventGroupSetBits(lsFlags, BIT_0);
		else if (limSW2.read()) xEventGroupSetBits(lsFlags, BIT_1);
		else if (limSW3.read()) xEventGroupSetBits(lsFlags, BIT_2);
		else if (limSW4.read()) xEventGroupSetBits(lsFlags, BIT_3);
		vTaskDelay(50);
	}
}

/*-------------------------------------------------------------------*/
/*Main function*/
/*-------------------------------------------------------------------*/
int main(void) {
    // TODO: insert code here
	prvSetupHardware();
	SCT_Init();

    /*Set up the ITM write console*/
    /*set up the RITimer*/
	Chip_RIT_Init(LPC_RITIMER);
	NVIC_SetPriority( RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1 );

    /*Set up the queue/semaphores*/
	sbRIT = xSemaphoreCreateBinary();
	xSignal = xSemaphoreCreateBinary();
	ySignal = xSemaphoreCreateBinary();
	doneMoving = xSemaphoreCreateBinary();
	xCmdQueue = xQueueCreate(20, sizeof(Coordinates));

	/*Set up the event group*/
	lsFlags = xEventGroupCreate();

	vQueueAddToRegistry(xCmdQueue, "coordinates");
	vQueueAddToRegistry(xSignal, "xSignal");
	vQueueAddToRegistry(ySignal, "ySignal");
	vQueueAddToRegistry(doneMoving, "take me to UARt");


	/*Set up Pins*/
    DigitalIoPin limSW1(1, 3, DigitalIoPin::pullup, true);		//up
	DigitalIoPin limSW2(0, 0, DigitalIoPin::pullup, true);		//down
	DigitalIoPin limSW3(0, 9, DigitalIoPin::pullup, true);		//right
	DigitalIoPin limSW4(0, 29, DigitalIoPin::pullup, true); 	//left

    DigitalIoPin laser(0, 12, DigitalIoPin::output, true);		//laser
	DigitalIoPin pen(0, 10, DigitalIoPin::output, true);		//pen

	DigitalIoPin yMotor(0, 27, DigitalIoPin::output, true);		//horizontal movement
	DigitalIoPin yDir(0, 28, DigitalIoPin::output, true);		//horizontal direction
	DigitalIoPin xMotor(0, 24, DigitalIoPin::output, true);		//vertical movement
	DigitalIoPin xDir(1, 0, DigitalIoPin::output, true);		//vertical direction

	DigitalIoPin SW1(0, 8, DigitalIoPin::pullup, true);
	DigitalIoPin SW2(1, 6, DigitalIoPin::pullup, true);
	DigitalIoPin SW3(1, 8, DigitalIoPin::pullup, true);

	args.clear();
	args.push_back(limSW1);
	args.push_back(limSW2);
	args.push_back(limSW3);
	args.push_back(limSW4);
	args.push_back(laser);
	args.push_back(pen);
	args.push_back(xMotor);
	args.push_back(xDir);
	args.push_back(yMotor);
	args.push_back(yDir);
	args.push_back(SW1);
	args.push_back(SW2);
	args.push_back(SW3);

    /*Create all tasks here*/
	xTaskCreate(vCalibrationTask, "calibration",
				configMINIMAL_STACK_SIZE + 128, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	//*4 enough in simulator, *10 in plotter
	xTaskCreate(vUARTCommTask, "UART",
			    configMINIMAL_STACK_SIZE * 8, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) &UARTHandle);

	xTaskCreate(vMotorXTask, "motorX",
				configMINIMAL_STACK_SIZE, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vMotorYTask, "motorY",
				configMINIMAL_STACK_SIZE, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vControllerTask, "controller",
			    configMINIMAL_STACK_SIZE + 128 + 128, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vSetLsFlagTask, "setFlag",
				configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) &flagHandle);

	/*Call the preparation functions*/

	/* Start the scheduler */
	vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}
