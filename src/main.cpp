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
#include "Plotter.h"

/*Declaration of global variables*/
volatile uint32_t RIT_count;		//counter for RITimer
volatile bool vertical;
int xSize = 0;
int ySize = 0;

SemaphoreHandle_t sbRIT = NULL;		//Semaphore for the RITimer
SemaphoreHandle_t xSignal = NULL;  	//signal for the motor to move in the X direction
SemaphoreHandle_t ySignal = NULL;	//signal for the motor to move in the Y direction
SemaphoreHandle_t doneMoving = NULL;//signal to keep reading
QueueHandle_t xCmdQueue;
std::vector<DigitalIoPin> args;		//vector for the PINS
TaskHandle_t limitHandle = NULL;
TaskHandle_t uartHandle = NULL;


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
    //Set the initial position to be at the origin
	cmd.currX = 1;
	cmd.currY = 1;
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);	//pen

	while (1) {
		if ((ch = Board_UARTGetChar()) != EOF) {
			if (ch == 10) {
				std::string output(parser.Parse(GCode));
//					Board_UARTPutSTR(GCode.c_str());
//					Board_UARTPutSTR("\r\n");
				if(output == "M1"){		//set pen position
					std::string value = "";
					value = value.append(GCode.begin()+3, GCode.end());
					if (value == "160"){
						LPC_SCT0->MATCHREL[1].L = 1600;	//up
					}
					else {
						LPC_SCT0->MATCHREL[1].L = 1100;	//down
					}
					Board_UARTPutSTR("OK\r\n");
				} else if(output == "M4"){		//set laser output
					std::string value = "";
					value = value.append(GCode.begin()+3, GCode.end());
					if (atoi(value.c_str())){
						(it+4)->write(true);
					}
					else {
						(it+4)->write(false);
					}
					//small delay for the laser to turn off
					vTaskDelay(500);
					Board_UARTPutSTR("OK\r\n");
				}
				else if(output == "M10"){	//send the start up reply
					Board_UARTPutSTR("M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90\r\n");
					Board_UARTPutSTR("OK\r\n");
				}
				else if(output == "G1"){	//get coordinates
					std::string xVal = "";
					std::string yVal = "";
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
				else if(output == "G28"){	//go to origin
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
//	LPC_SCTLARGE0->MATCHREL[1].L = 1100;	//set pen down
	LPC_SCTLARGE0->MATCHREL[1].L = 1600;	//set pen up

	//Limit switch 1	= up
	//Limit switch 2	= down
	//Limit switch 3	= left
	//Limit switch 4	= right
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();

	int delay = 50;

	(it+pen)->write(false);	//laser
	//Delay for laser to actually turn off
	vTaskDelay(100);

	//X+ until limit
	vertical = false;
	while ((it+limit4)->read()==false){
		(it+xDir)->write(false);
		RIT_start(2,delay);
	}
	//back-up to not hit the limit switch
	while ((it+limit4)->read()==true){		//Do not use the motor task since it won't run when limit switches are read
		(it+xDir)->write(true);
		(it+xMotor)->write(true);
		RIT_start(1, delay);
		(it+xMotor)->write(false);
		RIT_start(1, delay);
	}
	//Y+ until limit
	vertical = true;
	while((it+limit1)->read()==false){
		(it+yDir)->write(false);
		RIT_start(2,delay);
	}
	//back-up to not hit the limit switch
	while((it+limit1)->read()==true){
		(it+yDir)->write(true);
		(it+yMotor)->write(true);
		RIT_start(1, delay);
		(it+yMotor)->write(false);
		RIT_start(1, delay);
	}

//X+Y+

	//X- till limit
	vertical = false;
	while ((it+limit3)->read()==false){
		xSize++;
		(it+xDir)->write(true);
		RIT_start(2,delay);
	}

	//back-up to not hit the limit switch
	while ((it+limit3)->read()==true){
		(it+xDir)->write(false);
		(it+xMotor)->write(true);
		RIT_start(1, delay);
		(it+xMotor)->write(false);
		RIT_start(1, delay);
		xSize--;
	}

	//Y- till limit
	vertical = true;
	while((it+limit2)->read()==false){
		ySize++;
		(it+yDir)->write(true);
		RIT_start(2,delay);
	}
	//back-up to not hit the limit switch
	while((it+limit2)->read()==true){
		ySize--;
		(it+yDir)->write(false);
		(it+yMotor)->write(true);
		RIT_start(1, delay);
		(it+yMotor)->write(false);
		RIT_start(1, delay);
	}


//ENDS UP AT (0,0)

//	xSize = (topNum+bottomNum)/2;
//	ySize = (rightNum+leftNum)/2;

	char buffer[10];
	int n;
	n=sprintf(buffer, "X: %d \r\n", xSize);
	Board_UARTPutSTR(buffer);
	n=sprintf(buffer, "Y: %d \r\n", ySize);
	Board_UARTPutSTR(buffer);
	xSize/=2;
	ySize/=2;
//	LPC_SCTLARGE0->MATCHREL[1].L = 1600;	//set pen up

	//resuming the limit task
	vTaskResume(limitHandle);
	vTaskDelete(NULL);
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
				(it+xDir)->write(false);
			else (it+xDir)->write(true);

			if (y>=0)
				(it+yDir)->write(false);
			else (it+yDir)->write(true);

			//set the scaling value: detemining the steps/pixel ratio
			x = xSize/380;
			y = ySize/310;
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

	vQueueAddToRegistry(xCmdQueue, "coordinates");

	/*Set up Pins*/
    DigitalIoPin limSW1(1, 3, DigitalIoPin::pullup, true);		//up
	DigitalIoPin limSW2(0, 0, DigitalIoPin::pullup, true);		//down
	DigitalIoPin limSW3(0, 9, DigitalIoPin::pullup, true);		//right
	DigitalIoPin limSW4(0, 29, DigitalIoPin::pullup, true); 		//left

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

	xTaskCreate(vUARTCommTask, "UART",
			    configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
				&uartHandle);

	xTaskCreate(vMotorXTask, "motorX",
				configMINIMAL_STACK_SIZE, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vMotorYTask, "motorY",
				configMINIMAL_STACK_SIZE, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vControllerTask, "controller",
			    configMINIMAL_STACK_SIZE + 128 + 128, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/*Call the preparation functions*/

	/* Start the scheduler */
	vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}
