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

/*Declaration of global variables*/
volatile uint32_t RIT_count;		//counter for RITimer
volatile bool vertical;
SemaphoreHandle_t sbRIT = NULL;		//Semaphore for the RITimer
std::vector<DigitalIoPin> args;		//vector for the PINS
SemaphoreHandle_t xSignal = NULL;  //signal for the motor to move
SemaphoreHandle_t ySignal = NULL;
bool vertical;
bool caliLock = false;

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
		if (vertical) {
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
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}
/*-------------------------------------------------------------------*/
/*Task declaration*/
/*-------------------------------------------------------------------*/
static void vUARTCommTask(void *pvParameters) {
    int ch;
    std::string GCode;
    Parser parser;

	vTaskDelay(100);
	while (1) {
		if ((ch = Board_UARTGetChar()) != EOF) {
			if (ch == 10) {
				std::string output(parser.Parse(GCode));
				Board_UARTPutSTR(output.c_str());
				if (output != "invalid code") {
					Board_UARTPutSTR("\r\n");
					Board_UARTPutSTR("OK\r\n");
				}
				GCode = "";
			}
			else GCode += (char) ch;
		}
	}
}

static void vMotorXTask(void *pvParameters) {
	//xMotor = 6
	//xDir = 7
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);
	auto it = arr->begin();
	bool signal = true;
	vTaskDelay(100);

	while(1) {
		//normal movement used in the task
		if (xSemaphoreTake(xSignal, portMAX_DELAY) == pdTRUE) {
			/*LOOKING FOR CONDITION TO SELECT THE DIRECTION TO MOVE*/
			(it+6)->write(signal);
			signal = !signal;
		}
	}
}

static void vCalibrationTask(void *pvParameters) {
	std::vector<DigitalIoPin> *arr = static_cast<std::vector<DigitalIoPin>*>(pvParameters);

	auto it = arr->begin();
	(it+4)->write(false);	//drive the laser to false
	(it+5)->write(true);	//force the pen down

	if (!caliLock) {
		//ls1 = 0
		//ls2 = 1
		//ls3 = 2
		//ls4 = 3
		//laser = 4
		//pen = 5
		//xMotor = 6
		//xDir = 7
		//yMotor = 8
		//yDir = 9

		int topNum = 0;
		int rightNum = 0;
		int bottomNum = 0;
		int leftNum = 0;
		int xSize = 0;
		int ySize = 0;
//		int delay = 1000;

		//start checking the X+
		(it+7)->write(true);
		while ((it+3)->read()==false){
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//border hit, time to move back
		(it+7)->write(false);
		while ((it+3)->read()==true){
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//Keep going to Y-
		(it+9)->write(false);
		while(it->read()==false){
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		//border hit, time to move back
		(it+9)->write(true);
		while(it->read()==true){
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		/*--------------------------------------------------*/
		/*Reached the XO corner
		 * start running through the border
		 * register the number of steps of each directions*/
		/*--------------------------------------------------*/
		//check the X-; count the horizontal steps
		(it+7)->write(false);
		while((it+2)->read()==false){
			topNum++;
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//back-up to not hit the limit switch
		(it+7)->write(true);
		while((it+2)->read()==true){
			topNum--;
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//check the Y+ direction, count the vertical limit
		(it+9)->write(true);
		while((it+1)->read()==false){
			rightNum++;
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		//back-up to not hit the limit switch
		(it+9)->write(false);
		while((it+1)->read()==true){
			rightNum--;
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		//check the X+
		(it+7)->write(true);
		while((it+3)->read()==false){
			bottomNum++;
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//back-up to not hit the limit switch
		(it+7)->write(false);
		while((it+3)->read()==true){
			bottomNum--;
			(it+6)->write(true);
			vTaskDelay(1);
			(it+6)->write(false);
			vTaskDelay(1);
		}
		//checking the Y- direction
		(it+9)->write(false);
		while(it->read()==false){
			leftNum++;
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		//back-up to not hit the limit switch
		(it+9)->write(true);
		while(it->read()==true){
			leftNum--;
			(it+8)->write(true);
			vTaskDelay(1);
			(it+8)->write(false);
			vTaskDelay(1);
		}
		/*Finished calibration*/
		/*Calculate the output and print for debug*/
		xSize = (topNum+bottomNum)/2;
		ySize = (rightNum+leftNum)/2;

		char buffer[10];
		sprintf(buffer, "X: %d \r\n", xSize);
		Board_UARTPutSTR(buffer);
		sprintf(buffer, "Y: %d \r\n", ySize);
		Board_UARTPutSTR(buffer);
		caliLock = true;
	}
}

static void vControllerTask(void *pvParameters) {
	while(1) {
		/*Take command from a queue*/
		if (xQueueReceive(xCmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
			/*Extract and sort the commands*/

		}
	}
}

/*-------------------------------------------------------------------*/
/*Main function*/
/*-------------------------------------------------------------------*/
int main(void) {
    prvSetupHardware();

    /*Set up the ITM write console*/
    /*set up the RITimer*/
	Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority( RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1 );

    /*Set up the queue/semaphores*/
	sbRIT = xSemaphoreCreateBinary();
	xSignal = xSemaphoreCreateBinary();
	ySignal = xSemaphoreCreateBinary();

	/*Set up Pins*/
    DigitalIoPin limSW1(1, 3, DigitalIoPin::pullup, true);		//up
	DigitalIoPin limSW2(0, 0, DigitalIoPin::pullup, true);		//down
	DigitalIoPin limSW3(0, 29, DigitalIoPin::pullup, true);		//right
	DigitalIoPin limSW4(0, 9, DigitalIoPin::pullup, true); 		//left

    DigitalIoPin laser(0, 12, DigitalIoPin::output, true);		//laser
	DigitalIoPin pen(0, 10, DigitalIoPin::output, true);		//pen

	DigitalIoPin xMotor(0, 24, DigitalIoPin::output, true);		//horizontal movement
	DigitalIoPin xDir(1, 0, DigitalIoPin::output, true);		//horizontal direction
	DigitalIoPin yMotor(0, 27, DigitalIoPin::output, true);		//vertical movement
	DigitalIoPin yDir(0, 28, DigitalIoPin::output, true);		//vertical direction

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
	xTaskCreate(vUARTCommTask, "UART",
			    configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vMotorXTask, "motorX",
				configMINIMAL_STACK_SIZE, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vCalibrationTask, "calibration",
			    configMINIMAL_STACK_SIZE + 128, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vControllerTask, "controller",
			    configMINIMAL_STACK_SIZE + 128, &args, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/*Call the preparation functions*/

	/* Start the scheduler */
	vTaskStartScheduler();

    while(1) {

    }
    return 0 ;
}
