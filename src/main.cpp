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

/*Declaration of global variables*/
volatile uint32_t RIT_count;		//counter for RITimer
SemaphoreHandle_t sbRIT = NULL;		//Semaphore for the RITimer
std::vector<DigitalIoPin> args;		//vector for the PINS


void motor(char);

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
	}
	else {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
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

static void vUARTCommTask(void *pvParameters) {
    int ch;
    std::string GCode;
    Parser parser;

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

int main(void) {
    // TODO: insert code here
    prvSetupHardware();

    /*Set up the global variables*/
	sbRIT = xSemaphoreCreateBinary();

    /*Create all tasks here*/
	xTaskCreate(calibrate, "calibrate",
				configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	xTaskCreate(vUARTCommTask, "UART",
			    configMINIMAL_STACK_SIZE + 128, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

    while(1) {

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
