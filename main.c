#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define MASK(x) (1 << (x)) 
#define RED_LED 18

uint8_t global_rx;
osMutexId_t myMutex;

void initGPIO(void)
{
	// Enable Clock to PORTB
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	// Configure MUX
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	// Set Data Direction Registers for PortB
	PTB->PDDR |= MASK(RED_LED);
}

void initUART2(uint32_t baud_rate){
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~UART_C2_RE_MASK;
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= (UART_C2_RE_MASK);
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		global_rx = UART2->D;
	}
}

void LED_On(int LEDPin) {
	PTB->PDOR &= ~MASK(LEDPin);
}

void LED_Off(int LEDPin) {
	PTB->PDOR |= MASK(LEDPin);
}

//Decodes the data from the Serial Port and performs the necessary action
void tBrain (void *argument) {
}

//Controls motors
void tMotorControl (void *argument) {
}

//Controls LEDs
void tLED (void *argument) {
	for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		//Only for test
		if (global_rx) {
			LED_On(RED_LED);
		} else {
			LED_Off(RED_LED);
		}
		osMutexRelease(myMutex);
	}
}

//Provides audio output
void tAudio (void *argument) {
}

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
  initUART2(BAUD_RATE);
	initGPIO();
	LED_Off(RED_LED);
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  myMutex = osMutexNew(NULL);
	osThreadNew(tBrain, NULL, NULL);
  osThreadNew(tMotorControl, NULL, NULL);
  osThreadNew(tLED, NULL, NULL);
  osThreadNew(tAudio, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
