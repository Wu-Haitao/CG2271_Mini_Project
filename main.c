#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define MASK(x) (1 << (x)) 
#define RED_LED 18
#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define FREQ_TO_MOD(x) (375000/x) //48000000 / 128 = 375000

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

void initPWM(void)
{
	//enable clock for portB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//configure pin mux to alt3
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	//enable clock for timer1 and timer2
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	//select clock source for TPM
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->MOD = FREQ_TO_MOD(50);
	TPM2->MOD = FREQ_TO_MOD(50);
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//channel0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//channel1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//channel2
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//channel3
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
  TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
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
		osMutexAcquire(myMutex, osWaitForever);
		global_rx = UART2->D;
		osMutexRelease(myMutex);
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
	for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		if (global_rx & 0x8) {
			TPM1_C0V = 0;
			TPM1_C1V = FREQ_TO_MOD(50) * (global_rx & 0x7) / (0x7);
		} else {
			TPM1_C0V = FREQ_TO_MOD(50) * (global_rx & 0x7) / (0x7);
			TPM1_C1V = 0;
		}
		if ((global_rx >> 4) & 0x8) {
			TPM2_C0V = 0;
			TPM2_C1V = FREQ_TO_MOD(50) * ((global_rx >> 4) & 0x7) / (0x7);
		} else {
			TPM2_C0V = FREQ_TO_MOD(50) * ((global_rx >> 4) & 0x7) / (0x7);
			TPM2_C1V = 0;
		}
		osMutexRelease(myMutex);
	}
}

//Controls LEDs
void tLED (void *argument) {
	for (;;) {
		//osMutexAcquire(myMutex, osWaitForever);
		//Only for test
		//if (global_rx) {
		//	LED_On(RED_LED);
		//} else {
		//	LED_Off(RED_LED);
		//}
		//osMutexRelease(myMutex);
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
  initPWM();
	LED_Off(RED_LED);
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  myMutex = osMutexNew(NULL);
	osThreadNew(tBrain, NULL, NULL);
  osThreadNew(tMotorControl, NULL, NULL);
  //osThreadNew(tLED, NULL, NULL);
  osThreadNew(tAudio, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
