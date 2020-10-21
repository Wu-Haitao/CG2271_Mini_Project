#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define MASK(x) (1 << (x)) 
#define LEFT_PIN0 0
#define LEFT_PIN1 2
#define RIGHT_PIN0 3
#define RIGHT_PIN1 5
#define FREQ_TO_MOD(x) (375000/x) //48000000 / 128 = 375000
#define MUSIC_PIN 0
#define RED_LED 16
int GREEN_LED[] = {7, 0, 3, 4, 5, 6, 10, 11, 12, 13};
int musical_notes[] = {0, 262, 294, 330, 349, 392, 440, 494, 523, 587, 659, 698, 784, 880, 988, 1046, 1175, 1318, 1397, 1568, 1760, 1976, 554, 622, 740, 831, 932};
int special_music[] = {10, 10, 10, 10, 10, 10, 10, 10, 
9, 9, 9, 9, 9, 9, 9, 9, 
8, 8, 8, 8, 8, 8, 8, 8, 
7, 7, 7, 7, 7, 7, 7, 7, 
6, 6, 6, 6, 6, 6, 6, 6, 
5, 5, 5, 5, 5, 5, 5, 5, 
6, 6, 6, 6, 6, 6, 6, 6, 
7, 7, 7, 7, 7, 7, 7, 7, 
8, 8, 7, 7, 8, 8, 3, 3, 
5, 5, 5, 5, 6, 6, 7, 7, 
8, 8, 7, 7, 8, 8, 3, 3, 
12, 12, 10, 10, 12, 12, 13, 13, 
11, 11, 10, 10, 9, 9, 11, 11, 
10, 10, 9, 9, 8, 8, 7, 7, 
6, 6, 5, 5, 4, 4, 8, 8, 
7, 7, 5, 5, 8, 8, 7, 7, 
12, 12, 10, 11, 12, 12, 10, 11, 
12, 5, 6, 7, 8, 9, 10, 11, 
10, 10, 8, 9, 10, 10, 3, 4, 
5, 6, 5, 3, 5, 8, 7, 8, 
6, 6, 8, 7, 6, 6, 5, 4, 
5, 4, 3, 4, 5, 6, 7, 8, 
6, 6, 8, 7, 8, 8, 7, 6, 
7, 6, 7, 8, 9, 10, 11, 12, 
10, 10, 8, 9, 10, 10, 9, 8, 
9, 7, 8, 9, 10, 9, 8, 7, 
8, 8, 6, 7, 8, 8, 3, 4, 
5, 6, 5, 4, 5, 8, 7, 8, 
6, 6, 8, 7, 6, 6, 5, 4, 
5, 4, 3, 4, 5, 6, 7, 8, 
6, 6, 8, 7, 8, 8, 7, 6, 
7, 8, 9, 8, 7, 8, 6, 7, 
8, 8, 8, 8, 8, 8, 8, 8, 
5, 5, 5, 5, 5, 5, 5, 5, 
6, 6, 6, 6, 6, 6, 6, 6, 
3, 3, 3, 3, 3, 3, 3, 3, 
4, 4, 4, 4, 4, 4, 4, 4, 
8, 8, 8, 8, 8, 8, 8, 8, 
6, 6, 6, 6, 6, 6, 6, 6, 
7, 7, 7, 7, 7, 7, 7, 7, 
8, 8, 8, 8, 8, 8, 8, 8,
0, 0, 0, 0, 0, 0, 0, 0
};
int running_music[] = {10, 23, 
10, 23, 10, 7, 9, 8, 
6, 6, 0, 1, 3, 6, 
7, 7, 0, 3, 25, 7, 
8, 8, 0, 0, 10, 23, 
10, 23, 10, 7, 9, 8, 
6, 6, 0, 1, 3, 6, 
7, 7, 0, 3, 8, 7, 
6, 6, 0, 7, 8, 9, 
10, 10, 0, 5, 11, 10, 
9, 9, 0, 4, 10, 9, 
8, 8, 0, 3, 9, 8, 
7, 7, 7, 7, 7, 7
};

volatile uint8_t global_rx = 0x88;
osSemaphoreId_t mySem;
osEventFlagsId_t LED_Green, LED_Red;
osMessageQueueId_t motorMessage;
osMessageQueueId_t LEDMessage;
osMessageQueueId_t musicMessage;

void initGPIO(void)
{
	// Enable Clock to PORTC
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	// Configure MUX
	for (int i = 0; i < 10; i++) {
		PORTC->PCR[GREEN_LED[i]] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[GREEN_LED[i]] |= PORT_PCR_MUX(1);
	}
	PORTC->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED] |= PORT_PCR_MUX(1);
	// Set Data Direction Registers for PortC
	for (int i = 0; i < 10; i++) {
		PTC->PDDR |= MASK(GREEN_LED[i]);
	}
	PTC->PDDR |= MASK(RED_LED);
}

void initPWM(void)
{
	//Enable clock for portB and portD
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	//Configure pin mux to alt4
	PORTB->PCR[MUSIC_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[MUSIC_PIN] |= PORT_PCR_MUX(3);
	
	PORTD->PCR[LEFT_PIN0] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[LEFT_PIN0] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[LEFT_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[LEFT_PIN1] |= PORT_PCR_MUX(4);

	PORTD->PCR[RIGHT_PIN0] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[RIGHT_PIN0] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[RIGHT_PIN1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[RIGHT_PIN1] |= PORT_PCR_MUX(4);
	
	//Enable clock for timer0 and timer1
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//Select clock source for TPM
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->MOD = FREQ_TO_MOD(50);
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
		
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC |= (TPM_SC_CPWMS_MASK);

	//Motors
	//Channel0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//Channel2
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//Channel3
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//Channel5
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
  TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	//Music
	//Channel0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

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
		osSemaphoreRelease(mySem);
	}
}

//Decodes the data from the Serial Port and performs the necessary action
void tBrain (void *argument) {
	_Bool isMoving;
	_Bool music;
	for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);
		osMessageQueuePut(motorMessage, (uint8_t*)&global_rx, NULL, 0);
		if (global_rx & 0x77) {
			isMoving = 1;
		} else {
			isMoving = 0;
		}
		if (global_rx == 0) music = 1;	
		osMessageQueuePut(LEDMessage, &isMoving, NULL, 0);
		osMessageQueuePut(musicMessage, &music, NULL, 0);
		
	}
}

//Controls motors
void tMotorControl (void *argument) {
	uint8_t myRxData;
	for (;;) {
		osMessageQueueGet(motorMessage, &myRxData, NULL, 0);
		if (myRxData & 0x8) {
			TPM0_C0V = 0;
			TPM0_C2V = FREQ_TO_MOD(50) * (myRxData & 0x7) / (0x7);
		} else {
			TPM0_C0V = FREQ_TO_MOD(50) * (myRxData & 0x7) / (0x7);
			TPM0_C2V = 0;
		}
		if ((myRxData >> 4) & 0x8) {
			TPM0_C3V = 0;
			TPM0_C5V = FREQ_TO_MOD(50) * ((myRxData >> 4) & 0x7) / (0x7);
		} else {
			TPM0_C3V = FREQ_TO_MOD(50) * ((myRxData >> 4) & 0x7) / (0x7);
			TPM0_C5V = 0;
		}
	}
}

void LED_Off(int LEDPin) {
	PTC->PDOR &= ~MASK(LEDPin);
}

void LED_On(int LEDPin) {
	PTC->PDOR |= MASK(LEDPin);
}

//Controls LEDs
void tLED (void *argument) {
	_Bool isMoving;
	for (;;) {
		osMessageQueueGet(LEDMessage, &isMoving, NULL, 0);
		if (isMoving) {
			osEventFlagsSet(LED_Green, 0x1);
			osEventFlagsSet(LED_Red, 0x1);
		} else {
			osEventFlagsSet(LED_Green, 0x2);
			osEventFlagsSet(LED_Red, 0x2);
		}
	}
}

void tLED_Green_Pattern1 (void *argument) {
	int currentGreenLED = 0;
	_Bool flag = 1;
	for (;;) {
		osEventFlagsWait(LED_Green, 0x1, osFlagsWaitAny, osWaitForever);
		for (int i = 0; i < 10; i++) {
			LED_Off(GREEN_LED[i]);
		}
		if (currentGreenLED == 0) {
			flag = 1;
	  } else if (currentGreenLED == 9) {
	  	flag = 0;
  	}
  	if (flag) {
			currentGreenLED += 1;
  	} else currentGreenLED -= 1;
		LED_On(GREEN_LED[currentGreenLED]);
		osDelay(100);
	}
}

void tLED_Green_Pattern2 (void *argument) {
	for (;;) {
		osEventFlagsWait(LED_Green, 0x2, osFlagsWaitAny, osWaitForever);
		for (int i = 0; i < 10; i++) {
		  LED_On(GREEN_LED[i]);
	  }
	}
}

void tLED_Red_Pattern1 (void *argument) {
	for (;;) {
		osEventFlagsWait(LED_Red, 0x1, osFlagsWaitAny, osWaitForever);
		LED_On(RED_LED);
		osDelay(500);
		LED_Off(RED_LED);
		osDelay(500);
	}
}

void tLED_Red_Pattern2 (void *argument) {
  for (;;) {
		osEventFlagsWait(LED_Red, 0x2, osFlagsWaitAny, osWaitForever);
		LED_On(RED_LED);
		osDelay(250);
		LED_Off(RED_LED);
		osDelay(250);
	}
}

//Provides audio output
void tSpecialAudio (void *argument) {
	for (int i = 0; i < (sizeof(special_music) / sizeof(int)); i++) {
			TPM1->MOD = FREQ_TO_MOD(musical_notes[special_music[i]]);
			TPM1_C0V = (FREQ_TO_MOD(musical_notes[special_music[i]])) / 2;
			osDelay(200);
	}
}

void tAudio (void *argument) {
	int currentNote = 0;
	_Bool music = 0;
	for (;;) {
		osMessageQueueGet(musicMessage, &music, NULL, 0);
		if (!music) {
			if (currentNote == sizeof(running_music) / sizeof(int)) {
			  currentNote = 0;
		  } else currentNote++;
		  TPM1->MOD = FREQ_TO_MOD(musical_notes[running_music[currentNote]]);
		  TPM1_C0V = (FREQ_TO_MOD(musical_notes[running_music[currentNote]])) / 2;
		  osDelay(200);
	  } else {
			osThreadNew(tSpecialAudio, NULL, NULL);
			osThreadExit();
		}
	}	
}


int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
  initUART2(BAUD_RATE);
	initGPIO();
  initPWM();
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	mySem = osSemaphoreNew(1, 0, NULL);
	motorMessage = osMessageQueueNew(1, sizeof(uint8_t), NULL);
	LEDMessage = osMessageQueueNew(1, sizeof(_Bool), NULL);
	musicMessage = osMessageQueueNew(1, sizeof(_Bool), NULL);
	LED_Green = osEventFlagsNew(NULL);
	LED_Red = osEventFlagsNew(NULL);
	osThreadNew(tBrain, NULL, NULL);
  osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, NULL);
  osThreadNew(tLED_Green_Pattern1, NULL, NULL);
  osThreadNew(tLED_Green_Pattern2, NULL, NULL);
	osThreadNew(tLED_Red_Pattern1, NULL, NULL);
	osThreadNew(tLED_Red_Pattern2, NULL, NULL);
  osThreadNew(tAudio, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
