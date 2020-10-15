#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

//Decodes the data from the Serial Port and performs the necessary action
void tBrain (void *argument) {
}

//Controls motors
void tMotorControl (void *argument) {
}

//Controls LEDs
void tLED (void *argument) {
	for (;;) {}
}

//Provides audio output
void tAudio (void *argument) {
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(tBrain, NULL, NULL);
  osThreadNew(tMotorControl, NULL, NULL);
  osThreadNew(tLED, NULL, NULL);
  osThreadNew(tAudio, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
