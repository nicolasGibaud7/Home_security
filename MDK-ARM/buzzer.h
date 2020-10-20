#include "stm32f4xx_hal.h"

void playPowerOn(TIM_HandleTypeDef* htim);

// note [0 - 10 ]
// power [0 - 100]
void playNote(TIM_HandleTypeDef* htim, char note, char power);
