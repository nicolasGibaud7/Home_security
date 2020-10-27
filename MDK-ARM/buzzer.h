#include "stm32f4xx_hal.h"

void playPowerOn(TIM_HandleTypeDef* htim);
void playPowerOff(TIM_HandleTypeDef* htim);
void playWrongPin(TIM_HandleTypeDef* htim);
void playRightPin(TIM_HandleTypeDef* htim);
void playTrigerred(TIM_HandleTypeDef* htim);
void playEngaged(TIM_HandleTypeDef* htim);

void baseTrigSound(TIM_HandleTypeDef* htim);
// note [0 - 10 ]
// power [0 - 100]
void playNote(TIM_HandleTypeDef* htim, char note, char power, short lenght);
