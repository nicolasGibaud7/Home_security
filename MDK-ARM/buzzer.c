#include "buzzer.h"

void playPowerOn(TIM_HandleTypeDef* htim){
	playNote(htim, 4, 30);
	playNote(htim, 6, 30);
	playNote(htim, 8, 30);
}
void playNote(TIM_HandleTypeDef* htim, char note, char power){
	__HAL_TIM_SET_AUTORELOAD(htim, 3478 + 100 * note);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, __HAL_TIM_GetAutoreload(htim) * (power/100));
	
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1 );
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1 );
}
