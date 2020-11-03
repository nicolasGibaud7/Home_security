#include "buzzer.h"

void playPowerOn(TIM_HandleTypeDef* htim){
	playNote(htim, 8, 30, 300);
	playNote(htim, 6, 35, 300);
	playNote(htim, 4, 40, 300);
}

void playPowerOff(TIM_HandleTypeDef* htim){
	playNote(htim, 4, 40, 300);
	playNote(htim, 6, 35, 300);
	playNote(htim, 8, 30, 300);
}

void playWrongPin(TIM_HandleTypeDef* htim){
	playNote(htim, 8, 30, 150);
	HAL_Delay(100);
	playNote(htim, 10, 30, 650);
}

void playRightPin(TIM_HandleTypeDef* htim){
	playNote(htim, 3, 30, 150);
	HAL_Delay(100);
	playNote(htim, 1, 30, 650);
}

void playTrigerred(TIM_HandleTypeDef* htim){
	for(char i = 0; i < 3; i++){
		baseTrigSound(htim);
		HAL_Delay(200);
	}
	
	HAL_Delay(500);
	
	for(char i = 0; i < 3; i++){
		baseTrigSound(htim);
		HAL_Delay(200);
	}
}

void baseTrigSound(TIM_HandleTypeDef* htim){
	playNote(htim, 2, 30, 80);
	HAL_Delay(15);
	playNote(htim, 2, 35, 80);
	HAL_Delay(15);
	playNote(htim, 2, 40, 80);
	HAL_Delay(15);
	playNote(htim, 2, 45, 80);
}

void playEngaged(TIM_HandleTypeDef* htim){
	playNote(htim, 5, 100, 90);
	HAL_Delay(20);
	playNote(htim, 2, 100, 90);
}

void playNote(TIM_HandleTypeDef* htim, char note, char power, short lenght){
	__HAL_TIM_SET_AUTORELOAD(htim, 3478 + 100 * note);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, __HAL_TIM_GetAutoreload(htim) * (power/100.0));
	
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1 );
	HAL_Delay(length);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1 );
	HAL_Delay(7);
}
