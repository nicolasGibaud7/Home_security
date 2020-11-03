#include "Alarm.h"

char check_pin(Alarm* alarm){
	for(char i = 0; i < 4; i++){
		if(alarm->correct_pin[i] != alarm->user_current_pin[i])
			return 0;
	}
	return 1;
}

void init_alarm(Alarm* alarm, TIM_HandleTypeDef timer){
	alarm->alarm_state = OFF;
	alarm->starting_cpt = 0;
	alarm->missed_pin_cpt = 0;
	
	alarm->timer = timer;
	
	alarm->user_current_pin[0] = 0;
	alarm->user_current_pin[1] = 0;
	alarm->user_current_pin[2] = 0;
	alarm->user_current_pin[3] = 0;
	
	alarm->correct_pin[0] = BTN_BLUE_Pin;
	alarm->correct_pin[1] = BTN_YELLOW_Pin;
	alarm->correct_pin[2] = BTN_YELLOW_Pin;
	alarm->correct_pin[3] = BTN_BLUE_Pin;
	
	setPowerButtonLed(GPIO_PIN_SET);
}

void change_state(Alarm* alarm, Alarm_state new_state){
	alarm->alarm_state = new_state;
	
	switch(new_state){
		case OFF: 
			setPowerButtonLed(GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin | LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&alarm->timer);
			break;
		
		case STARTING:
			setPowerButtonLed(GPIO_PIN_SET);
			alarm->starting_cpt = 0;
			HAL_TIM_Base_Start_IT(&(alarm->timer));
			HAL_GPIO_WritePin(GPIOD,LED_ORANGE_Pin, GPIO_PIN_SET);
			break;
		
		case ENGAGED:
				setPowerButtonLed(GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LED_ORANGE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&alarm->timer);
			break;
		
		case TRIGERRED:
			setPowerButtonLed(GPIO_PIN_RESET);
			alarm->triggered_cpt = 0;
			alarm->missed_pin_cpt = 0;
			reset_user_pin(alarm);
		
			HAL_GPIO_WritePin(GPIOD, LED_RED_Pin | LED_ORANGE_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Start_IT(&(alarm->timer));
			break;
		
		case ALARM:
			setPowerButtonLed(GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_ORANGE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Stop_IT(&alarm->timer);
			break;
	}
}

void reset_user_pin(Alarm* alarm){
	alarm->user_current_pin[0] = 0;
	alarm->user_current_pin[1] = 0;
	alarm->user_current_pin[2] = 0;
	alarm->user_current_pin[3] = 0;
}

void setPowerButtonLed(GPIO_PinState pinstate){	
	HAL_GPIO_WritePin(GPIOD, BTN_POWER_LED_Pin, pinstate);
}
