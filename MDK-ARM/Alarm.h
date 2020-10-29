#include "stm32f4xx_hal.h"
#include "main.h"

#define do_nothing() __asm{NOP}
#define STARTING_TIME 10 //seconde
#define MAX_PIN_TIME 60 //seconde

typedef enum{
	OFF,
	STARTING,
	ENGAGED,
	TRIGERRED,
	ALARM
}Alarm_state;

typedef enum{
	OK,
	DOOR_TRIGERRED
}Door_state;

typedef struct{
	Alarm_state alarm_state;
	char starting_cpt;
	char triggered_cpt;
	char missed_pin_cpt;
	
	uint16_t correct_pin[4];
	uint16_t user_current_pin[4];
	
	TIM_HandleTypeDef timer;
}Alarm;

char check_pin(Alarm* alarm);
void init_alarm(Alarm* alarm, TIM_HandleTypeDef timer);

void change_state(Alarm* alarm, Alarm_state new_state);

void reset_user_pin(Alarm* alarm);

void setPowerButtonLed(GPIO_PinState pinstate);
