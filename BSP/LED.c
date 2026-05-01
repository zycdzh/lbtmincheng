#include "main.h"
#include "led.h"

void R_LED_ON(void){

	HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);

}

void G_LED_ON(void){

	 HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);

}

void R_LED_OFF(void){

	HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);

}

void G_LED_OFF(void){

	 HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);

}

void LED_GREEN_TOGGLE(void){
	HAL_GPIO_TogglePin(G_LED_GPIO_Port, G_LED_Pin);

}

void LED_RED_TOGGLE(void){
	HAL_GPIO_TogglePin(R_LED_GPIO_Port, R_LED_Pin);

}
