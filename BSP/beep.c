#include "beep.h"
#include "main.h"
const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

const Sound_tone_e Mavic_Startup_music[4][Startup_Success_music_len] = 
{
  {So5M, So5M,Mi3M,Mi3M, So5M,So5M,Mi3M,Mi3M,   So5M,Mi3M,Re2M,Mi3M, So5M,So5M,So5M,So5M,Silent},/*¹¤³Ì*/
	{So5M, Mi3M,So5M,Mi3M, So5M,Mi3M,Do1M,Silent, Re2M,Fa4M,Mi3M,Re2M, So5M,So5M,So5M,So5M,Silent},/*Ó¢ÐÛ*/
	{Mi3M, Re2M,Do1M,Re2M, Mi3M,Mi3M,Mi3M,Silent, Re2M,Re2M,Re2M,Silent, Mi3M,So5M,So5M,Silent,Silent},/*ÉÚ±ø*/
	{Do1M, Re2M,Mi3M,Do1M, So5M,Silent,Silent,Silent, La6M,La6M,Do1H,La6M, So5M,So5M,So5M,So5M,Silent},/*²½±ø*/
	// So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
};



//play the start up music


void buzzer_on(uint16_t psc, uint16_t pwm)
{
//		TIM_SetAutoreload(TIM12,psc);
//    TIM_SetCompare1(TIM12, pwm);
	
	  __HAL_TIM_SetAutoreload(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm);
}
void buzzer_off(void)
{
 //   TIM_SetCompare1(TIM12, 0);
	
	  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
}

void Sing_Startup_music(int chioose){
	int i = 0;
	for(i = 0; i<Startup_Success_music_len; i++){
	
	if(Mavic_Startup_music[chioose][i] != Silent){
		buzzer_on(tone_tab[Mavic_Startup_music[chioose][i]],tone_tab[Mavic_Startup_music[chioose][i]]/2);
	}
	if(Mavic_Startup_music[chioose][i] == Silent){
	
		buzzer_off();
	}
		HAL_Delay(250);
	}
		buzzer_off();
}
