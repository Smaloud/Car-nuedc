#include "gpio.h"
# include "menu.h"
#include "OLED.h"
int menu1(){
	int menu1_flag = 1;
	OLED_ShowString(1,3,"Control");
	OLED_ShowString(2,3,"Encoder");
	OLED_ShowString(3,3,"UART");
	
	uint8_t keyValue;
	while(1){
	keyValue = read_Key();
		
	if(keyValue==1){ //控制键: 选择下一项
		menu1_flag++;
		if(menu1_flag==4) menu1_flag=1;
	}
	if(keyValue==2){ //选择键: 选择此项
		OLED_Clear();
		return menu1_flag;
		
	}
	
	switch(menu1_flag)
		{
			case 1:
			{
				OLED_ShowString(2,1,"*");
				OLED_ShowString(3,1," ");
				OLED_ShowString(4,1," ");

			}break;
			case 2:
			{
				OLED_ShowString(2,1," ");
				OLED_ShowString(3,1,"*");
				OLED_ShowString(4,1," ");
			}break;
			case 3:
			{
				OLED_ShowString(2,1," ");
				OLED_ShowString(3,1," ");
				OLED_ShowString(4,1,"*");				
			}break;
		}
	}

}

//key = read_key();

uint8_t read_Key(void){
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == 1){
		HAL_Delay(50);
		while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == 1){
			return 1;
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3) == 1){
		HAL_Delay(50);
		while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3) == 1){
			return 2;
		}
	}
	return 0;
}










