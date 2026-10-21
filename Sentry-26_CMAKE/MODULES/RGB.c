#include "RGB.h"

RGB_enum RGB_color;
//����ԭ��ͼ��LED��������VSS��������IO�ڣ�����ʹ�ÿ�©�������ʼ������Ϊ�ߵ�ƽ��IO����Ϊ���迹����ʼ״̬Ϊ��
/*PB7 R 
  PB4 G
  PB3 B*/
//���RGB��Ϊ������ʹ��ʱ����һ��RGB_enum���͵�ö�ٱ������ɣ�����ö�ٱ�������������ͬ����ɫ
void RGB_action(RGB_enum RGB)
{
    switch(RGB)
    {
        case RGB_OFF:
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        break;                    
                                  
        case RGB_RED:             
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        break;                    
                                  
        case RGB_GREEN:           
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        break;                    
                                  
        case RGB_BLUE:            
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
        break;                    
                                  
        case RGB_YELLOW:          
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        break;                    
                                  
        case RGB_CYAN:            
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
        break;                    
                                  
        case RGB_MAGENTA:         
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
        break;                    
                                  
        case RGB_WHITE:           
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
        break;                    
                                  
        default:                  
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        break;
            
    }
}


