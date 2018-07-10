
#include "TIME2.h"

static void TIM2_NVIC_Configuration(void);
static void TIM2_Configuration(int16_t tim);

extern void main_loop(void);


/*��ʱ��2��ʼ������ʱ ntime  ms*/
void time_2_init(int16_t ntime)
{
	TIM2_NVIC_Configuration();
	TIM2_Configuration(ntime);
	
}

/*�ж�����Ϊ tim  ms
ע��TIM_Period��һ��16λ�޷����������65535��Ҳ����tim*1000����ܳ���65535
Ҳ����tim����ܳ���65
*/
static void TIM2_Configuration(int16_t tim)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
	
    TIM_TimeBaseStructure.TIM_Period = tim*1000;	//�Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);	//ʱ��Ԥ��Ƶ�� 72M/72      
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);			// �������жϱ�־ 
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);		//ʹ�ܶ�ʱ���ж�
    TIM_Cmd(TIM2, ENABLE);	// ����ʱ��    
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);	//�ȹرյȴ�ʹ��  
}


/* TIM2�ж����ȼ����� */
static void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
	
	main_loop();
	 	
}
