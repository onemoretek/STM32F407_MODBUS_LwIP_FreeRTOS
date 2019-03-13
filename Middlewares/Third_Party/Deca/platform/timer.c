#include "timer.h"
#include "usart.h"
#include "dw_main.h"


//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM2_Int_Init(uint16 arr,uint16 psc)
{
#if 0	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,���������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx	
#endif	
}
//��ʱ��3�жϷ������
//void TIM2_IRQHandler(void)   //TIM3�ж�
//{
//#if 0
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
//		{
//			TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
//			
//			if(ERROR_FLAG<=1000)
//        ERROR_FLAG++;
//			
//		}
//#endif 		
//}



////ͨ�ö�ʱ��3�жϳ�ʼ��
////����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
////arr���Զ���װֵ��
////psc��ʱ��Ԥ��Ƶ��
////����ʹ�õ��Ƕ�ʱ��3!
//void TIM3_Int_Init(uint16 arr,uint16 psc)
//{
//#if 0	
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
//	
//	//��ʱ��TIM3��ʼ��
//	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
// 
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,���������ж�

//	//�ж����ȼ�NVIC����
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


//	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
//#endif
//}
////��ʱ��3�жϷ������
//void TIM3_IRQHandler(void)   //TIM3�ж�
//{
//#if 0	
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
//		{
//			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
//			if(time3_usart1<=200)
//        time3_usart1++;
//			if(time3_usart2<=200)
//        time3_usart2++;
//			if(time3_usart3<=200)
//        time3_usart3++;
//		}
//#endif		
//}




//void TIM4_Int_Init(uint16 arr,uint16 psc)
//{
//#if 0	
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
//	
//	//��ʱ��TIM3��ʼ��
//	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
// 
//	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,���������ж�

//	//�ж����ȼ�NVIC����
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


//	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx					 
//#endif
//}
////��ʱ��3�жϷ������
//void TIM4_IRQHandler(void)   //TIM3�ж�
//{
//#if 0	
//	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
//		{
//			TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
//			
//						if(Time_time4_Cuo<0xFFFF)Time_time4_Cuo++;
//			else Time_time4_Cuo=0;
//			
//		}
//#endif		
//}






