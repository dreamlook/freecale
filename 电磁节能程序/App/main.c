/*!
 * @author     ��С��
 * @version    v1.0
 * @date       2016-11-6
 */

#include "common.h"
#include "include.h"
#include "math.h"
#include "servo.h"

//���
#define S3010_FTM   FTM3
#define S3010_CH    FTM_CH0
#define S3010_HZ    (100)//���

//���
#define MOTOR_FTM   FTM0

#define MOTOR1_PWM  FTM_CH0
#define MOTOR2_PWM  FTM_CH1
#define MOTOR3_PWM  FTM_CH2
#define MOTOR4_PWM  FTM_CH3

#define OLED_SCL  PTC16_OUT
#define OLED_SDA  PTC17_OUT
#define OLED_RST  PTC18_OUT
#define OLED_DC   PTC19_OUT
#define OLED_CS   PTA14_OUT

#define MOTOR_HZ    (20*1000)
#define MOTOR_DUTY  80

#define LB 8

int16 var[5];//������
int16 value[5][LB];//�����˲�
uint16 servo=1450;
uint16 pidduo=0;
uint8 adcm=30;
uint16 adcm2;
uint16 speed_duty1=250;
uint16 speed_duty2=250;
int16 sp;

int16 number;
int16 chaxian=0;

extern struct servo_pd
{
    int p;
    int d;
}servopd;

extern struct servo_error
{
    int present;
    int last;
    int befor;
}error;

extern struct servo_limit
{
    uint16 max;
    uint16 min;
}limit;

//��������
void oled_test();//oled
void PIT0_IRQHandler(void);  //500ms��ʱ���жϷ�����
void uart1_handler(void); //�����жϷ�����
void init();
void motor();
void adc_collect();
void path_judge();
int16 servo_angle(uint16 y);
void encode_init();
/*
 * main����               
 */
void  main(void)
{
    adcm2=adcm*2;
    limit.max = 1610;//1610
    limit.min = 1290;//1290 
    sp=0;
    init();
    while(1)
    {
    }
}
void path_judge()
{
  int16 left_right_add;
  //int16 left_right_reduce;
  int16 zuo_you_reduce;
  int16 zuo_you_add;
  zuo_you_reduce=var[3]-var[4];
  zuo_you_add=var[3]+var[4];
  left_right_add=var[1]+var[2];
  //left_right_reduce=var[1]-var[2];
  servopd.p=3;
  speed_duty1=speed_duty2=320;
  pidduo=30+zuo_you_reduce/8;
  if(var[1]>110&&var[2]>110){
      servopd.p=2;
      servopd.d=30;
      speed_duty1=speed_duty2=300;
  }
  else{
      if(var[4]<50&&var[3]>var[4]){
          pidduo=30+(45-var[3]/10);
          speed_duty1=speed_duty2=270;
          if(var[4]<10&&left_right_add<150)
            speed_duty1=speed_duty2=240;
          if(var[4]<5&&left_right_add<150)
            speed_duty1=speed_duty2=220;
          speed_duty2-=50;
          servopd.d=250;
      }
      else if(var[3]<50&&var[4]>var[3]){
          pidduo=30-(45-var[4]/10);
          speed_duty1=speed_duty2=270;
          if(var[3]<20&&left_right_add<130)
            speed_duty1=speed_duty2=240;
          if(var[3]<5&&left_right_add<130)
            speed_duty1=speed_duty2=220;
          speed_duty1-=50;
          servopd.d=250;
      }
      else{
          servopd.p=3;
          servopd.d=250;
      } 
  }
  if(var[0]>190){
        servopd.p=1;
        servopd.d=50;
        if(zuo_you_reduce>100)
          zuo_you_reduce=100;
        if(zuo_you_reduce<-100)
          zuo_you_reduce=-100;
        pidduo=30+zuo_you_reduce/8;
      }
 } 

void init()
{
  OLED_Init();//oled ��ʼ��
  encode_init();
  pit_init_ms(PIT0, 5);      //��ʼ��PIT0����ʱʱ��Ϊ�� 10ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);
  //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
  enable_irq (PIT0_IRQn);                    //ʹ��PIT0�ж�
  
  set_vector_handler(UART1_RX_TX_VECTORn , uart1_handler); 
  //�� uart1_handler ������ӵ��ж�����������Ҫ�����ֶ�����
  uart_rx_irq_en(UART1);         //������1�����ж�
  
  adc_init(ADC1_SE13);              //ADC��ʼ��
  adc_init(ADC1_SE10);              //ADC��ʼ��
  adc_init(ADC1_SE12);              //ADC��ʼ��
  adc_init(ADC1_SE11);              //ADC��ʼ��
  adc_init(ADC1_SE14);              //ADC��ʼ��
  ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,1450);      //��ʼ�� ��� PWM
  
  ftm_pwm_init(FTM0, FTM_CH1,10000,50); // vcan_port_cfg.h �� ���� FTM
  ftm_pwm_init(FTM0, FTM_CH0,10000,80); //��ʼ�� FTM PWM ��ʹ�� FTM0_CH3��Ƶ��Ϊ200k
  ftm_pwm_init(FTM0, FTM_CH3,10000,50); // vcan_port_cfg.h �� ���� FTM
  ftm_pwm_init(FTM0, FTM_CH2,10000,80); //��ʼ�� FTM PWM ��ʹ�� FTM0_CH3��Ƶ��Ϊ200k
}

void encode_init()
{
    ftm_quad_init(FTM1);                 //������   A8  A9
}
/*
*  ��ʾ��
*/
void oled_test()
{
   uint8 i;
  for(i=0; i<3; i--)
  {
    OLED_P6x8Str(0,0,"left:");Display_number(30,0,var[1]);
    OLED_P6x8Str(60,0,"right:");Display_number(90,0,var[2]);
    
    
    OLED_P6x8Str(0,1,"zhongjian:");Display_number(40,1,var[0]);
    
    OLED_P6x8Str(0,2,"zuo:");Display_number(30,2,var[3]);
    OLED_P6x8Str(60,2,"you:");Display_number(90,2,var[4]);
    
    OLED_P6x8Str(0,3,"servopd.p:");
    Display_number(45,3,servopd.p);
    
    OLED_P6x8Str(0,4,"duty:");
    Display_number(30,4,speed_duty1);
    
    OLED_P6x8Str(0,5,"speed:");
    Display_number(30,5,sp);
  }
}

void motor()
{
  ftm_pwm_duty(FTM0, FTM_CH1,50);     //����ռ�ձ� 
  ftm_pwm_duty(FTM0, FTM_CH0,speed_duty1);     //�����ٶ�
  ftm_pwm_duty(FTM0, FTM_CH3,50);     //����ռ�ձ� 
  ftm_pwm_duty(FTM0, FTM_CH2,speed_duty2);     //�����ٶ�
  ftm_pwm_duty(S3010_FTM, S3010_CH,servo);
}
void adc_collect()
{
    int8 i;
    int8 j;
    int16 sum[5];
    var[1] = adc_once   (ADC1_SE10, ADC_8bit);//ad����ֵ:left ˮƽ
    var[2] = adc_once   (ADC1_SE13, ADC_8bit);//ad����ֵ:right ˮƽ
    var[3] = adc_once   (ADC1_SE11, ADC_8bit);//ad����ֵ:�� б45��
    var[4] = adc_once   (ADC1_SE12, ADC_8bit);//ad����ֵ:�� б45��
    var[0] = adc_once   (ADC1_SE14, ADC_8bit);//ad����ֵ:��ֱ���
    
    for(i=0;i<5; i++)
    {
      sum[i]=0;
    }
    
    /*           �����˲�          */
    for(i=LB-1; i>0; i--)
    {
      for(j=0; j<5; j++)
      {
         sum[j]+=value[j][i-1];
         value[j][i]=value[j][i-1];
      }
    }
    for(j=0; j<5; j++)
    {
      value[j][0]=var[j];
      var[j]=(int)((sum[j]-value[j][LB-2]+var[j])/(LB-1));
    }
}

void PIT0_IRQHandler(void)
{
    adc_collect();              //���ݲɼ�
    sp = -ftm_quad_get(FTM1);  //������
    ftm_quad_clean(FTM1);
    oled_test();                //��ʱ��ʾ���� 
    path_judge();               //·��ʶ�� 
    servo=servo_angle(pidduo);  //pd���ڶ��
    motor();                    //������ �� �����ռ�ձ� 
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
    
}

/*!
 *  @brief      UART1�����жϷ�����
 *  @since      v5.0
 *  @warning    �˺�����Ҫ�û������Լ�������ɣ�����������ṩһ��ģ��
 *  Sample usage:       set_vector_handler(UART1_RX_TX_VECTORn , uart1_test_handler);    //�� uart3_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void uart1_handler(void)
{
    char ch;
    uart_getchar (VCAN_PORT,&ch);   //�ȴ�����һ�����ݣ����浽 ch��
    if(ch=='a')
    {
      //servopd.p-=0.01;
      //printf("servo:%d\n",servo);
      //servoing();
    uart_putchar(VCAN_PORT, ch);  
    }
    if(ch=='b')
    {
      //servopd.d+=0.5;
      //printf("servo:%d\n",servo);
      //servoing();
    } 
}
