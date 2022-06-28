#include "pid.h"

 
PID pid; //���PID�㷨����Ҫ������
void PID_Init()            
{
    pid.choose_model = MODEL_PID;
    
    pid.T=330;                //�������ڣ���ʱ��ʹ��1ms������Сִ��PID������Ϊ330ms
    
  pid.Sv=280;                //�û��趨ֵ
    pid.Kp=0.5;                //����ϵ��
  pid.Ti=180;            //����ʱ��
    pid.Td=1;                    //΢��ʱ��
    pid.OUT0=0;                //һ��ά�ֵ����
    
    pid.pwmcycle = 330;    //PWM������
}
    
void PID_Calc()  //pid����
{
    float DelEk;            //���κ��ϴ�ƫ��������ƫ��֮��
    float ti,ki;
    float td;
    float kd;
    float out;
 
    if(pid.Tdata < (pid.T))  //��С��������δ��
     {
            return ;
     }
    pid.Tdata = 0;
    pid.Ek=pid.Sv-pid.Pv;               //�õ���ǰ��ƫ��ֵ
    pid.Pout=pid.Kp*pid.Ek;          //�������
 
    pid.SEk+=pid.Ek;                    //��ʷƫ���ܺ�
 
    DelEk=pid.Ek-pid.Ek_1;              //�������ƫ��֮��
 
    ti=pid.T/pid.Ti;
    ki=ti*pid.Kp;
 
    pid.Iout=ki*pid.SEk*pid.Kp;  //�������    /*ע�⣺��������ж��˸�pid.Kp,ԭ�������У����Զ�ɾ������ȷ��Ӧ����pid.Iout=ki*pid.SEK */
 
    td=pid.Td/pid.T;
 
    kd=pid.Kp*td;
 
  pid.Dout=kd*DelEk;                //΢�����
 
    
     switch(pid.choose_model)
     {
         case MODEL_P:     out= pid.Pout;                                                printf("ʹ��P����\r\n") ;
             break;
         
         case MODEL_PI:  out= pid.Pout+ pid.Iout;                            printf("ʹ��PI����\r\n") ;
             break;
                 
         case MODEL_PID: out= pid.Pout+ pid.Iout+ pid.Dout;        printf("ʹ��PID����\r\n") ;
             break;
     }
    printf("PID��õ�OUT:\t%d\r\n",(int)out) ;
 
 //
 
        /*�ж���������Ƿ���Ͽ���Ҫ��*/
     if(out>pid.pwmcycle)        //���ܱ�PWM���ڴ�������ȫ����
     {
        pid.OUT=pid.pwmcycle;
     }
     else if(out<0)             //ֵ����Ϊ����
     {
        pid.OUT=pid.OUT0; 
     }
     else 
     {
        pid.OUT=out;
     }
     printf("ʵ�����ʹ�õ�pid.OUT:\t%d\r\n",(int)pid.OUT) ;
     pid.Ek_1=pid.Ek;  //����ƫ��
     
     Turn_Angle((int)pid.OUT);        //���PWM
     
}