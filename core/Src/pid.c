#include "pid.h"

 
PID pid; //存放PID算法所需要的数据
void PID_Init()            
{
    pid.choose_model = MODEL_PID;
    
    pid.T=330;                //采样周期，定时器使用1ms，则最小执行PID的周期为330ms
    
  pid.Sv=280;                //用户设定值
    pid.Kp=0.5;                //比例系数
  pid.Ti=180;            //积分时间
    pid.Td=1;                    //微分时间
    pid.OUT0=0;                //一个维持的输出
    
    pid.pwmcycle = 330;    //PWM的周期
}
    
void PID_Calc()  //pid计算
{
    float DelEk;            //本次和上次偏差，最近两次偏差之差
    float ti,ki;
    float td;
    float kd;
    float out;
 
    if(pid.Tdata < (pid.T))  //最小计算周期未到
     {
            return ;
     }
    pid.Tdata = 0;
    pid.Ek=pid.Sv-pid.Pv;               //得到当前的偏差值
    pid.Pout=pid.Kp*pid.Ek;          //比例输出
 
    pid.SEk+=pid.Ek;                    //历史偏差总和
 
    DelEk=pid.Ek-pid.Ek_1;              //最近两次偏差之差
 
    ti=pid.T/pid.Ti;
    ki=ti*pid.Kp;
 
    pid.Iout=ki*pid.SEk*pid.Kp;  //积分输出    /*注意：上面程序中多了个pid.Kp,原程序中有，请自动删除，正确的应该是pid.Iout=ki*pid.SEK */
 
    td=pid.Td/pid.T;
 
    kd=pid.Kp*td;
 
  pid.Dout=kd*DelEk;                //微分输出
 
    
     switch(pid.choose_model)
     {
         case MODEL_P:     out= pid.Pout;                                                printf("使用P运算\r\n") ;
             break;
         
         case MODEL_PI:  out= pid.Pout+ pid.Iout;                            printf("使用PI运算\r\n") ;
             break;
                 
         case MODEL_PID: out= pid.Pout+ pid.Iout+ pid.Dout;        printf("使用PID运算\r\n") ;
             break;
     }
    printf("PID算得的OUT:\t%d\r\n",(int)out) ;
 
 //
 
        /*判断算出的数是否符合控制要求*/
     if(out>pid.pwmcycle)        //不能比PWM周期大，最大就是全高吗
     {
        pid.OUT=pid.pwmcycle;
     }
     else if(out<0)             //值不能为负数
     {
        pid.OUT=pid.OUT0; 
     }
     else 
     {
        pid.OUT=out;
     }
     printf("实际输出使用的pid.OUT:\t%d\r\n",(int)pid.OUT) ;
     pid.Ek_1=pid.Ek;  //更新偏差
     
     Turn_Angle((int)pid.OUT);        //输出PWM
     
}