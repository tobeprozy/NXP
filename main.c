#include "common.h"
#include "include.h"

//关于电感
//adc经过判断和计算后的偏差
uint16 AD[8],AD_v[8];
uint16  max_v[8]={2516,2619,0,2573,2573,0,0,0},min_v[8]={39,50,50,42,50,50,50,50};                                 //找到6个电感中的最大最小值，电感标定采集值
int16 cline_err;
uint16 adc_av(ADCn_Ch_e adcn_ch, ADC_nbit bit);
void Read_ADC(void);
void adc_max(void);
void to_one(void);
void adc_init1(void);
uint8 stop=2;
uint8 ms=0;
//关于编码器
//记录编码器采集的脉冲数
//设定速度
//pid计算后应该给予的速度电压
//当前速度获取程序 //编码器初始化
float djc;
uint8 count=0;
uint16  cspeed;
uint16  speed;
uint16  uspeed;
uint16  speedh=0;
uint32  angle;
uint32  lastangle;
void bmq_get(void);
//虚拟示波器程序
void vcan_sendware1(uint8 *wareaddr, uint32 waresize);
//以下为多种pid的运算方案
uint16 motor_PID(uint16 speed, uint16 cspeed);
float duoji_PID3(float cline_err);              //变系数PD舵机控制(位置式) KD可变，

////////////////控制参数 ////////////////
int  DuoP=24;   //
int  DuoD=2;
int  timecount;
int  speedcnt;
int  speedkp;
int  dianjispeed;
int errz[8];
int flag=0;
unsigned char  Dir_last=0;
int  dir_error=0,dis_error;
int  Calculate_Length;
int  DirectionErr[9]={0};
int disgy_AD_val_1,disgy_AD_val_2;
float Error_Delta;
unsigned short servPWMDuty; //当前舵机值
int8 flag1=0;
int8 flag2=0;
int8 flag3=0;
int8 flags=0;
int8 diuxian=0;
int8 zhidao=0;


void Push_And_Pull(int *buff,int len,int newdata);
float Slope_Calculate(uint8 begin,uint8 end,int *p);
void adjduoji();
void adjspeed(uint16 speed, uint16 cspeed);
///////////////////////////以下是正式进入调车程序///////////////////////////////




//清除中断标志位,一定要清除，不然就一直在中断
void pit_ch1_irq(void)
{
  ftm_pwm_duty(FTM2,FTM_CH5,3000);
  float o_pwm;
  
  Read_ADC();  
  adc_max();
  to_one();
  //AD[0]=AD[0];
  djc=(float)(AD[4]-AD[0]+3);
  angle=duoji_PID3(djc)+4800;    
  
  /*******正常道路*********/
        if(abs(djc)<12)
          uspeed=300;
        else if(25>abs(djc)>=12)
          uspeed=150;
        else
          uspeed=0;
        
        
        
         
//      while(flag1==0&&AD_v[3]>1950&&AD_v[1]<650)
//              
//      {  Read_ADC();  
//        // adc_max();
//         to_one();
//         
//        if(AD_v[3]>1950&&AD_v[1]<300)
//        {    
//        // djc=16*(float)(AD[3]-AD[1]);
//          angle=5110;
//         ftm_pwm_duty(FTM2,FTM_CH4,angle);
//         flags=1;
//        } 
//     
//        if(AD_v[0]<100)
//        { flags=2;
//          flag1=1;
//          ftm_pwm_duty(FTM2,FTM_CH4,4800); 
//        }
//        
//        
//        djc=(float)(AD[0]-AD[4]+3);
//        angle=duoji_PID3(djc)+4800;  
//        ftm_pwm_duty(FTM2,FTM_CH4,angle);  //正常走
//      }
//      
//      
//       while(flag2==0&&AD_v[1]>1650&&AD_v[3]<650)
//              
//      {  Read_ADC();  
//        // adc_max();
//         to_one();
//         
//        if(AD_v[1]>1650&&AD_v[5]<650)
//        {    
//        // djc=16*(float)(AD[3]-AD[1]);
//          angle=4400;
//         ftm_pwm_duty(FTM2,FTM_CH4,angle);
//          flags=3;
//        }      
//        
//        if(AD_v[4]<40&&AD_v[3]<90)
//        {  flags=4; 
//           flag2=1;
//          ftm_pwm_duty(FTM2,FTM_CH4,4800); 
//        }
//      }
//      
//      
//        flags=0;
//      
      ftm_pwm_duty(FTM2,FTM_CH4,angle);  //正常走
       
      
       
       
       
      if(500<AD_v[5]<1000&&500<AD_v[4]<1000&&4900<angle<4700&&AD_v[1]<50)
       {
       flag2=0;
       }
      
      if(600<AD_v[5]<900&&600<AD_v[4]<900&&4900<angle<4700&&AD_v[1]<50&&abs(djc)<5) 
      {  
        flag1=0; 
      }
      
      
       if(PTC6_IN==0)
       {     
         ftm_pwm_duty(FTM2,FTM_CH5,uspeed);  
       
       }
       else
       {
         
        
        while(PTC6_IN);
        
          stop=stop-1;
          if(stop==0)
          { 
            while(1)
            {
          ftm_pwm_duty(FTM2,FTM_CH4,4800);  
          ftm_pwm_duty(FTM2,FTM_CH5,10000); 
            }
       }
       }
       
   PIT_Flag_Clear(PIT1);

}



 void main(void)
{


//////////////////以下是开关，显示类初始化/////////////////////////////////////
//四个led灯初始化，用于指示当前进入的函数，虽然可能没什么用
     led_init(LED3);
     gpio_init (PTC6, GPI,0);
////////////////////以下是与中断定义有关的初始化////////////////////////////////
     
     pit_init_ms(PIT1,5);                                   //  定时 500ms
     enable_irq(PIT_CH1_IRQn);                             // 使能PIT_CH1中断
//   pit_init_ms(PIT0, 3);
//   enable_irq(PIT_CH0_IRQn);
//   NVIC_SetPriority(PIT_CH1_IRQn,0);
//   NVIC_SetPriority(PIT_CH0_IRQn,1);

//////////////////////以下是adc初始化程序///////////////////////////////////////
     adc_init1();
//////////////////////以下是lcd初始化程序///////////////////////////////////////
    
///////////////////以下是与电机和舵机有关的初始化///////////////////////////////
 //     ftm_pulse_init(FTM1, FTM_PS_1, TCLK1);
       
      ftm_pwm_init(FTM2, FTM_CH5,1000,10000);     //PTB5    //FTM0_PRECISON  精度入口
      ftm_pwm_init(FTM2, FTM_CH4,300,4100);      //PTB4     //4254基本上是90度角


     EnableInterrupts; 

 while(1)
    {
     //   ftm_pwm_duty(FTM2,FTM_CH4,4500); 
    }

}






 /////////////////////以上是主要程序/////////////////////////////////////

/////////////////////以下进入函数定义区////////////////////////////////




////////////////////////虚拟示波器核心函数////////////////////////////////////
void vcan_sendware1(uint8 *wareaddr, uint32 waresize)
//示波器核心函数   前面表示数组，长度必须与通道对应，avr[5] 必须对应通道5，
//类型必须大致对应，char 对应 char 或者unsigned char。才能显示
{
#define CMD_WARE     3


     uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令


     uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令


     uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送前命令


     uart_putbuff(VCAN_PORT, wareaddr, waresize);    //发送数据


     uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //发送后命令

}


/////////////////////////////编码器脉冲获取////////////////////////////
void bmq_get(void)
{
     //con=PTE5_IN;
     
     if(count==4)
     {
       count=0;
       cspeed = ftm_pulse_get(FTM0);
       ftm_pulse_clean(FTM0) ; 
     }
 
   
          //cspeed是当前速度，全局变量
}



/////////////////////////////adc采集和处理程序///////////////////////////

void adc_init1(void)                          //电感的定义和获取
{
adc_init(ADC0_SE0);      //PTA0               //必须初始化一个值，否则不显示状态，情况目前未知
adc_init(ADC0_SE1);     //PTA1

adc_init(ADC0_SE2);     //PTA6
adc_init(ADC0_SE3);     //PTA7

adc_init(ADC0_SE12);     //PTF4               //警告是由于没有用ADC0_SE0之类的名称，没用已定义的宏定义
adc_init(ADC0_SE13);     //PTF5

adc_init(ADC0_SE14);    // PTF6
adc_init(ADC0_SE15);     //PTA7

}



/////////////////////////////////取平均函数///////////////////////////////////
//端口名，精度，取5次做平均
uint16 adc_av(ADCn_Ch_e adcn_ch, ADC_nbit bit)
{
   int i;
   float AD_av[5];

   uint16 sum=0;
for(i=0;i<5;i++)
  {
  AD_av[i]=adc_once(adcn_ch,bit);
  }
  for(i=0;i<5;i++)
  {
  sum+=AD_av[i];
  }
sum=sum/4;
return sum;
}
/////////////三个数组分别表示多次采集数组，平均值数值，采集之和数组////////////
//连续采集5X4次电感值
void Read_ADC(void)
{
     int16  i,j,k;
     uint16  AD_duo[8][5],ad_sum[6],temp;

     for(i=0;i<5;i++)
     {
   AD_duo[0][i]=adc_av(ADC0_SE0,ADC_12bit);
   AD_duo[1][i]=adc_av(ADC0_SE1,ADC_12bit);

   AD_duo[2][i]=adc_av(ADC0_SE2,ADC_12bit);
   AD_duo[3][i]=adc_av(ADC0_SE3,ADC_12bit);

   AD_duo[4][i]=adc_av(ADC0_SE12,ADC_12bit);
   AD_duo[5][i]=adc_av(ADC0_SE13,ADC_12bit);

   AD_duo[6][i]=adc_av(ADC0_SE14,ADC_12bit);
   AD_duo[7][i]=adc_av(ADC0_SE15,ADC_12bit);
     }

//下面进行冒泡排序，为的是取掉一个最大值，去掉一个最小值
//第一个循环中，i=0到5，代表了6个电感
//j=0到4，五个数据排序
    for(i=0;i<8;i++)
     {
        for(j=0;j<5;j++)
        {
           for(k=0;k<5-j;k++)
           {
              if(AD_duo[i][k] > AD_duo[i][k+1])    //前面的比后面的大  则进行交换
              {
                 temp = AD_duo[i][k+1];
                 AD_duo[i][k+1] = AD_duo[i][k];
                 AD_duo[i][k] = temp;
              }
           }
        }
     }

     for(i=0;i<8;i++)    //求中间三项的和
     {
        ad_sum[i] = AD_duo[i][1] + AD_duo[i][2] + AD_duo[i][3];
        AD_v[i] = ad_sum[i] / 3;
     }

}

 void adc_max(void)
{
       uint8  j;

       for(j=0;j<8;j++)
           {
               if(AD_v[j] > max_v[j]+20)
               {
                   max_v[j]=AD_v[j];
               }
           if(AD_v[j]<min_v[j])
               {
                   min_v[j]= AD_v[j];
                }
           }
}

///////////////////////对平均值进行归一化处理////////////////////////////////
void to_one(void)
{  uint8  i;
   float sensor_to_one[8];

    for(i=0;i<8;i++)
    {
       sensor_to_one[i] = (float)(AD_v[i] - min_v[i])/(float)(max_v[i] - min_v[i]);
       if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
       if(sensor_to_one[i]>1.0)  sensor_to_one[i]=1.0;

       AD[i] = (uint16)(100 * sensor_to_one[i]);     //AD[i]为归一化后的值  范围为0-100
    }

}






float duoji_PID3(float cline_err)                //变系数PD舵机控制(位置式) KD可变，
{
volatile static float err=0,last_err=0,derr=0,l_last_err,dderr;
volatile static float M_PWM=0;
volatile static float Kd,Kp,Ki;

    err=cline_err;  
    //Push_And_Pull(errz,8,err); 
    //derr=10*Slope_Calculate(0,8,errz);//偏差50代表中间
    derr=err-last_err;                         //derr dderr赋初值
    //dderr=err-2*last_err+l_last_err;
    
   /******DIU XIAN*******/
//     if(AD_v[0]<100&&AD[4]<100||AD_v[0]<80||AD_v[4]<80)
//     {
//       
//        if(AD[0]<AD[4])
//          M_PWM=420;
//        if(AD[0]>AD[4])
//          M_PWM=-430;
//       
//
//        
//        diuxian=1;
//        zhidao=0;
//     }   
//     
//   else    
//   {
       
      if(AD[0]>19&&AD[4]>19)    //直道
                  
     { if(err<0)
        Kp=1.8;
     else  if(err>0)
        Kp=1.6;
     zhidao=1;
     diuxian=0;   
      }
   // if(abs(err)>35)
      Kp=2+exp(0.011*abs(err));

      
    
  //  if(abs(err)<800)
     Kd=100-exp(0.045*abs(err));
 // else Kd++;

   M_PWM=err*Kp+derr*Kd;                     //注意这里是非增量式PID
 
 //  }  

     
     
     
   l_last_err=last_err;                      //记录上上次偏差last_err-err;/7记录上次偏差值servo_control (M_PWM);最后赋值给舵机函数
   last_err=err;
   
   /*************XIAN WEI***********/
   if(M_PWM>450)
     M_PWM=450;
   if(M_PWM<-480)
     M_PWM=-480;  
   
  return M_PWM;
  }


/////////////////最小二乘法拟合斜率，求电感插值导数/////////////////
float Slope_Calculate(uint8 begin,uint8 end,int *p)    
{
   int xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
  {
    result=((end-begin)*xysum*1.0-xsum*ysum*1.0)/((end-begin)*x2sum*1.0-xsum*xsum*1.0);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

///////////////////退栈函数///////////////
void Push_And_Pull(int *buff,int len,int newdata)
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}
