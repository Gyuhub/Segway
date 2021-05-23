/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int cnt;
int cnt2;
int cnt3;
int cnt4;
int time=3;
int Pin_State;
int Pin_State_L;
volatile int cma=0;
volatile int idx=0;
volatile int iRoll;
volatile int iYaw;
int Linearity=1;
int stayflag=0;
int spinflagL=0;
int spinflagML=0;
int spinflagR=0;
int spinflagMR=0;
int moveflagF=0;
int moveflagMF=0;
int moveflagB=0;
int moveflagMB=0;
int stableflag=0;
int tiltflag=0;
volatile int JRefThetaDot;
volatile int JThetaDot;
volatile int JTargetRoll;

volatile double dRoll;
volatile double dYaw;
volatile double PreYaw;
volatile double ErrPYaw;
volatile double RefYawDot;
volatile double RollDot;
volatile double RollDDot;
volatile double YawDot;
volatile double pYawDot;
volatile double YawDDot;
volatile double RefYawDot;
volatile double ThetaDot;
volatile double ThetaDDot;
volatile double ThetaDot2;
volatile double ThetaDDot2;
volatile double RefThetaDot;
volatile double PThetaDot;
volatile double TargetRoll=0.0;
volatile double ErrRoll;
volatile double PreErrRoll;
volatile double DeltaErrRoll=0;
volatile double TargetYaw=0.0;
volatile double ErrYawDot;
volatile double ErrSumYawDot;
volatile double PIYawDot;
volatile double ErrYawDDot;
volatile double ErrSumYawDDot;
volatile double PIYawDDot;
volatile double TargetSpeed;
volatile double TargetSpeed2;
volatile double TargetAccel;
volatile double TargetAccel2;
volatile double ErrVelocity;
volatile double ErrSpeed;
volatile double ErrAccel;
volatile double ErrSumSpeed=0;
volatile double ErrSumAccel=0;
volatile double ErrSpeed2;
volatile double ErrAccel2;
volatile double ErrSumSpeed2=0;
volatile double ErrSumAccel2=0;
double Kp=0.7; // Last Value = 0.56
double Ki=0.000000000000000001; // Last Value = 0.000000000000000001 0.000000000000000001;
double Kd=1.6; // Last Value = 1.18
double YawKp=1.8;//15
double YawKi=0.000000000000000001;
double KKp=0.045; // 0.007
double KKKp=0.1;//0.5
volatile double PD_out_Roll;
volatile double IP_out_AngleSpeed;
volatile double IP_out_AngleAccel;
volatile double IP_out_AngleSpeed2;
volatile double IP_out_AngleAccel2;

double m=0.15;
double M=2.7;
double R=0.065;
double L=0.1115;
double W=0.33;
double Jw=0.0012675;
double Jpi;
double Jm=0.0001;
double sin[46] = {0.000,0.0175,0.0349,0.0523,0.0698,0.0872,0.1045,0.1219,0.1392,0.1564,0.1736,0.1908,0.2079,0.2250,0.2419,0.2588,0.2756,0.2924,0.3090,0.3256,0.3420,0.3584,0.3746,0.3907,0.4067,0.4226,0.4384,0.4540,0.4695,0.4848,0.5,0.5150,0.5299,0.5446,0.5592,0.5736,0.5878,0.6018,0.6157,0.6293,0.6428,0.6561,0.6691,0.6820,0.6947,0.7071};
double cos[46] = {1.000,0.9998,0.9994,0.9986,0.9976,0.9962,0.9945,0.9925,0.9903,0.9877,0.9848,0.9816,0.9781,0.9744,0.9703,0.9659,0.9613,0.9563,0.9511,0.9455,0.9397,0.9336,0.9272,0.9205,0.9135,0.9063,0.8988,0.8910,0.8829,0.8746,0.8660,0.8572,0.8480,0.8387,0.8290,0.8192,0.8090,0.7986,0.7880,0.7771,0.7660,0.7547,0.7431,0.7314,0.7193,0.7071};
double n=1/14;
double Rm=0.68;
double Kt=0.039;
double Kb=0.062;
double fm=0.00041;
double fw=0;
volatile int Voltage=0;
volatile int Voltage2=0;
volatile int PreVoltage=0;
volatile int PreSpinVoltage=0;
volatile int SpinVoltage=0;
volatile int spinVoltR=0;
volatile int spinVoltL=0;
double alpha = 244.1025641;
double beta = 0.01521102041;
double gamma = 0.000001020408163;
double delta = 0.001327983193;
double varepsilon = 0.001327983193;
double mu = 0.01956825;
double rho = 0.01956722959;
double iota = 0.06517197529;
double kappa = 0.0006639915966;
double lambda = 0.0008069136746;

uint16_t Motor_CCR_R = 0;
uint16_t Motor_CCR_L = 0;
int32_t encoder_cnt_R;
int32_t encoder_R;
int32_t encoder_cnt_L;
int32_t encoder_L;
int32_t Pencoder;
volatile double Distance_L;
volatile double AngleSpeed_L;
volatile double AngleAccel_L;
volatile double Speed_L;
volatile double Speed;
volatile double Pre_Speed;
volatile double Angle_L;
volatile double Pulse_L;
volatile double Distance_R;
volatile double AngleAccel_R;
volatile double Speed_R;
volatile double Angle_R;
volatile double Pulse_R;
volatile double PAngle;

#define RX_BUFFER_SIZE          50
#define RX_BLUETOOTH_SIZE       2
#define PI              3.1415926535897932384626433832795028841

uint8_t tx_buffer[15] = "/r/n";
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_bluetooth[RX_BLUETOOTH_SIZE];
char rx_roll[7];
char rx_yaw[7];

uint8_t tx_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim9)
  {
    //===================Saturation=============================================
    if(abs(iRoll)>60.0)
    {
      if( rx_buffer[49]=='S')
      {
        for(int i=0; i<sizeof(rx_buffer); i++)
        {
          rx_buffer[i] = 0;
        }
      }
      else
      {
        TIM4->CCR4 = 0; TIM4->CCR1 = 0;
        while(1);
      }
    }
    cnt++;
    cnt2++;
    cnt3++;
    cnt4++;
    //===================Encoder================================================
    encoder_cnt_L = TIM2->CNT;
    encoder_cnt_R = TIM3->CNT;
    if(encoder_cnt_L>30000) {encoder_L = encoder_cnt_L - 30000;}
    else {encoder_L = encoder_cnt_L + 30000;}
    if(encoder_cnt_R>30000) {encoder_R = encoder_cnt_R - 30000;}
    else {encoder_R = encoder_cnt_R + 30000;}
    //===================BT serial==============================================
    if(cnt3==100) // 100ms Bluetooth receive
    {
    //===================BT Flag detect=========================================
      HAL_UART_Receive_DMA(&huart6, rx_bluetooth, RX_BLUETOOTH_SIZE);
      if((rx_bluetooth[0]=='R'))
      {
        spinflagR=1;
        spinflagMR=0;
        spinflagL=0;
        spinflagML=0;
        moveflagF=0;
        moveflagMF=0;
        moveflagB=0;
        moveflagMB=0;
        stayflag=0;
      }
      else if ((rx_bluetooth[0]=='L'))
      {
        spinflagR=0;
        spinflagMR=0;
        spinflagL=1;
        spinflagML=0;
        moveflagF=0;
        moveflagMF=0;
        moveflagB=0;
        moveflagMB=0;
        stayflag=0;
      }
      else if ((rx_bluetooth[0]=='F'))
      {
        spinflagR=0;
        spinflagMR=0;
        spinflagL=0;
        spinflagML=0;
        moveflagF=1;
        moveflagMF=0;
        moveflagB=0;
        moveflagMB=0;
        stayflag=0;
      }
      else if ((rx_bluetooth[0]=='B'))
      {
        spinflagR=0;
        spinflagMR=0;
        spinflagL=0;
        spinflagML=0;
        moveflagF=0;
        moveflagMF=0;
        moveflagB=1;
        moveflagMB=0;
        stayflag=0;
      }
//=======================BT Weighted Flag=======================================
      else if ((rx_bluetooth[0]=='M'))
      {
        if((rx_bluetooth[1]=='R'))
        {
          spinflagR=0;
          spinflagMR=1;
          spinflagL=0;
          spinflagML=0;
          moveflagF=0;
          moveflagMF=0;
          moveflagB=0;
          moveflagMB=0;
          stayflag=0; 
        }
        else if((rx_bluetooth[1]=='L'))
        {
          spinflagR=0;
          spinflagMR=0;
          spinflagL=0;
          spinflagML=1;
          moveflagF=0;
          moveflagMF=0;
          moveflagB=0;
          moveflagMB=0;
          stayflag=0;
        }
        else if((rx_bluetooth[1]=='F'))
        {
          spinflagR=0;
          spinflagMR=0;
          spinflagL=0;
          spinflagML=0;
          moveflagF=0;
          moveflagMF=1;
          moveflagB=0;
          moveflagMB=0;
          stayflag=0;
        }
        else if((rx_bluetooth[1]=='B'))
        {
          spinflagR=0;
          spinflagMR=0;
          spinflagL=0;
          spinflagML=0;
          moveflagF=0;
          moveflagMF=0;
          moveflagB=0;
          moveflagMB=1;
          stayflag=0;
        }
      }
      else 
      {
        spinflagR=0;
        spinflagMR=0;
        spinflagL=0;
        spinflagML=0;
        moveflagF=0;
        moveflagMF=0;
        moveflagB=0;
        moveflagMB=0;
        stayflag=1;
      }
      cnt3=0;
    }
//=======================Speed P Control========================================    
    if(cnt4==27)
    {
      Pencoder = (encoder_L-30000); // 제어주기동안 회전한 encoder pulse
      PAngle = Pencoder/(1064*2*PI); // deltaPulse/분해능*2PI = 제어주기동안 회전한 각도 theta[rad]
      
      PThetaDot = PAngle*1000/27; // 제어주기동안 움직인 각속도 rad/s
      pYawDot = dYaw*1000/27;
      if(moveflagF||moveflagB||moveflagMF||moveflagMB)
      {
        ErrVelocity = RefThetaDot - PThetaDot;
        TargetRoll = KKp*ErrVelocity;
        JRefThetaDot = RefThetaDot;
        JTargetRoll = TargetRoll;
      }
    //===================Spin P Control=========================================
      else if(spinflagR||spinflagL||spinflagMR||spinflagML)
      {
        TargetRoll=0;
        
        ErrPYaw = RefYawDot - pYawDot;
        TargetYaw = dYaw + KKKp*ErrPYaw;
      }
        cnt4=0;
    }
//=======================Outer Loop=============================================
    if(cnt2==9) // 9ms PD control
    {
    //===================EBIMU receive==========================================
    //===================EBIMU reset buffer=====================================
      HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
      if( (rx_buffer[0] != 49) | (rx_buffer[1] != 48) | (rx_buffer[2] != 48) | (rx_buffer[3] != 45) | (rx_buffer[4] != 48) )
      {
        for(int i=0; i<sizeof(rx_buffer); i++)
        {
          rx_buffer[i] = 0;
        }
      }
      else if( rx_buffer[45]=='S')
      {
        for(int i=0; i<sizeof(rx_buffer); i++)
        {
          rx_buffer[i] = 0;
        }
      }
    //===================EBIMU receive==========================================  
      for(int j=4; j<=34; j++)
      {
        
        if(cma==1 && (rx_buffer[j]!=','))
        {
          rx_roll[j-6] = rx_buffer[j];
        }
        if(cma==3 && (rx_buffer[j]!=','))
        {
          rx_yaw[idx] = rx_buffer[j];
          idx++;
        }
        if(rx_buffer[j]==',')
        {
          cma++;
        }
      }
      dRoll = atof(rx_roll);
      iRoll = atoi(rx_roll);
      
      dYaw = atof(rx_yaw);
      iYaw = atoi(rx_yaw);
      
      cma=0;
      idx=0;
    //===================Flag Saturation========================================
      if(iRoll>20 || iRoll<-20)
      {
        spinflagR=0;
        spinflagMR=0;
        spinflagL=0;
        spinflagML=0;
        moveflagF=0;
        moveflagMF=0;
        moveflagB=0;
        moveflagMB=0;
        stayflag=1;
      }
    //===================Flag detect============================================
    //===================Spin Control===========================================
      if(spinflagL)
      {
        RefYawDot = -1.0;
      }
      else if(spinflagR)
      {
        RefYawDot = 1.0;
      }
      else if(spinflagMR)
      {
        RefYawDot = 1.0;
      }
      else if(spinflagML)
      {
        RefYawDot = -1.0;
      }
    //===================Move Control===========================================
      else if(moveflagF)
      {
        KKp = 0.045;
        RefThetaDot = 1.2;
      }
      else if(moveflagB)
      {
        KKp = 0.045;
        RefThetaDot = -1.2;
      }
      else if(moveflagMF)
      {
        KKp = 0.05;
        RefThetaDot = 1.3;
      }
      else if(moveflagMB)
      {
        KKp = 0.05;
        RefThetaDot = -1.3;
      }
      else if(stayflag)
      {
        ErrVelocity=0;
        TargetRoll=0;
        ErrPYaw=0;
        Kp=0.7; // 0.4 0.55
        Ki=0.000000000000000001; // 1.0125 0.000000000000000001
        Kd=1.6; // 0.4 1.15 1.3
      }
      else
      {
        ErrVelocity=0;
        TargetRoll=0;
        ErrPYaw=0;
        Kp=0.7; // 0.4
        Ki=0.000000000000000001; // 1.0125 0.000000000000000001
        Kd=1.6; // 0.4
      }
    //===================PD Position Control====================================
      if (iRoll <= 10) { Linearity = 1; }
      else if (iRoll>10) { Linearity =0; }
      
      ErrRoll = TargetRoll - (dRoll*PI/180);
      DeltaErrRoll = ErrRoll - PreErrRoll;
      PD_out_Roll = Kp*ErrRoll + Kd*DeltaErrRoll;
      
      RollDot = PD_out_Roll*1000/9; // psi dot
      RollDDot = RollDot*1000/9; // psi double dot
      
      PreErrRoll = ErrRoll;
      cnt2=0;
    } // RollDot and RollDDot is computed which are psi_dot and psi_double_dot
//=======================Inner Loop=============================================
    if(cnt==time) // 3ms IP control
    {
    //===================ThetaDot,ThetaDDot=====================================  
      Pulse_L = (encoder_L-30000); // 제어주기동안 회전한 encoder pulse
      Angle_L = Pulse_L/(1064*2*PI); // deltaPulse/분해능*2PI = 제어주기동안 회전한 각도 theta[rad]
      
      Pulse_R = (encoder_R - 30000);
      Angle_R = Pulse_R/(1064*2*PI);
      
      /*//We need not m/s but cm/s
      Distance_L = Angle_L*6.5; // 제어주기동안 움직인 거리 L = theta*r [cm]
      Distance_R = Angle_R*6.5;
      
      Speed_L = Distance_L/time; // 제어주기동안 움직인 속도 v = L/t [cm/s]
      Speed_R = Distance_R/time;*/
      
      ThetaDot = Angle_L*1000/time; // 제어주기동안 움직인 각속도 rad/s
      ThetaDot2 = Angle_R*1000/time;
      
      ThetaDDot = ThetaDot*1000/time; // 제어주기동안 움직인 각가속도 rad/s^2
      ThetaDDot2 = ThetaDot2*1000/time; 
      
    //===================Linearty(when |iRoll|<10)==============================
      if(Linearity)
      {
        //Left
        TargetSpeed = ((beta*ThetaDDot+rho*RollDDot-varepsilon*RollDot)*(-1)/delta+2*Voltage/(alpha*delta));
        TargetAccel = ((rho*RollDDot+delta*ThetaDot-varepsilon*RollDot)*(-1)/beta+2*Voltage/(alpha*beta));
        //Right
        TargetSpeed2 = ((beta*ThetaDDot2+rho*RollDDot-varepsilon*RollDot)*(-1)/delta+2*Voltage/(alpha*delta));
        TargetAccel2 = ((rho*RollDDot+delta*ThetaDot2-varepsilon*RollDot)*(-1)/beta+2*Voltage/(alpha*beta));
      }
      else
      {
        //Left
        TargetSpeed = ((beta*ThetaDDot-gamma*RollDDot-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))*(-1)/delta+2*Voltage/(alpha*delta));
        TargetAccel = ((-gamma*RollDDot+delta*ThetaDot-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))*(-1)/beta+2*Voltage/(alpha*beta));
        //Right
        TargetSpeed2 = ((beta*ThetaDDot2-gamma*RollDDot-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))*(-1)/delta+2*Voltage/(alpha*delta));
        TargetAccel2 = ((-gamma*RollDDot+delta*ThetaDot2-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))*(-1)/beta+2*Voltage/(alpha*beta));
      }// W [rad/s] a [rad/s^2]
    //===================IP Speed Control=======================================
      //Left
      ErrSpeed = TargetSpeed - ThetaDot;
      ErrSumSpeed += ErrSpeed;
      IP_out_AngleSpeed = -Kp*ThetaDot + Ki*ErrSumSpeed; // IP_out_AngleSpeed은 계산을 거친 목표 AngleSpeed W [rad/s]
      
      ErrAccel = TargetAccel - ThetaDDot;
      ErrSumAccel += ErrAccel;
      IP_out_AngleAccel = -Kp*ThetaDDot + Ki*ErrSumAccel; // IP_out_AngleAccel 계산을 거친 목표 AngleAccel a [rad/s^2]
      
      //Right
      ErrSpeed2 = TargetSpeed2 - ThetaDot2;
      ErrSumSpeed2 += ErrSpeed2;
      IP_out_AngleSpeed2 = -Kp*ThetaDot2 + Ki*ErrSumSpeed2; // IP_out_AngleSpeed은 계산을 거친 목표 AngleSpeed W [rad/s]
      
      ErrAccel2 = TargetAccel2 - ThetaDDot2;
      ErrSumAccel2 += ErrAccel2;
      IP_out_AngleAccel2 = -Kp*ThetaDDot2 + Ki*ErrSumAccel2; // IP_out_AngleAccel 계산을 거친 목표 AngleAccel a [rad/s^2]
      
      JThetaDot = IP_out_AngleSpeed;
    //===================Voltage Calculate======================================
      if(Linearity)
      {
        Voltage = (alpha/2*(beta*IP_out_AngleAccel+rho*RollDDot+delta*IP_out_AngleSpeed-varepsilon*RollDot));
        Voltage2 = (alpha/2*(beta*IP_out_AngleAccel2+rho*RollDDot+delta*IP_out_AngleSpeed2-varepsilon*RollDot));
      }
      else
      {
        Voltage = (alpha*(beta*IP_out_AngleAccel-gamma*RollDDot+delta*IP_out_AngleSpeed-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))/2);
        Voltage2 = (alpha*(beta*IP_out_AngleAccel2-gamma*RollDDot+delta*IP_out_AngleSpeed2-varepsilon*RollDot+mu*((-1)*sin[iRoll]*RollDot*RollDot+cos[iRoll]*RollDDot))/2);
      }
    //===================Spin left==============================================
      if(spinflagL||spinflagML)
      {
    //===================Spin Control===========================================
        YawDot = dYaw/time;
        YawDDot = YawDot/time;
      
        ErrYawDot = TargetYaw/time - YawDot;
        ErrSumYawDot += ErrYawDot;
        PIYawDot = YawKp*ErrYawDot + YawKi*ErrSumYawDot; // IPYawDot 계산을 거친 목표 PhiDot
        
        ErrYawDDot = (TargetYaw/time)/time - YawDDot;
        ErrSumYawDDot += ErrYawDDot;
        PIYawDDot = YawKp*ErrYawDDot + YawKi*ErrSumYawDDot; // IPYawDDot 계산을 거친 목표 PhiDDot
        
        SpinVoltage = (iota*PIYawDDot + kappa*PIYawDot)/lambda;
        
        //When Vr and Vl are different
        //spinVoltR = (SpinVoltage+2*Voltage2)/2;
        //spinVoltL = (2*Voltage-SpinVoltage)/2;
        spinVoltR = SpinVoltage;
        spinVoltL = SpinVoltage;
        
        PreSpinVoltage = spinVoltR;
    //===================Spin Direction Control=================================
        if(spinVoltL<0) { spinVoltL = spinVoltL*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);} // SpinVoltL이 positive이면 반시계방향으로 바퀴가 회전
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET);}
        if(spinVoltR<0) { spinVoltR = spinVoltR*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);} // SpinVoltR이 positive이면 반시계방향으로 바퀴가 회전
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);}
      }
    //===================Spin right=============================================
      else if(spinflagR||spinflagMR)
      {
    //===================Spin Control===========================================
        YawDot = dYaw/time;
        YawDDot = YawDot/time;
      
        ErrYawDot = TargetYaw/time - YawDot;
        ErrSumYawDot += ErrYawDot;
        PIYawDot = YawKp*ErrYawDot + YawKi*ErrSumYawDot; // IPYawDot 계산을 거친 목표 PhiDot
        
        ErrYawDDot = (TargetYaw/time)/time - YawDDot;
        ErrSumYawDDot += ErrYawDDot;
        PIYawDDot = YawKp*ErrYawDDot + YawKi*ErrSumYawDDot; // IPYawDDot 계산을 거친 목표 PhiDDot
        
        SpinVoltage = (iota*PIYawDDot + kappa*PIYawDot)/lambda;        
        
        //When Vr and Vl are diffenet
        //spinVoltL = (SpinVoltage+2*Voltage2)/2;
        //spinVoltR = (2*Voltage-SpinVoltage)/2;
        spinVoltR = SpinVoltage;
        spinVoltL = SpinVoltage;

        PreSpinVoltage = spinVoltL;
    //===================Spin Direction Control=================================
        if(spinVoltL<0) { spinVoltL = spinVoltL*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET);} // SpinVoltL이 positive이면 반시계방향으로 바퀴가 회전
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET);}
        if(spinVoltR<0) { spinVoltR = spinVoltR*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);} // SpinVoltR이 positive이면 반시계방향으로 바퀴가 회전
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);}
      }
    //===================Voltage Direction Control==============================
      else
      {
        if(Voltage<0) { Voltage = Voltage*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);} // Voltage가 negative이면 positive로 바꿔준다
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);}
        if(Voltage2<0) { Voltage2 = Voltage2*(-1.0); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);} // Voltage가 negative이면 positive로 바꿔준다
        else { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);}
      }
      PreVoltage = Voltage;
    //===================Spin Voltage Saturation================================
      if(spinflagR||spinflagL||spinflagMR||spinflagML) {if(spinVoltR>3500.0)  {spinVoltR = 3500.0; } if(spinVoltL>3500.0) {spinVoltL = 3500.0;}}// 회전 안정성을 위해 PWM의 최댓값이 1000 그 이상이면 최대로 고정
      else {if(Voltage>4000.0)  {Voltage = 4000.0; } if(Voltage2>4000.0) {Voltage2 = 4000.0;} } // PWM의 최댓값이 3500 그 이상이면 최대로 고정
    //===================Motor PWM input========================================
      //MOTOR CCR CODE
      if(spinflagR||spinflagL||spinflagMR||spinflagML) { TIM4->CCR4 = spinVoltL; TIM4->CCR1 = spinVoltR;}
      else if (moveflagF||moveflagB||moveflagMF||moveflagMB) {TIM4->CCR4 = Voltage; TIM4->CCR1 = Voltage2; }
      else if (stayflag) {TIM4->CCR4 = Voltage; TIM4->CCR1 = Voltage2; }
      else { TIM4->CCR4 = Voltage; TIM4->CCR1 = Voltage2; }
      cnt=0;
      TIM3->CNT=0;
      TIM2->CNT=0;
    }
    //TIM4->CCR1 = Motor_CCR_R;
    //TIM4->CCR4 = Motor_CCR_L;
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);
    
    Pin_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
    Pin_State_L = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9);
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_4);
  
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart6, rx_buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4200-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 839;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
