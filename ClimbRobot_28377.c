/*
 * ClimbRobot_28377.c
 *
 *  Created on: 07/21/2017
 *  Modified on : 10/19/2018
 *      Author: Liu Zhaoming
 *            : Feng Jinglun
 */

#include "F28x_Project.h"

#define EPWM3_TIMER_TBPRD  0xFFFF  // Period register
#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0
#define EPWM8_TIMER_TBPRD  62500  // Period register
#define EPWM8_MID_CMP      62500*0.925
#define EPWM8_LOW_CMP      62500*0.94
#define EPWM8_HIG_CMP      62500*0.91
#define EPWM8_MID_ESC      62500*0.95 // 1ms 0.9--2ms

#define RESULTS_BUFFER_SIZE 256
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;
volatile Uint16 bufferFull;

float varVolt =  .543221;  // variance determined using excel and reading samples of raw sensor data
float varProcess = .5e-6;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmCMPA; //for giving the speed
    Uint16 EPwmCMPB; //for giving the speed
    float EPwmSpeed; //for giving the speed
}EPWM_INFO;
EPWM_INFO epwm7_info,epwm8_info,epwm9_info,epwm10_info;
float EPWM_Duty_Cycle = 1;

uint16_t EPwm1TimerIntCount = 0;
char msg_rec[10];

Uint16 state = 10;
Uint16 operator = 0;
Uint16 pre_state = 1;
Uint16 pre_operator = 0x00;
Uint16 emmergency = 0x0F;
int sendstate = 0;

void scib_echoback_init(void);
void scib_fifo_init(void);
void scib_xmit(int a);
void scib_msg(char *msg);

void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCEpwm(Uint16 channel);
interrupt void adca1_isr(void);

void update_compare(EPWM_INFO*);
void InitEPwm7Example(void);
__interrupt void epwm7_isr(void);
void InitEPwm8Example(void);
void InitEPwm9Example(void);
void InitEPwm10Example(void);

float filteredData(float voltage);

void state_machine_processing();

void main(void)
 {
    Uint16 ReceivedCharc;
    char *msg;
    int msg_start;
    int msg_state;
    int msg_i;
    msg_start = 0;
    msg_state = 0;
    msg_i = 0;

   InitSysCtrl();

   DELAY_US(1000000); //等待微妙  wait some microseconds

   InitGpio();

   CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
   CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
   CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;
   CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;

   InitEPwm10Gpio(); // for motion motor1
   InitEPwm9Gpio(); // for motion motor2
   InitEPwm8Gpio(); // for suction motor and servo motor
   InitEPwm7Gpio();

   GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 5); //RXDB
   GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 5); //TXDB
   GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(90, GPIO_MUX_CPU1, 6); //RXDC
   GPIO_SetupPinOptions(90, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(89, GPIO_MUX_CPU1, 6); //TXDC
   GPIO_SetupPinOptions(89, GPIO_OUTPUT, GPIO_ASYNC);

   DINT;

   InitPieCtrl();

   IER = 0x0000;
   IFR = 0x0000;

   InitPieVectTable();

   EALLOW; // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM7_INT = &epwm7_isr;
   PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
   EDIS;   // This is needed to disable write to EALLOW protected registers

   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // ePWM Time Base Clock sync: When set PWM time bases of all the PWM modules start counting
   EDIS;

   InitEPwm10Example(); // for motion motor1
   InitEPwm9Example(); // for motion motor2
   InitEPwm8Example(); // for suction motor and sevor
   InitEPwm7Example();

   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

   ConfigureADC();

   ConfigureEPWM();

   SetupADCEpwm(0);

   IER |= M_INT1;                              // 外部中断 external interruption
   IER |= M_INT3;

   for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
   {
       AdcaResults[resultsIndex] = 0;
   }
   resultsIndex = 0;
   bufferFull = 0;

   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

   EALLOW;

   PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4

   EINT;  // Enable Global interrupt INTM
   ERTM;  // Enable Global realtime interrupt DBGM

   scib_fifo_init();  // for communication of ESP
   scib_echoback_init();

   msg = "AT+CWJAP=\"Robotics Lab\",\"r0b0tzl4b2\r\n\0"; // if you change a router, then change the name and password
   scib_msg(msg);
   DELAY_US(5000000); //等待微妙 wait for some microseconds

   msg = "AT+CIPMUX=1\r\n\0";
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   msg = "AT+CIPSERVER=1,8888\r\n\0";
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   msg = "AT+CIFSR\r\n\0";
   scib_msg(msg);

   EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
   EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode


   for(;;)
   {
       while(ScibRegs.SCIFFRX.bit.RXFFST != 0)
       {
           ReceivedCharc = ScibRegs.SCIRXBUF.bit.SAR;
           switch(ReceivedCharc)
           {
           case 0xFF:
               msg_start = 1;
               break;
           case 0xFE:
               msg_start = 2;
               break;
           }

           switch(msg_start)
           {
           case 1:
               msg_rec[msg_i] = ReceivedCharc;
               msg_i++;
               break;
           case 2:
               if(msg_i == 4)
               {
                   msg_state = 1;//收到有效输入 Valid input data
                   msg_rec[msg_i] = ReceivedCharc;
                   msg_i = 0;//把i清零 set i with 0
               }
               else
               {
                   msg_state = 0;//没有收到有效输入 invalid inputs
                   msg_i = 0;//把i清零 set i with 0
               }
               break;
           }

           if(msg_i > 4)
           {
               msg_i = 0;
               msg_start = 0;
               msg_state = 0;
           }

       }

       //Setting the State Machine
       if(msg_state == 1)
       {
           msg_state = 0;
           state = msg_rec[1];
           operator = msg_rec[2];
       }

       state_machine_processing(state, operator);

   }

}

void scib_echoback_init()
{
    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x001A;

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

void scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF.all =a;
}

void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all = 0xE040;
    ScibRegs.SCIFFRX.all = 0x2044;
    ScibRegs.SCIFFCT.all = 0x0;
}

void InitEPwm10Example()
{
   EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm10Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
   EPwm10Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm10Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm10Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm10Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm10Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   EPwm10Regs.CMPA.bit.CMPA = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set compare A value
   EPwm10Regs.CMPB.bit.CMPB = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set Compare B value

   EPwm10Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
   EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                   // up count

   EPwm10Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
   EPwm10Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                   // up count
   EPwm10Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm10Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm10Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

   epwm10_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
   epwm10_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
                                                   // counter
   epwm10_info.EPwmRegHandle = &EPwm10Regs;          // Set the pointer to the
                                                   // ePWM module
   epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
   epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
   epwm10_info.EPwmSpeed = 0.5;
}

void InitEPwm9Example()
{
       EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
       EPwm9Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
       EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
       EPwm9Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
       EPwm9Regs.TBCTR = 0x0000;                  // Clear counter
       EPwm9Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
       EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV2;

       EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
       EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

       EPwm9Regs.CMPA.bit.CMPA = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set compare A value
       EPwm9Regs.CMPB.bit.CMPB = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set Compare B value

       EPwm9Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
       EPwm9Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                       // up count

       EPwm9Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
       EPwm9Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                       // up count
       EPwm9Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
       EPwm9Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
       EPwm9Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

       epwm9_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
       epwm9_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
                                                        // counter
       epwm9_info.EPwmRegHandle = &EPwm9Regs;          // Set the pointer to the
                                                       // ePWM module
                                                       // CMPA/CMPB values
       epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
       epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
       epwm9_info.EPwmSpeed = 0.5;
}

void InitEPwm8Example()
{
       EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
       EPwm8Regs.TBPRD = EPWM8_TIMER_TBPRD;       // Set timer period
       EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
       EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
       EPwm8Regs.TBCTR = 0x0000;                  // Clear counter
       EPwm8Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
       EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;

       EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
       EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

       EPwm8Regs.CMPA.bit.CMPA = EPWM8_MID_ESC;      // Set compare A value
       EPwm8Regs.CMPB.bit.CMPB = EPWM8_MID_CMP;      // Set Compare B value

       EPwm8Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
       EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                       // up count

       EPwm8Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
       EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                       // up count
       epwm8_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
       epwm8_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
       //epwm8_info.EPwmTimerIntCount = 0;               // Zero the interrupt
                                                       // counter
       epwm8_info.EPwmRegHandle = &EPwm8Regs;          // Set the pointer to the
                                                       // ePWM module
                                                       // CMPA/CMPB values
       epwm8_info.EPwmCMPA = EPWM8_MID_ESC;
       epwm8_info.EPwmCMPB = EPWM8_MID_CMP;
       epwm8_info.EPwmSpeed = 0.5;
}

void InitEPwm7Example()
{
   EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm7Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
   EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm7Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm7Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   EPwm7Regs.CMPA.bit.CMPA = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set compare A value
   EPwm7Regs.CMPB.bit.CMPB = EPWM3_TIMER_TBPRD * EPWM_Duty_Cycle;      // Set Compare B value

   EPwm7Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
   EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                   // up count

   EPwm7Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
   EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                   // up countevent
   EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
   EPwm7Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm7Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

   epwm7_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
   epwm7_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
                                                   // counter
   epwm7_info.EPwmRegHandle = &EPwm7Regs;          // Set the pointer to the
                                                   // ePWM module
   epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
   epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
}

__interrupt void epwm7_isr(void)
{
    update_compare(&epwm7_info);
    EPwm7Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void update_compare(EPWM_INFO *epwm_info)
{

    if(epwm_info->EPwmCMPA != epwm_info->EPwmRegHandle->CMPA.bit.CMPA || epwm_info->EPwmCMPB != epwm_info->EPwmRegHandle->CMPB.bit.CMPB)
    {
        epwm_info->EPwmRegHandle->CMPA.bit.CMPA = epwm_info->EPwmCMPA;
        epwm_info->EPwmRegHandle->CMPB.bit.CMPB = epwm_info->EPwmCMPB;
    }
   return;
}

void state_machine_processing()
{
    switch(state)
    {
    case 0://moving state
        switch(operator & emmergency)
        {
        case 0x00://halt
            epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
            epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            update_compare(&epwm9_info);
            update_compare(&epwm10_info);
            break;
        case 0x01://move forward
            epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD * (1 - epwm10_info.EPwmSpeed);
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * (1 - epwm9_info.EPwmSpeed);
            epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            update_compare(&epwm9_info);
            update_compare(&epwm10_info);
            break;
        case 0x02://move backward
            epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD * (1 - epwm10_info.EPwmSpeed);
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * (1 - epwm9_info.EPwmSpeed);
            epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0;
//            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
            update_compare(&epwm9_info);
            update_compare(&epwm10_info);
            break;
        case 0x04://turn left
            epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD * (1 - epwm10_info.EPwmSpeed);
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * (1 - epwm9_info.EPwmSpeed);
            epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * (1 - epwm7_info.EPwmSpeed);
//            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
            update_compare(&epwm9_info);
            update_compare(&epwm10_info);
            break;
        case 0x08://turn right
            epwm10_info.EPwmCMPB = EPWM3_TIMER_TBPRD * (1 - epwm10_info.EPwmSpeed);
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * (1 - epwm9_info.EPwmSpeed);
            epwm10_info.EPwmCMPA = EPWM3_TIMER_TBPRD;
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
//            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * (1 - epwm7_info.EPwmSpeed);
//            epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD;
            update_compare(&epwm9_info);
            update_compare(&epwm10_info);
            break;
        }
        break;
    case 1://suction motor
        switch(operator)
        {
        case 0x00://stop
            epwm8_info.EPwmCMPA = EPWM8_MID_ESC;
            update_compare(&epwm8_info);
            pre_state = 1;
            pre_operator = 0x00;
            break;
        case 0x01://start
            epwm8_info.EPwmCMPA = 62500 * (0.95-0.05 * epwm8_info.EPwmSpeed);
            update_compare(&epwm8_info);
            pre_state = 1;
            pre_operator = 0x01;
            break;
        }
        break;
    case 4://servo
        switch(operator % 3)
        {
        case 0:
            epwm8_info.EPwmCMPB = EPWM8_MID_CMP;
            update_compare(&epwm8_info);
            state = 0;
            operator = 0;
            break;
        case 1:
            epwm8_info.EPwmCMPB = EPWM8_LOW_CMP;
            update_compare(&epwm8_info);
            state = 0;
            operator = 0;
            break;
        case 2:
            epwm8_info.EPwmCMPB = EPWM8_HIG_CMP;
            update_compare(&epwm8_info);
            state = 0;
            operator = 0;
            break;
        }
        break;
    case 5://setting speed
        switch(operator)
        {
        case 1://wheel speed
            epwm10_info.EPwmSpeed = msg_rec[3];
            epwm10_info.EPwmSpeed = epwm10_info.EPwmSpeed/100;
            epwm9_info.EPwmSpeed = msg_rec[3];
            epwm9_info.EPwmSpeed = epwm9_info.EPwmSpeed/100;
//            epwm7_info.EPwmSpeed = msg_rec[3];
//            epwm7_info.EPwmSpeed = epwm7_info.EPwmSpeed/100;
            state = 0;
            operator = 0;
            break;
        case 2://suction speed
            epwm8_info.EPwmSpeed = msg_rec[3];
            epwm8_info.EPwmSpeed = epwm8_info.EPwmSpeed/100;
            state = pre_state;
            operator = pre_operator;
            break;
        }
        break;
    case 6://LED
        switch(operator)
        {
            case 1:
                EALLOW;
                GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;         // Drives LED LD2 on controlCARD
                GpioCtrlRegs.GPAPUD.bit.GPIO12=0;   //pullup enable
                EDIS;
                GpioDataRegs.GPADAT.bit.GPIO12 = 0;     // Turn on LED
                DELAY_US(1000 * 500);                   // ON delay
                GpioDataRegs.GPADAT.bit.GPIO12 = 1;     // Turn off LED
                DELAY_US(1000 * 500);
                break;
            case 2:
                EALLOW;
                GpioCtrlRegs.GPAPUD.bit.GPIO12=1;   //pullup enable
                GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;         // Drives LED LD2 on controlCARD
                EDIS;
                GpioDataRegs.GPADAT.bit.GPIO12 = 1;         // Turn off LED
                break;
        }
        break;

    }
    return;
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// ConfigureEPWM - Configure EPWM SOC and compare values
//
void ConfigureEPWM(void)
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;     // Set compare A value to 2048 counts
    EPwm1Regs.TBPRD = 0x1000;             // Set period to 4096 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(Uint16 channel)
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void)
{
    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
    if(RESULTS_BUFFER_SIZE <= resultsIndex)
    {
        resultsIndex = 0;
        bufferFull = 1;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

float filteredData(float voltage)
{
  // kalman process
  Pc = P + varProcess;
  G = Pc/(Pc + varVolt);    // kalman gain
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(voltage-Zp)+Xp;   // the kalman estimate of the sensor voltage
  return Xe;
}
