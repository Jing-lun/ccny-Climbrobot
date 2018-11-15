/*
 * ClimbRobot_28377.c
 *
 *  Created on: 2017年7月21日
 *      Author: Liuzhaoming
 */



//
// Included Files
//
#include "F28x_Project.h"

//
// Defines ePWM
//
// for motion driver
#define EPWM3_TIMER_TBPRD  3125  // Period register
#define EPWM3_MAX_CMPA     3125*0.5
#define EPWM3_MIN_CMPA     3125*0.5
#define EPWM3_MAX_CMPB     3125*0.5
#define EPWM3_MIN_CMPB     3125*0.5

#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0
// for suction ESC and servo
#define EPWM8_TIMER_TBPRD  62500  // Period register
#define EPWM8_MID_CMP      62500*0.925
#define EPWM8_LOW_CMP      62500*0.94
#define EPWM8_HIG_CMP      62500*0.91
#define EPWM8_MID_ESC      62500*0.95 // 1ms 0.9--2ms
//
// Defines ADC
//
#define RESULTS_BUFFER_SIZE 256

//
// Globals ADC
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;
volatile Uint16 bufferFull;
//
// Globals kalman variables
//
float varVolt =  .543221;  // variance determined using excel and reading samples of raw sensor data
float varProcess = .5e-6;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

//
// Globals ePWM
//
float EPwm_CMP = 0.08;
typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmCMPA; //for giving the speed
    Uint16 EPwmCMPB; //for giving the speed
    float EPwmSpeed; //for giving the speed
}EPWM_INFO;
EPWM_INFO epwm7_info,epwm8_info,epwm9_info;

uint16_t    EPwm1TimerIntCount = 0;
char msg_rec[10];

//
//  Globals State Machine
//
Uint16 state = 10;
Uint16 operator = 0;
Uint16 pre_state = 1;
Uint16 pre_operator = 0x00;
Uint16 emmergency = 0x0F;
int sendstate = 0;

//
// Function Prototypes
//
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);

void scib_echoback_init(void);
void scib_fifo_init(void);
void scib_xmit(int a);
void scib_msg(char *msg);

void scic_echoback_init(void);
void scic_fifo_init(void);
void scic_xmit(int a);
void scic_msg(char *msg);


//
// Function Prototypes ADC
//
void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCEpwm(Uint16 channel);
interrupt void adca1_isr(void);
//
//  Function Prototypes Epwm
//
void InitEPwm7Example(void);
__interrupt void epwm7_isr(void);
void update_compare(EPWM_INFO*);
void InitEPwm8Example(void);
//__interrupt void epwm8_isr(void);
void InitEPwm9Example(void);

interrupt void xint1_isr(void);
//
// Function Prototypes Kalman filter
//
float filteredData(float voltage);
//
//	Function Prototypes State machine
//
void state_machine_processing(Unit16 state, Unit16 operator);


//
// Main
//
void main(void)
{
    Uint16 ReceivedCharb;
    Uint16 ReceivedCharc;
    char *msg;
    //char msgsend[5];
    int msg_start;
    int msg_state;
    int msg_i;
    msg_start = 0;
	msg_state = 0;
	msg_i = 0;
	float PressureValue = 0;


//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xS_SysCtrl.c file.
//
   InitSysCtrl();

   DELAY_US(1000000); //等待微妙  wait some microseconds
//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xS_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//



  InitGpio();


//
// Enable PWM7 PWM 的时钟寄存器 clock register for PWM 
//
   CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
   CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
   //CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;

//
// For this case just init GPIO pins for ePWM7
// These functions are in the F2837xS_EPwm.c file
//
   InitEPwm7Gpio(); // for motion motor
   InitEPwm8Gpio(); // for suction motor and servo motor
   //InitEPwm9Gpio(); // for suction motor



//
// For this example, only init the pins for the SCI-A port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xS_Gpio.c file.
//
//   GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5); //RXDA
//   GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PUSHPULL);
//   GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5); //TXDA
//   GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 5); //RXDB
   GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 5); //TXDB
   GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(90, GPIO_MUX_CPU1, 6); //RXDC
   GPIO_SetupPinOptions(90, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(89, GPIO_MUX_CPU1, 6); //TXDC
   GPIO_SetupPinOptions(89, GPIO_OUTPUT, GPIO_ASYNC);

//
// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xS_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
//
   InitPieVectTable();
//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
   EALLOW; // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM7_INT = &epwm7_isr;
   PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
   PieVectTable.XINT1_INT = &xint1_isr;  //外部中断 external interruption 
   EDIS;   // This is needed to disable write to EALLOW protected registers

//
// For this example, only initialize the ePWM
//
   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // ePWM Time Base Clock sync: When set PWM time bases of all the PWM modules start counting
   EDIS;

   InitEPwm7Example(); // for motion motor
   InitEPwm8Example(); // for suction motor and sevor
   //InitEPwm9Example(); // for suction motor

   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

//
// Configure the ADC and power it up
//
   ConfigureADC();

//
// Configure the ePWM
//
   ConfigureEPWM();

//
// Setup the ADC for ePWM triggered conversions on channel 0
//
   SetupADCEpwm(0);

//
// Step 4. User specific code, enable interrupts:
//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
   IER |= M_INT1;                              // 外部中断 external interruption 
   IER |= M_INT3;
   //IER |= M_INT5;

//
// Initialize results buffer
//
   for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
   {
	   AdcaResults[resultsIndex] = 0;
   }
   resultsIndex = 0;
   bufferFull = 0;

//
// enable PIE interrupt
//
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

//
// sync ePWM
//
   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
   PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4

   //test interrupt
   //
   // GPIO30 & GPIO31 are outputs, start GPIO30 high and GPIO31 low
   //
      EALLOW;
      GpioDataRegs.GPCSET.bit.GPIO66 = 1;         // Load the output latch
      GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // GPIO
      GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;         // output
      EDIS;
      GpioDataRegs.GPCSET.bit.GPIO66 = 1;      // GPIO66 is high

   //
   // GPIO0 and GPIO1 are inputs
   //
      EALLOW;
      GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;         // GPIO
      GpioCtrlRegs.GPCDIR.bit.GPIO64 = 0;          // input
      GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 0;        // XINT1 Synch to SYSCLKOUT only

      GpioCtrlRegs.GPCCTRL.bit.QUALPRD0 = 0xFF;   // Each sampling window
                                                  // is 510*SYSCLKOUT
      EDIS;
      //test interrupt

      GPIO_SetupXINT1Gpio(64);
      XintRegs.XINT1CR.bit.POLARITY = 3;          // Rasing edge interrupt
      XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1

//
// Enable global Interrupts and higher priority real-time debug events:
//
   EINT;  // Enable Global interrupt INTM
   ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 4. User specific code:
//

//   scia_fifo_init();       // Initialize the SCI FIFO
//   scia_echoback_init();   // Initialize SCI for echoback

   scib_fifo_init();  // for communication of ESP
   scib_echoback_init();

   scic_fifo_init();  // for debug
   scic_echoback_init();

   msg = "\r\n\n\nHello World!\0"; // \0 for denote the end of string
   //scib_msg(msg);
   scic_msg(msg);
   //These code for ESP8266	
   // Connect to the Router
   msg = "AT+CWJAP=\"Robotics Lab 518a\",\"ccny10031\"\r\n\0"; // if you change a router, then change the name and password
   //scic_msg(msg);
   scib_msg(msg);
   DELAY_US(5000000); //等待微妙 wait for some microseconds

   msg = "AT+CIPMUX=1\r\n\0";
   //scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   msg = "AT+CIPSERVER=1,8888\r\n\0";
   //scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   msg = "AT+CIFSR\r\n\0";
   //scic_msg(msg);
   scib_msg(msg);

//   msg = "a\r\n";

   //
   //start ePWM
   //
   EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
   EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode


   for(;;)
   {
       //
       // Wait for inc character
       //
	   //与电脑的连接，Debug专用  for debug receive from computer(com monitor)
       //while(ScibRegs.SCIFFRX.bit.RXFFST != 0)
       while(ScicRegs.SCIFFRX.bit.RXFFST != 0)
       {
           //ReceivedCharb = ScibRegs.SCIRXBUF.bit.SAR;
    	   ReceivedCharb = ScicRegs.SCIRXBUF.bit.SAR;
           //scic_xmit(ReceivedCharb);
           scib_xmit(ReceivedCharb);
           //epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * ((float)ReceivedCharb/10);
    	   //epwm7_info.EPwmMinCMPA = EPWM3_MIN_CMPA*(1-ReceivedCharc/10);
    	   //scib_xmit(epwm7_info.EPwmCMPA);
       }
       //scib_msg(msg);
       //处理网络通信  processing network data
       //while(ScicRegs.SCIFFRX.bit.RXFFST != 0)
       while(ScibRegs.SCIFFRX.bit.RXFFST != 0)
       {
    	   //ReceivedCharc = ScicRegs.SCIRXBUF.bit.SAR;
    	   ReceivedCharc = ScibRegs.SCIRXBUF.bit.SAR;
           //scib_xmit(ReceivedCharc); // for debug, display on computer(com monitor)
           scic_xmit(ReceivedCharc); // for debug, display on computer(com monitor)
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
				   //msg_rec[msg_i] = 0x00; // 空字符\0 null character
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
    	   //scib_msg(msg_rec);//for Debug
    	   scic_msg(msg_rec);//for Debug
    	   msg_state = 0;
    	   state = msg_rec[1];
    	   operator = msg_rec[2];
       }

       state_machine_processing(state, operator);

       //
       //start ePWM
       //
       //EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
       //EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode


       //
       //wait while ePWM causes ADC conversions, which then cause interrupts,
       //which fill the results buffer, eventually setting the bufferFull
       //flag
       //
//       while(!bufferFull);
//       bufferFull = 0; //clear the buffer full flag

       //
       //stop ePWM
       //
       //EPwm1Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
       //EPwm1Regs.TBCTL.bit.CTRMODE = 3; //freeze counter

       //
       //at this point, AdcaResults[] contains a sequence of conversions
       //from the selected channel
       //

//       PressureValue = filteredData(AdcaResults[0]);

       /*
       if(sendstate != 0)
       {

//    	   int send,i;
//    	   int temp,temp1 = 10000;
//    	   temp = PressureValue*100;
//    	   temp = temp % 100000;
//    	   for(i=4; i>=0; i--)
//    	   {
//    		   send = temp/temp1 + 48;
//    		   scic_xmit(send);
//    		   temp = temp % (temp1);
//    		   temp1 = temp1/10;
//    	   }

    	   char msgtest[2];
    	   msgtest[0]=0xEE;
    	   msgtest[1]=0xE1;
    	   scic_msg(msgtest);
       }
       */

       //prepare send heart message
//       if(sendstate == 10000)
//       {
//    	   if(state != 10)
//    	   {
//    		   //msg = "AT+CIPSEND=0,5\r\n\0";
//        	   char msgtest[2];
//    		   msg = "AT+CIPSEND=0,2\r\n\0";
//    		   scic_msg(msg);
//        	   msgtest[0]=0xEE;
//        	   msgtest[1]=0xE1;
//    		  while(ScibRegs.SCIFFRX.bit.RXFFST != 0)
//    		   {
//    			   ReceivedCharb = ScibRegs.SCIRXBUF.bit.SAR;
//    			   scic_xmit(ReceivedCharb);
//    		   }
//        	   if(ReceivedCharb == ">")
//        	   {
//        		   scic_msg(msgtest);
//        		   sendstate = 0;
//        	   }
//    		   //sendstate = 1;
//    	   }
//       }
//	   sendstate++;



       //itoa(PressureValue, msgsend, 4);
       //msgsend[4] = "\0";

       //
       //software breakpoint, hit run again to get updated conversions
       //
       //asm("   ESTOP0");
   }

}

//
//  scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
void scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    // SCIA at 9600 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x45.
    // SCIA at 115200 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x1A.
    SciaRegs.SCIHBAUD.all = 0x0001;
    SciaRegs.SCILBAUD.all = 0x0045;

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void scib_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIB at 9600 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x45.
    // SCIB at 115200 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x1A.
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x001A;

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void scic_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.all = 0x0003;
    ScicRegs.SCICTL2.bit.TXINTENA = 1;
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIC at 115200 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x1A.
    //

    ScicRegs.SCIHBAUD.all = 0x0000;
    ScicRegs.SCILBAUD.all = 0x001A;

    ScicRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all =a;
}

//
// scia_msg - Transmit message via SCIA
//
void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
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

void scic_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scic_xmit(msg[i]);
        i++;
    }
}

void scic_xmit(int a)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCITXBUF.all =a;
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}

void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all = 0xE040;
    ScibRegs.SCIFFRX.all = 0x2044;
    ScibRegs.SCIFFCT.all = 0x0;
}

void scic_fifo_init()
{
    ScicRegs.SCIFFTX.all = 0xE040;
    ScicRegs.SCIFFRX.all = 0x2044;
    ScicRegs.SCIFFCT.all = 0x0;
}




//
// Jinglun 20180920
// InitEPwm7Example - Initialize EPWM7 values
// for motion motor move forward
//
void InitEPwm7Example()
{
   //
   // Setup TBCLK
   //
   EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm7Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
   EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm7Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm7Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   //
   // Setup shadow register load on ZERO
   //
   EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm7Regs.CMPA.bit.CMPA = EPWM3_TIMER_TBPRD*0.5;      // Set compare A value
   EPwm7Regs.CMPB.bit.CMPB = EPWM3_TIMER_TBPRD*0.5;      // Set Compare B value

   //
   // Set actions
   //
   EPwm7Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
   EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                   // up count

   EPwm7Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
   EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                   // up count

   //
   // Interrupt where we will change the Compare Values
   //
   EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm7Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm7Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

   //
   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   //
   epwm7_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
   epwm7_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
   //epwm7_info.EPwmTimerIntCount = 0;               // Zero the interrupt
                                                   // counter
   epwm7_info.EPwmRegHandle = &EPwm7Regs;          // Set the pointer to the
                                                   // ePWM module
   //epwm7_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;        // Setup min/max
                                                   // CMPA/CMPB values
   //epwm7_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
   //epwm7_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
   //epwm7_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
   epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5;
   epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5;
   epwm7_info.EPwmSpeed = 0.5;
}

//
// InitEPwm8Example - Initialize EPWM8 values
//
void InitEPwm8Example()
{
	   //
	   // Setup TBCLK
	   //
	   EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	   EPwm8Regs.TBPRD = EPWM8_TIMER_TBPRD;       // Set timer period
	   EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	   EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
	   EPwm8Regs.TBCTR = 0x0000;                  // Clear counter
	   EPwm8Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
	   EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;

	   //
	   // Setup shadow register load on ZERO
	   //
	   EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   //
	   // Set Compare values
	   //
	   EPwm8Regs.CMPA.bit.CMPA = EPWM8_MID_ESC;      // Set compare A value
	   EPwm8Regs.CMPB.bit.CMPB = EPWM8_MID_CMP;      // Set Compare B value

	   //
	   // Set actions
	   //
	   EPwm8Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
	   EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
	                                                   // up count

	   EPwm8Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
	   EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
	                                                   // up count

	   //
	   // Interrupt where we will change the Compare Values
	   //
	   //EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
	   //EPwm8Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
	   //EPwm8Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

	   //
	   // Information this example uses to keep track
	   // of the direction the CMPA/CMPB values are
	   // moving, the min and max allowed values and
	   // a pointer to the correct ePWM registers
	   //
	   epwm8_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
	   epwm8_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
	   //epwm8_info.EPwmTimerIntCount = 0;               // Zero the interrupt
	                                                   // counter
	   epwm8_info.EPwmRegHandle = &EPwm8Regs;          // Set the pointer to the
	                                                   // ePWM module
	   //epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max
	                                                   // CMPA/CMPB values
	   epwm8_info.EPwmCMPA = EPWM8_MID_ESC;
	   epwm8_info.EPwmCMPB = EPWM8_MID_CMP;
	   epwm8_info.EPwmSpeed = 0.5;
}

//
// Jinglun 20180920
// InitEPwm9Example - Initialize EPWM9 values
// for motion motor move backward
//
void InitEPwm9Example()
{
   //
   // Setup TBCLK
   //
   EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm9Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
   EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm9Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm9Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm9Regs.TBCTL.bit.HSPCLKDIV = 4;//TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   //
   // Setup shadow register load on ZERO
   //
   EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm9Regs.CMPA.bit.CMPA = EPWM3_TIMER_TBPRD*0.5;      // Set compare A value
   EPwm9Regs.CMPB.bit.CMPB = EPWM3_TIMER_TBPRD*0.5;      // Set Compare B value

   //
   // Set actions
   //
   EPwm9Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
   EPwm9Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                   // up count

   EPwm9Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
   EPwm9Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B,
                                                   // up count

   //
   // Interrupt where we will change the Compare Values
   //
   EPwm9Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
   EPwm9Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
   EPwm9Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event

   //
   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   //
   epwm9_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
   epwm9_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
   //epwm7_info.EPwmTimerIntCount = 0;               // Zero the interrupt
                                                   // counter
   epwm9_info.EPwmRegHandle = &EPwm9Regs;          // Set the pointer to the

   epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5;
   epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5;
   epwm9_info.EPwmSpeed = 0.5;       
}

//
// epwm7_isr - EPWM7 ISR to update compare values
//
__interrupt void epwm7_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    update_compare(&epwm7_info);

    //
    // Clear INT flag for this timer
    //
    EPwm7Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// Jinglun 20180920
// epwm9_isr - EPWM9 ISR to update compare values
//
__interrupt void epwm9_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    update_compare(&epwm9_info);

    //
    // Clear INT flag for this timer
    //
    EPwm9Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm8_isr - EPWM8 ISR to update compare values
//
/*
__interrupt void epwm8_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
	if(epwm8_info.EPwmCMPB != epwm8_info.EPwmRegHandle->CMPB.bit.CMPB)
	{
		//epwm8_info.EPwmRegHandle->CMPA.bit.CMPA = epwm8_info.EPwmCMPA;
		epwm8_info.EPwmRegHandle->CMPB.bit.CMPB = epwm8_info.EPwmCMPB;
	}

    //
    // Clear INT flag for this timer
    //
    EPwm7Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
*/

//
// update_compare - Update the compare values for the specified EPWM
//
void update_compare(EPWM_INFO *epwm_info)
{

	if(epwm_info->EPwmCMPA != epwm_info->EPwmRegHandle->CMPA.bit.CMPA)
	{
		epwm_info->EPwmRegHandle->CMPA.bit.CMPA = epwm_info->EPwmCMPA;
		epwm_info->EPwmRegHandle->CMPB.bit.CMPB = epwm_info->EPwmCMPB;
	}
   return;
}


//
// state machine processing
//
void state_machine_processing(Unit16 state, Unit16 operator)
{
	switch(state)
	{
	case 0://moving state
		switch(operator & emmergency)
		{
		case 0x00://halt
			epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
			epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0;
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
			epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0;
			break;
		case 0x01://move forward
            // for disable the pwm9
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
			epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm7 and then let it move
			epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5 + EPWM3_TIMER_TBPRD * 0.5 * (1 - epwm7_info.EPwmSpeed);
			epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5 + EPWM3_TIMER_TBPRD * 0.5 * (1 - epwm7_info.EPwmSpeed);
			break;
		case 0x02://move backward
            // for disable the pwm7
			epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
			epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm9 then let it move
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5 * epwm9_info.EPwmSpeed;
			epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5 * epwm9_info.EPwmSpeed;
			break;
		case 0x04://turn left
            // for disable the pwm9
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm7 and then let it move
			epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5 * epwm7_info.EPwmSpeed;
            // for disable the pwm7
            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm9 then let it move
            epwm9_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5 + EPWM3_TIMER_TBPRD * 0.5 * (1 - epwm7_info.EPwmSpeed);
			break;
		case 0x08://turn right
            // for disable the pwm7
            epwm7_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm9 then let it move
			epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0.5 + EPWM3_TIMER_TBPRD * 0.5 * (1 - epwm7_info.EPwmSpeed);
            // for disable the pwm9
            epwm9_info.EPwmCMPA = EPWM3_TIMER_TBPRD * 0;
            // for enable the pwm7 and then let it move
			epwm7_info.EPwmCMPB = EPWM3_TIMER_TBPRD * 0.5 * epwm7_info.EPwmSpeed;
			break;
		}
		break;
	case 1://suction motor
		switch(operator)
		{
		case 0x00://stop
			epwm8_info.EPwmRegHandle->CMPA.bit.CMPA = EPWM8_MID_ESC;
			//epwm8_info.EPwmCMPA = EPWM8_MID_ESC;
		    pre_state = 1;
		    pre_operator = 0x00;
			break;
		case 0x01://start
			epwm8_info.EPwmRegHandle->CMPA.bit.CMPA = epwm8_info.EPwmCMPA;
			//epwm8_info.EPwmCMPA = 62500 * (0.95-0.05 * epwm8_info.EPwmSpeed);
		    pre_state = 1;
		    pre_operator = 0x01;
			break;
		}
		break;
	case 4://servo
		switch(operator % 3)
		{
		case 0:
			epwm8_info.EPwmRegHandle->CMPB.bit.CMPB = EPWM8_MID_CMP;
			state = 0;
			operator = 0;
			break;
		case 1:
			epwm8_info.EPwmRegHandle->CMPB.bit.CMPB = EPWM8_LOW_CMP;
			state = 0;
			operator = 0;
			break;
		case 2:
			epwm8_info.EPwmRegHandle->CMPB.bit.CMPB = EPWM8_HIG_CMP;
			state = 0;
			operator = 0;
			break;
		}
		break;
	case 5://setting speed
		switch(operator)
		{
		case 1://wheel speed
			epwm7_info.EPwmSpeed = msg_rec[3];
			epwm7_info.EPwmSpeed = 1 - epwm7_info.EPwmSpeed/100;
			state = 0;
			operator = 0;
			break;
		case 2://suction speed
			epwm8_info.EPwmSpeed = msg_rec[3];
			epwm8_info.EPwmSpeed = epwm8_info.EPwmSpeed/100;
			epwm8_info.EPwmCMPA = 62500 * (0.95-0.05 * epwm8_info.EPwmSpeed);
			//epwm8_info.EPwmRegHandle->CMPA.bit.CMPA = 62500 * (0.95-0.05 * epwm8_info.EPwmSpeed);
			state = pre_state;
			operator = pre_operator;
			break;
		}
		break;
	/*
	case 6://LED
		switch(operator)
		{
		}
		break;
	*/
	}
	return;
}

//
// xint1_isr - External Interrupt 1 ISR
//
interrupt void xint1_isr(void)
{
	if(GpioDataRegs.GPCDAT.bit.GPIO64 == 1)
	{
		GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // lower GPIO31
		emmergency = 0x0E;
	}
	else
	{
		GpioDataRegs.GPCSET.bit.GPIO66 = 1;
		emmergency = 0x0F;
	}

    //
	// Acknowledge this interrupt to get more from group 1
    //
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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

//
// End of file
//
