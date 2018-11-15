# ClimbRobot
Climbing Robot DPS code
We use the chip ESP8266 to connect the DSP with WIFI [ESP8266](https://en.wikipedia.org/wiki/ESP8266)    
[Starting for ESP8266](https://circuitdigest.com/microcontroller-projects/how-to-use-at-commands-with-esp8266-module)      
When you have a new ESP8266, you should configure it at first with **AT commands**.  [AT commands set](https://www.espressif.com/sites/default/files/documentation/4a-esp8266_at_instruction_set_en.pdf)     
Firstly, you need a serial port debugging tool to configure, like this [one](https://github.com/Neutree/COMTool/releases).   
Connecting the ESP with computer via an USB-232 port, and then you can operate the chip by AT commands. Such as set **IP Address, check the connection**.


**code for ESP8266**	
```
   //SCIC for debug ESP chip by computer
   // Connect to the Router
   msg = "AT+CWJAP=\"Robotics Lab 518a\",\"ccny10031\"\r\n\0"; // if you change a router, then change the name and password
   scic_msg(msg); // connect scic with computer, you can see the feedback from ESP on computer
   scib_msg(msg);
   DELAY_US(5000000); //等待微妙 wait for some microseconds

   //Set by TCP multiple connections
   msg = "AT+CIPMUX=1\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   //Set ESP as server, port=8888
   msg = "AT+CIPSERVER=1,8888\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
   DELAY_US(100000); //等待微妙 wait for some microseconds

   // Get IP Address, if you connect with computer and open scic, you will see the IP address on computer
   msg = "AT+CIFSR\r\n\0";
   scic_msg(msg);
   scib_msg(msg);
```

# For ESP8266
## Pin Layout

ESP8266 | DSP
---|---
TX | RX
GND | GND
CH_PD | 3.3V
VCC | 3.3V
RX | TX

## ESP8266 configuration 

Firstly, we can the USB module to test ESP8266 and configure it.       

![image](https://ss1.bdstatic.com/70cFuXSh_Q1YnxGkpoWK1HF6hhy/it/u=3711389681,1822270054&fm=27&gp=0.jpg)      

The default baud rate of ESP is 115200.

# Commutation Protocol
	FORWARD       = 0xFF, 0x00, 0x01, 0x00, 0xFE
	BACKWARD      = 0xFF, 0x00, 0x02, 0x00, 0xFE
	STOP          = 0xFF, 0x00, 0x00, 0x00, 0xFE
	LEFT          = 0xFF, 0x00, 0x04, 0x00, 0xFE
	RIGHT         = 0xFF, 0x00, 0x08, 0x00, 0xFE
	SUCTION_ON    = 0xFF, 0x01, 0x01, 0x00, 0xFE
	SUCTION_OFF   = 0xFF, 0x01, 0x00, 0x00, 0xFE
	SERVO         = 0xFF, 0x04, 0x00, 0x01, 0xFE
	motion_speed  = 0xFF, 0x05, 0x01, 0x32, 0xFE
	suction_speed = 0xFF, 0x05, 0x02, 0x32, 0xFE

# Quick Start
1.Install [CCS](http://www.ti.com/tool/CCSTUDIO) and [controlSUITE](http://www.ti.com/tool/CONTROLSUITE)
2.Downdload code form [ClimbRobot](https://github.com/liuzhaoming5954/ClimbRobot)
3.Import the project to CCS
4.Configure the project. Right click on the name of project and choose "Properties", configure “Include Options”. Find out "F2837xS_common\include" and "F2837xS_headers\include" in your installation directory of controlSUITE.
![Include Options](http://www.21ic.com/d/file/201601/c7eb07e699c54bb394a7fc358b5e75f2.png)

Set file search path "F2837xS_common\cmd" and "F2837xS_headers\cmd" to installation directory of controlSUITE.
![file search path](http://www.21ic.com/d/file/201601/e7b0d28a352bc603ea7cfc8fcb68d932.png)

5.Build and download, test the code

# JTAG emulator
The JTAG emulator is [XDS100v2](https://www.digikey.com/products/en?mpart=TMDSEMU100V2U-14T&v=296)
![XDS100v2](https://media.digikey.com/Photos/Texas%20Instr%20Photos/TMDSEMU100V2U-14T_sml.jpg)

# State Machine
state 0: moving state     

sub-state | operation
---|---
0 | halt
1 | move forward
2 | move backward
4 | turn left
8 | turn right
 	   

state 1: suction motor operation  

sub-state | operation
---|---
sub |state:
0 | stop
1 | start

state 4 : servo operation      

sub-state | operation
---|---
0 | middle
1 | low
2 | high

state 5 : setting speed        

sub-state | operation
---|---
1 | set wheel speed
2 | set suction motor speed
