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
