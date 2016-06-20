/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *   http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial contribution
 *******************************************************************************/

#define MQTTCLIENT_QOS2 1

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetStack.h>
#include <Countdown.h>
#include <MQTTClient.h>
#include "Oled_C.h"
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/ssi.h>
//#include <ti/drivers/SPI.h>
#include "letters_mapping.h"
#include <time.h>

char printbuf[100];

int arrivedcount = 0;

int input = 0;

void messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;
  
  /*sprintf(printbuf, "Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n", 
		++arrivedcount, message.qos, message.retained, message.dup, message.id);
  Serial.print(printbuf);*/
  sprintf(printbuf, "Payload %s\n", (char*)message.payload);
  Serial.print(printbuf);
  input = atoi((char*)message.payload);
}


EthernetStack ipstack;
MQTT::Client<EthernetStack, Countdown> client = MQTT::Client<EthernetStack, Countdown>(ipstack);

byte mac[] = { 0x00, 0x1A, 0xB6, 0x02, 0xAA, 0x42 };  // replace with your device's MAC
const char* topic = "quiz";

void connect()
{
//  char hostname[] = "169.254.212.206";
//  char hostname[] = "169.254.245.200";
  char hostname[] = "169.254.212.206";
  int port = 1883;
  sprintf(printbuf, "Connecting to %s:%d\n", hostname, port);
  Serial.print(printbuf);
  int rc = ipstack.connect(hostname, port);
  if (rc != 1)
  {
    sprintf(printbuf, "rc from TCP connect is %d\n", rc);
    Serial.print(printbuf);
  }
 
  Serial.println("MQTT connecting");
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;       
  data.MQTTVersion = 3;
  data.clientID.cstring = (char*)"energia-sample";
  rc = client.connect(data);
  if (rc != 0)
  {
    sprintf(printbuf, "rc from MQTT connect is %d\n", rc);
    Serial.print(printbuf);
  }
  Serial.println("MQTT connected");
  
  /*rc = client.subscribe(topic, MQTT::QOS2, messageArrived);   
  if (rc != 0)
  {
    sprintf(printbuf, "rc from MQTT subscribe is %d\n", rc);
    Serial.print(printbuf);
  }
  Serial.println("MQTT subscribed");*/
}


void writeLine(int line,int X1,int X2,int Y1,int Y2,char forground1,char forground2){
	OLED_C_MemorySize(X1,X2,Y1,Y2);
    DDRAM_access();
    writeLetterLine(line,forground1,forground2);
}


void writeLetterLine(int line,char forground1,char forground2){
	if(line&128){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&64){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&32){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&16){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&8){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&4){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&2){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
	if(line&1){
    	OLED_C_Color(forground1,forground2);
	}else{
		OLED_C_Color(0xFF,0xFF);
	}
}

void writeLetter(int line1,int line2,int line3,int line4,int line5,int line6,int line7,int line8,int X1,int X2,int Y1,int Y2,char forground1,char forground2){
	unsigned j;
	OLED_C_MemorySize(X1,X2,Y1,Y2);
    DDRAM_access();
    //8x8
    for(j=0;j<64;j+=8){
  	  if(j<8){
  		  writeLetterLine(line1,forground1,forground2);
  	  }else if(j<16){
  		  writeLetterLine(line2,forground1,forground2);
  	  }else if(j<24){
  		  writeLetterLine(line3,forground1,forground2);
  	  }else if(j<32){
  		  writeLetterLine(line4,forground1,forground2);
  	  }else if(j<40){
  		  writeLetterLine(line5,forground1,forground2);
  	  }else if(j<48){
  		  writeLetterLine(line6,forground1,forground2);
  	  }else if(j<56){
  		  writeLetterLine(line7,forground1,forground2);
  	  }else if(j<64){
  		  writeLetterLine(line8,forground1,forground2);
  	  }
    }
}

void SPI2_Write(unsigned char input){
	uint32_t tmp=0;
	SSIDataPut(SSI2_BASE,input);
	while (SSIBusy(SSI2_BASE));
	while (SSIDataGetNonBlocking(SSI2_BASE,&tmp));
}

void delay_ms(double ms){
	   double sec = ms/1000;
	   SysCtlDelay(F_CPU/(3/sec));
	   sec=2;
}

// Init MCU function
void InitBoard(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);   // Set GPIOC pin 7  as digital output  (OLED_RST pin)

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_2);   // Set GPIOH pin 2 as digital output  (OLED_CS pin)

	GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, 4);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_3);   // Set GPIOM pin 3  as digital output  (OLED_DC pin)

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);   // Set GPIOE pin 4  as digital output  (OLED_T pin)(R/W)

// Initialize SPI2 module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

//	SPI_init();
        SPI.begin();

	GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);

	GPIOPinConfigure(GPIO_PD3_SSI2CLK);

	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

	SSIClockSourceSet(SSI2_BASE,SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(SSI2_BASE,SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480
			, 120000000), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER,1000000,8);



	SSIEnable(SSI2_BASE);

	GPIOPinWrite (GPIO_PORTE_BASE, GPIO_PIN_4, 0);//OLED_T=0;
}

//Send command to OLED C display
void OLED_C_command(unsigned char reg_index, unsigned char reg_value){
//Select index addr
	GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 0);//OLED_CS = 0;
	GPIOPinWrite (GPIO_PORTM_BASE, GPIO_PIN_3, 0);//OLED_DC = 0;
    SPI2_Write(reg_index);
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 4);//OLED_CS = 1;
//Write data to reg
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 0);//OLED_CS = 0;
    GPIOPinWrite (GPIO_PORTM_BASE, GPIO_PIN_3, 8);//OLED_DC = 1;
    SPI2_Write(reg_value);
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 4);//OLED_CS = 1;
}

//Send data to OLED C display
void OLED_C_data(unsigned char data_value){
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 0);//OLED_CS = 0;
    GPIOPinWrite (GPIO_PORTM_BASE, GPIO_PIN_3, 8);//OLED_DC = 1;
    SPI2_Write(data_value);
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 4);//OLED_CS = 1;
}

// Init sequence for 96x96 OLED color module
void OLED_C_Init(){
				   GPIOPinWrite (GPIO_PORTC_BASE, GPIO_PIN_7, 0);//OLED_RST=0;
                   delay_ms(10);
				   GPIOPinWrite (GPIO_PORTC_BASE, GPIO_PIN_7, 128);//OLED_RST=1;
                   delay_ms(10);
                   /*  Soft reser */
                   OLED_C_command(SEPS114A_SOFT_RESET,0x00);
                   /* Standby ON/OFF*/
                   OLED_C_command(SEPS114A_STANDBY_ON_OFF,0x01);          // Standby on
                   delay_ms(5);                                           // Wait for 5ms (1ms Delay Minimum)
                   OLED_C_command(SEPS114A_STANDBY_ON_OFF,0x00);          // Standby off
                   delay_ms(5);                                           // 1ms Delay Minimum (1ms Delay Minimum)
                   /* Display OFF */
                   OLED_C_command(SEPS114A_DISPLAY_ON_OFF,0x00);
                   /* Set Oscillator operation */
                   //OLED_C_command(SEPS114A_ANALOG_CONTROL,0x00);          // using external resistor and internal OSC
                   OLED_C_command(SEPS114A_ANALOG_CONTROL,0x40);
                   /* Set frame rate */
                   OLED_C_command(SEPS114A_OSC_ADJUST,0x03);              // frame rate : 95Hz
                   /* Set active display area of panel */
                   OLED_C_command(SEPS114A_DISPLAY_X1,0x00);
                   OLED_C_command(SEPS114A_DISPLAY_X2,0x5F);
                   OLED_C_command(SEPS114A_DISPLAY_Y1,0x00);
                   OLED_C_command(SEPS114A_DISPLAY_Y2,0x5F);
                   /* Select the RGB data format and set the initial state of RGB interface port */
                   OLED_C_command(SEPS114A_RGB_IF,0x00);                 // RGB 8bit interface
                   /* Set RGB polarity */
                   OLED_C_command(SEPS114A_RGB_POL,0x00);
                   /* Set display mode control */
                   OLED_C_command(SEPS114A_DISPLAY_MODE_CONTROL,0x80);   // SWAP:BGR, Reduce current : Normal, DC[1:0] : Normal
                   /* Set MCU Interface */
                   OLED_C_command(SEPS114A_CPU_IF,0x00);                 // MPU External interface mode, 8bits
                   /* Set Memory Read/Write mode */
                   OLED_C_command(SEPS114A_MEMORY_WRITE_READ,0x00);
                   /* Set row scan direction */
                   OLED_C_command(SEPS114A_ROW_SCAN_DIRECTION,0x00);     // Column : 0 --> Max, Row : 0 --> Max
                   /* Set row scan mode */
                   OLED_C_command(SEPS114A_ROW_SCAN_MODE,0x00);          // Alternate scan mode
                   /* Set column current */
                   OLED_C_command(SEPS114A_COLUMN_CURRENT_R,0x6E);
                   OLED_C_command(SEPS114A_COLUMN_CURRENT_G,0x4F);
                   OLED_C_command(SEPS114A_COLUMN_CURRENT_B,0x77);
                   /* Set row overlap */
                   OLED_C_command(SEPS114A_ROW_OVERLAP,0x00);            // Band gap only
                   /* Set discharge time */
                   OLED_C_command(SEPS114A_DISCHARGE_TIME,0x01);         // Discharge time : normal discharge
                   /* Set peak pulse delay */
                   OLED_C_command(SEPS114A_PEAK_PULSE_DELAY,0x00);
                   /* Set peak pulse width */
                   OLED_C_command(SEPS114A_PEAK_PULSE_WIDTH_R,0x02);
                   OLED_C_command(SEPS114A_PEAK_PULSE_WIDTH_G,0x02);
                   OLED_C_command(SEPS114A_PEAK_PULSE_WIDTH_B,0x02);
                   /* Set precharge current */
                   OLED_C_command(SEPS114A_PRECHARGE_CURRENT_R,0x14);
                   OLED_C_command(SEPS114A_PRECHARGE_CURRENT_G,0x50);
                   OLED_C_command(SEPS114A_PRECHARGE_CURRENT_B,0x19);
                   /* Set row scan on/off  */
                   OLED_C_command(SEPS114A_ROW_SCAN_ON_OFF,0x00);        // Normal row scan
                   /* Set scan off level */
                   OLED_C_command(SEPS114A_SCAN_OFF_LEVEL,0x04);         // VCC_C*0.75
                   /* Set memory access point */
                   OLED_C_command(SEPS114A_DISPLAYSTART_X,0x00);
                   OLED_C_command(SEPS114A_DISPLAYSTART_Y,0x00);
                   /* Display ON */
                   OLED_C_command(SEPS114A_DISPLAY_ON_OFF,0x01);

                   uint32_t input = 0;
                   //SSIDataGet(SSI2_BASE,input);
}

//Sekvence before writing data to memory
void DDRAM_access(){
	GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 0);//OLED_CS = 0;
	GPIOPinWrite (GPIO_PORTM_BASE, GPIO_PIN_3, 0);//OLED_DC = 0;
    SPI2_Write(0x08);
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 4);//OLED_CS = 1;
    //delay_ms(0.01);
}

//Set memory area(address) to write a display data
void OLED_C_MemorySize(char X1, char X2, char Y1, char Y2){
    OLED_C_command(SEPS114A_MEM_X1,X1);
    OLED_C_command(SEPS114A_MEM_X2,X2);
    OLED_C_command(SEPS114A_MEM_Y1,Y1);
    OLED_C_command(SEPS114A_MEM_Y2,Y2);
}

//Select color
void OLED_C_Color(char colorMSB, char colorLSB ){
    OLED_C_data(colorMSB);
    OLED_C_data(colorLSB);
}


void snakeFxn(){
	unsigned i;
	int oldInput = 0;
	/*time_t t;
	time(&t);
	srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
	int value;
	/*
	 * 0 Neutral
	 * 1 UP
	 * 2 Down
	 * 3 Left
	 * 4 Right
	 * 5 Drücken
	 */

	OLED_C_MemorySize(0x2C,0x33,0x2C,0x34);
    DDRAM_access();
    while(1){
    	value=rand() % 5;
    	switch(value+1){
			case 1://O
				writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,0x2C,0x33,0x2C,0x33,0x00,0x00);
				break;
			case 2://U
				writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x2C,0x33,0x2C,0x33,0x00,0x00);
				break;
			case 3://L
				writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,0x2C,0x33,0x2C,0x33,0x00,0x00);
				break;
			case 4://R
				writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,0x2C,0x33,0x2C,0x33,0x00,0x00);
				break;
			case 5://D
				writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,0x2C,0x33,0x2C,0x33,0x00,0x00);
				break;
    	}
        delay(5000);
    	//Task_sleep(3000);
    	/*Mailbox_pend(mbox,&input,10);
    	if(input == oldInput){
    		Mailbox_pend(mbox,&input,10);
    	}*/
        client.subscribe("quiz",MQTT::QOS0,messageArrived);
        Serial.println("Snake: ");
        Serial.println(input,DEC);
        Serial.println(value,DEC);
    	if(input == value+1){
    		int Y1 = 0x2C;
    		int Y2 = 0x33;
    		for(i=0; i < 14 ; i++){
    			writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,0x2C,0x33,Y1,Y2,0x0C,0xC0);

        		Y1++;
				Y2++;

				OLED_C_MemorySize(0x2C,0x33,Y1-1,Y1-1);
			    DDRAM_access();
			    writeLetterLine(255,0xFF,0xFF);

				OLED_C_MemorySize(0x2C,0x33,0x37,0x37);
			    DDRAM_access();
			    writeLetterLine(255,0x00,0x00);

			    delay_ms(90);
    		}
    	}else{
    		int Y1 = 0x2C;
    		int Y2 = 0x33;
    		for(i=0; i < 14 ; i++){
        		writeLetter(fLine1,fLine2,fLine3,fLine4,fLine5,fLine6,fLine7,fLine8,0x2C,0x33,Y1,Y2,0xF8,0x00);

        		Y1++;
				Y2++;

				OLED_C_MemorySize(0x2C,0x33,Y1-1,Y1-1);
			    DDRAM_access();
			    writeLetterLine(255,0xFF,0xFF);

				OLED_C_MemorySize(0x2C,0x33,0x37,0x37);
			    DDRAM_access();
			    writeLetterLine(255,0x00,0x00);

			    delay_ms(90);
    		}
    	}
    	oldInput=input;
    	input=0;
    }

}


void setup()
{
  Serial.begin(9600);

  Serial.println("Starting Ethernet1");
  Ethernet.enableLinkLed();
  Ethernet.enableActivityLed();
  Ethernet.begin(0);

  Serial.println("\nIP Address obtained");
  // We are connected and have an IP address.
  Serial.println(Ethernet.localIP());
  
  Serial.println("OLED_C");
  connect();

  InitBoard();
  //UART1_Init(56000);
  OLED_C_Init();
  
  Serial.println("MQTT Hello example");

  unsigned j;
  OLED_C_command(0x1D,0x02);                //Set Memory Read/Write mode

  OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
  DDRAM_access();
  for(j=0;j<9216;j++){
                      OLED_C_Color(0xFF,0xFF);
                     }
  char buf[100];
  sprintf(buf, "Snake START MUHAHAHA");
  Serial.println(buf);
  snakeFxn();
}

void loop()
{
  /*if (!client.isConnected())
    connect();
  
  MQTT::Message message;
  
  arrivedcount = 0;

  // Send and receive QoS 0 message
  char buf[100];
  /*sprintf(buf, "Hello World! QoS 0 message");
  Serial.println(buf);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*)buf;
  message.payloadlen = strlen(buf)+1;
  int rc = client.publish(topic, message);
  while (arrivedcount == 0)
    client.yield(1000);
        
  // Send and receive QoS 1 message
  sprintf(buf, "Hello World!  QoS 1 message");
  Serial.println(buf);
  message.qos = MQTT::QOS1;
  message.payloadlen = strlen(buf)+1;
  rc = client.publish(topic, message);
  while (arrivedcount == 1)
    client.yield(1000);
        
  // Send and receive QoS 2 message
  /*sprintf(buf, "Hello World!  QoS 2 message");
  Serial.println(buf);
  message.qos = MQTT::QOS2;
  message.payloadlen = strlen(buf)+1;
  rc = client.publish(topic, message);
  while (arrivedcount == 2)
    client.yield(1000);*/
  
  /*sprintf(buf, "Snake START MUHAHAHA");
  Serial.println(buf);
  snakeFxn();
  /*
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*)buf;
  message.payloadlen = strlen(buf)+1;
      if(client.subscribe("quiz",MQTT::QOS0,messageArrived)) {
        Serial.println("Subscription successfull");
      }
    
  delay(2000);*/
}
