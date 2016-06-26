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
  char hostname[] = "169.254.78.142";
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


void writeNumbers(int number,int X1,int X2,int Y1,int Y2,char forground1,char forground2){
	switch(number){
	case 0:
		writeLetter(z0Line1,z0Line2,z0Line3,z0Line4,z0Line5,z0Line6,z0Line7,z0Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 1:
		writeLetter(z1Line1,z1Line2,z1Line3,z1Line4,z1Line5,z1Line6,z1Line7,z1Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 2:
		writeLetter(z2Line1,z2Line2,z2Line3,z2Line4,z2Line5,z2Line6,z2Line7,z2Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 3:
		writeLetter(z3Line1,z3Line2,z3Line3,z3Line4,z3Line5,z3Line6,z3Line7,z3Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 4:
		writeLetter(z4Line1,z4Line2,z4Line3,z4Line4,z4Line5,z4Line6,z4Line7,z4Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 5:
		writeLetter(z5Line1,z5Line2,z5Line3,z5Line4,z5Line5,z5Line6,z5Line7,z5Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 6:
		writeLetter(z6Line1,z6Line2,z6Line3,z6Line4,z6Line5,z6Line6,z6Line7,z6Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 7:
		writeLetter(z7Line1,z7Line2,z7Line3,z7Line4,z7Line5,z7Line6,z7Line7,z7Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 8:
		writeLetter(z8Line1,z8Line2,z8Line3,z8Line4,z8Line5,z8Line6,z8Line7,z8Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	case 9:
		writeLetter(z9Line1,z9Line2,z9Line3,z9Line4,z9Line5,z9Line6,z9Line7,z9Line8,X1,X2,Y1,Y2,forground1,forground2);
		break;
	}
}



void writeNumber(int value,int x1,int x2,int y1,int y2,int c1,int c2){
	unsigned j;
	int tausenderStelle = value/1000;
	int hunderterStelle = (value-(tausenderStelle*1000))/100;
	int zehnerStelle = (value-(value/100)*100)/10;
	int einerStelle = (value-(value/10)*10);
	if(tausenderStelle > 0){
		writeNumbers(tausenderStelle,x1,x2,y1,y2,c1,c2);
		writeNumbers(hunderterStelle,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(zehnerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(einerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
	}else if(hunderterStelle > 0){
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1,x2,y1,y2,c1,c2);
		writeNumbers(hunderterStelle,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(zehnerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(einerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
	}else if(zehnerStelle > 0){
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1,x2,y1,y2,c1,c2);
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(zehnerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(einerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
	}else{
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1,x2,y1,y2,c1,c2);
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1+=9,x2+=9,y1,y2,c1,c2);
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,x1+=9,x2+=9,y1,y2,c1,c2);
		writeNumbers(einerStelle,x1+=9,x2+=9,y1,y2,c1,c2);
	}
}

void printStatistik(int uVal, int iVal, int lVal){
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x09,0x10,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x12,0x19,0x51,0x58,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,0x1B,0x22,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x24,0x2B,0x51,0x58,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x2D,0x34,0x51,0x58,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x36,0x3D,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x3F,0x46,0x51,0x58,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x48,0x4F,0x51,0x58,0x00,0x00);
	writeLetter(kLine1,kLine2,kLine3,kLine4,kLine5,kLine6,kLine7,kLine8,0x51,0x58,0x51,0x58,0x00,0x00);


	int x1=0x07;
	int x2=0x0E;
	int y1=0x39;
	int y2=0x40;
	int c1=0x00;
	int c2=0x00;
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,x1,x2,y1,y2,c1,c2);
	writeNumber(uVal,x1+=9*2,x2+=9*2,y1,y2,c1,c2);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=9*6),(x2+=9*6),y1,y2,c1,c2);
	writeLetter(vLine1,vLine2,vLine3,vLine4,vLine5,vLine6,vLine7,vLine8,(x1+=9),(x2+=9),y1,y2,c1,c2);

	x1=0x07;
	x2=0x0E;
	y1=0x2D;
	y2=0x34;
	c1=0x00;
	c2=0x00;
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,x1,x2,y1,y2,c1,c2);
	writeNumber(iVal,x1+=9*2,x2+=9*2,y1,y2,c1,c2);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=9*6),(x2+=9*6),y1,y2,c1,c2);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,(x1+=9),(x2+=9),y1,y2,c1,c2);

	x1=0x07;
	x2=0x0E;
	y1=0x21;
	y2=0x28;
	c1=0x00;
	c2=0x00;
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,x1,x2,y1,y2,c1,c2);
	writeNumber(lVal,x1+=9*2,x2+=9*2,y1,y2,c1,c2);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9*5),(x2+=9*5),y1,y2,c1,c2);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,(x1+=9),(x2+=9),y1,y2,c1,c2);
	writeLetter(xLine1,xLine2,xLine3,xLine4,xLine5,xLine6,xLine7,xLine8,(x1+=9),(x2+=9),y1,y2,c1,c2);
	//U xxxx mV
	//I xxxx mA
	//L xxxx LUX
	//print Punkte
	//ask for values
}

void modusNavigation(int modusMenuePoint){
	unsigned j;
	switch(modusMenuePoint){
		case 0:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x12,0x16);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 1:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x12,0x16);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 2:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x12,0x16);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 3:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
				OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x12,0x16);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 4:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x12,0x16);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  OLED_C_Color(0x00,0x00);
			}
			break;
	}
}

void printModus(int choosenMode){
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,0x1B,0x22,0x51,0x58,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,0x24,0x2B,0x51,0x58,0x00,0x00);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,0x2D,0x34,0x51,0x58,0x00,0x00);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x36,0x3D,0x51,0x58,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x3F,0x46,0x51,0x58,0x00,0x00);

	/*
	writeLetter(bLine1,bLine2,bLine3,bLine4,bLine5,bLine6,bLine7,bLine8,0x07,0x0E,0x43,0x4A,0x00,0x00);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x10,0x17,0x43,0x4A,0x00,0x00);
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,0x19,0x20,0x43,0x4A,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x22,0x29,0x43,0x4A,0x00,0x00);*/

	//standby, moodlamp, colorswirl, lightflow, no Modus
	int x1=0x07;
	int x2=0x0E;
	int c1 = ((choosenMode == 1) ? 0x0C : 0x00);
	int c2 = ((choosenMode == 1) ? 0x0C : 0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,x1,x2,0x43,0x4A,c1,c2);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);
	writeLetter(bLine1,bLine2,bLine3,bLine4,bLine5,bLine6,bLine7,bLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);
	writeLetter(yLine1,yLine2,yLine3,yLine4,yLine5,yLine6,yLine7,yLine8,(x1+=9),(x2+=9),0x43,0x4A,c1,c2);


	x1=0x07;
	x2=0x0E;
	c1 = ((choosenMode == 2) ? 0x0C : 0x00);
	c2 = ((choosenMode == 2) ? 0x0C : 0x00);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,x1,x2,0x36,0x3d,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);
	writeLetter(pLine1,pLine2,pLine3,pLine4,pLine5,pLine6,pLine7,pLine8,(x1+=9),(x2+=9),0x36,0x3d,c1,c2);


	x1=0x07;
	x2=0x0E;
	c1 = ((choosenMode == 3) ? 0x0C : 0x00);
	c2 = ((choosenMode == 3) ? 0x0C : 0x00);
	writeLetter(cLine1,cLine2,cLine3,cLine4,cLine5,cLine6,cLine7,cLine8,x1,x2,0x29,0x30,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(wLine1,wLine2,wLine3,wLine4,wLine5,wLine6,wLine7,wLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x29,0x30,c1,c2);


	x1=0x07;
	x2=0x0E;
	c1 = ((choosenMode == 4) ? 0x0C : 0x00);
	c2 = ((choosenMode == 4) ? 0x0C : 0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,x1,x2,0x1D,0x24,c1,c2);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(gLine1,gLine2,gLine3,gLine4,gLine5,gLine6,gLine7,gLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(hLine1,hLine2,hLine3,hLine4,hLine5,hLine6,hLine7,hLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(fLine1,fLine2,fLine3,fLine4,fLine5,fLine6,fLine7,fLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);
	writeLetter(wLine1,wLine2,wLine3,wLine4,wLine5,wLine6,wLine7,wLine8,(x1+=9),(x2+=9),0x1D,0x24,c1,c2);


	x1=0x07;
	x2=0x0E;
	c1 = ((choosenMode == 5) ? 0x0C : 0x00);
	c2 = ((choosenMode == 5) ? 0x0C : 0x00);
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,x1,x2,0x11,0x18,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x11,0x18,c1,c2);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=18),(x2+=18),0x11,0x18,c1,c2);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x11,0x18,c1,c2);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x11,0x18,c1,c2);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,(x1+=9),(x2+=9),0x11,0x18,c1,c2);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,(x1+=9),(x2+=9),0x11,0x18,c1,c2);
}

void helligkeitsNavigating(int value){
	unsigned j;
	int zehnerStelle = value/10;
	if(value==100){
		writeNumbers(1,0x1E,0x25,0x36,0x3d,0x00,0x00);
		writeNumbers(0,0x27,0x2e,0x36,0x3d,0x00,0x00);
		writeNumbers(0,0x30,0x37,0x36,0x3d,0x00,0x00);
	}else if(zehnerStelle > 0){
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,0x1E,0x25,0x36,0x3d,0xFF,0xFF);
		writeNumbers(zehnerStelle,0x27,0x2e,0x36,0x3d,0x00,0x00);
		writeNumbers(value-(zehnerStelle*10),0x30,0x37,0x36,0x3d,0x00,0x00);
	}else{
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,0x1E,0x25,0x36,0x3d,0xFF,0xFF);
		writeLetter(emtLine1,emtLine2,emtLine3,emtLine4,emtLine5,emtLine6,emtLine7,emtLine8,0x27,0x2e,0x36,0x3d,0xFF,0xFF);
		writeNumbers(value,0x30,0x37,0x36,0x3d,0x00,0x00);
	}
	writeLetter(proLine1,proLine2,proLine3,proLine4,proLine5,proLine6,proLine7,proLine8,0x39,0x40,0x36,0x3d,0x00,0x00);
}

void mainMenuNavigating(int menuePoint){
	unsigned j;
	switch(menuePoint){
		case 0:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 1:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 2:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			break;
		case 3:
			OLED_C_MemorySize(0x02,0x05,0x44,0x48);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x37,0x3B);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x2A,0x2E);
			DDRAM_access();
			for(j=0;j<20;j++){
		    	  if(j<4 || j>15 || (j%4)==3|| (j%4)==0){
		        	  OLED_C_Color(0x00,0x00);
		    	  }else{
		        	  OLED_C_Color(0xFF,0xFF);
		    	  }
			}
			OLED_C_MemorySize(0x02,0x05,0x1E,0x22);
			DDRAM_access();
			for(j=0;j<20;j++){
		        	  OLED_C_Color(0x00,0x00);
			}
			break;
	}
}

void paintMenue(){
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,0x1E,0x25,0x51,0x58,0x00,0x00);
	writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x28,0x2F,0x51,0x58,0x00,0x00);
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,0x32,0x39,0x51,0x58,0x00,0x00);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x3C,0x43,0x51,0x58,0x00,0x00);

	writeLetter(fLine1,fLine2,fLine3,fLine4,fLine5,fLine6,fLine7,fLine8,0x07,0x0E,0x43,0x4A,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,0x10,0x17,0x43,0x4A,0x00,0x00);
	writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,0x19,0x20,0x43,0x4A,0x00,0x00);
	writeLetter(bLine1,bLine2,bLine3,bLine4,bLine5,bLine6,bLine7,bLine8,0x22,0x29,0x43,0x4A,0x00,0x00);
	writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x2B,0x32,0x43,0x4A,0x00,0x00);

	writeLetter(hLine1,hLine2,hLine3,hLine4,hLine5,hLine6,hLine7,hLine8,0x07,0x0E,0x36,0x3D,0x00,0x00);
	writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x10,0x17,0x36,0x3D,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,0x19,0x20,0x36,0x3D,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,0x22,0x29,0x36,0x3D,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x2B,0x32,0x36,0x3D,0x00,0x00);
	writeLetter(gLine1,gLine2,gLine3,gLine4,gLine5,gLine6,gLine7,gLine8,0x34,0x3B,0x36,0x3D,0x00,0x00);
	writeLetter(kLine1,kLine2,kLine3,kLine4,kLine5,kLine6,kLine7,kLine8,0x3D,0x44,0x36,0x3D,0x00,0x00);
	writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x46,0x4D,0x36,0x3D,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x4F,0x56,0x36,0x3D,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x58,0x5F,0x36,0x3D,0x00,0x00);

	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,0x07,0x0E,0x29,0x30,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,0x10,0x17,0x29,0x30,0x00,0x00);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,0x19,0x20,0x29,0x30,0x00,0x00);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x22,0x29,0x29,0x30,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x2B,0x32,0x29,0x30,0x00,0x00);

	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x07,0x0E,0x1D,0x24,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x10,0x17,0x1D,0x24,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,0x19,0x20,0x1D,0x24,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x22,0x29,0x1D,0x24,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x2B,0x32,0x1D,0x24,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x34,0x3B,0x1D,0x24,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x3D,0x44,0x1D,0x24,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x46,0x4D,0x1D,0x24,0x00,0x00);
	writeLetter(kLine1,kLine2,kLine3,kLine4,kLine5,kLine6,kLine7,kLine8,0x4F,0x56,0x1D,0x24,0x00,0x00);
}


void snakeFxn(){
	unsigned j;
//	int input = 0;
	int oldInput = 0;
	int value;
	int menuePoint = 0;
	int menuTyp = 0;
	int helligkeitsValue = 100;
	int modusMenuePoint = 0;
	int isFarbwahl=1;
	int choosenMode=5;
        int uVal=1234;
        int iVal=123;
        int lVal=12;
	int first=1;
	/*
	 * 0 MainMenu
	 * 1 Farbe
	 * 2 Helligkeit
	 * 3 Modus
	 * 4 Statistik
	 */
	/*
	 * 0 Neutral
	 * 1 UP
	 * 2 Down
	 * 3 Left
	 * 4 Right
	 * 5 Drücken
	 */

    while(1){
        client.subscribe("quiz",MQTT::QOS0,messageArrived);
        
    	switch(menuTyp){
    	case 0:
    		if(first){
        		paintMenue();
        		mainMenuNavigating(menuePoint);
        		first=0;
        	}
        	if(input==1){
        		menuePoint++;
        		if(menuePoint>3){
        			menuePoint=0;
        		}
            	mainMenuNavigating(menuePoint);
        	}else if(input==2){
        		menuePoint--;
        		if(menuePoint<0){
        			menuePoint=3;
        		}
            	mainMenuNavigating(menuePoint);
        	}else if(input==3 || input==5){
        		menuTyp=menuePoint+1;
        		if(menuePoint==0 && !isFarbwahl){
        			break;
        		}
        	      OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
        	      DDRAM_access();
        	      for(j=0;j<9216;j++){
        	    	  OLED_C_Color(0xFF,0xFF);
        	      }
        	      first=1;
        	}
    		break;
    	case 1:
    		if(isFarbwahl){
    			if(input==4){
					menuTyp=0;
					OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
					DDRAM_access();
					for(j=0;j<9216;j++){
						OLED_C_Color(0xFF,0xFF);
					}
					first=1;
				}
    		}else{
    			//paintMenue();
    			int x1=0x04;
    			int x2=0x0B;
    			writeLetter(wLine1,wLine2,wLine3,wLine4,wLine5,wLine6,wLine7,wLine8,x1,x2,0x05,0x0C,0x00,0x00);
    			writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			writeLetter(gLine1,gLine2,gLine3,gLine4,gLine5,gLine6,gLine7,gLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=18),(x2+=18),0x05,0x0C,0x00,0x00);
    			writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x05,0x0C,0x00,0x00);
    			menuTyp=0;
    		}
    		break;
    	case 2:
        	if(first){
        		writeLetter(hLine1,hLine2,hLine3,hLine4,hLine5,hLine6,hLine7,hLine8,0x07,0x0E,0x51,0x58,0x00,0x00);
        		writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x10,0x17,0x51,0x58,0x00,0x00);
        		writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,0x19,0x20,0x51,0x58,0x00,0x00);
        		writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,0x22,0x29,0x51,0x58,0x00,0x00);
        		writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x2B,0x32,0x51,0x58,0x00,0x00);
        		writeLetter(gLine1,gLine2,gLine3,gLine4,gLine5,gLine6,gLine7,gLine8,0x34,0x3B,0x51,0x58,0x00,0x00);
        		writeLetter(kLine1,kLine2,kLine3,kLine4,kLine5,kLine6,kLine7,kLine8,0x3D,0x44,0x51,0x58,0x00,0x00);
        		writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x46,0x4D,0x51,0x58,0x00,0x00);
        		writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x4F,0x56,0x51,0x58,0x00,0x00);
        		writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x58,0x5F,0x51,0x58,0x00,0x00);
        		helligkeitsNavigating(helligkeitsValue);
        	}
        	if(input==1){
        		helligkeitsValue--;
        		if(helligkeitsValue<0){
        			helligkeitsValue=100;
        		}
        	}else if(input==2){
        		helligkeitsValue++;
        		if(helligkeitsValue>100){
        			helligkeitsValue=0;
        		}
        	}else if(input==4){
        		menuTyp=0;
				OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
				DDRAM_access();
				for(j=0;j<9216;j++){
					OLED_C_Color(0xFF,0xFF);
				}
        		first=1;
        	}
    		break;
    	case 3:
    		if(first){
    			printModus(choosenMode);
        		modusNavigation(modusMenuePoint);
    			first=0;
    		}
        	if(input==1){
        		modusMenuePoint++;
        		if(modusMenuePoint>4){
        			modusMenuePoint=0;
        		}
        		modusNavigation(modusMenuePoint);
        	}else if(input==2){
        		modusMenuePoint--;
        		if(modusMenuePoint<0){
        			modusMenuePoint=4;
        		}
        		modusNavigation(modusMenuePoint);
        	}else if(input==3 || input==5){
    			if(modusMenuePoint==4){
    				isFarbwahl=1;
    			}else{
    				isFarbwahl=0;
    			}
    			choosenMode=modusMenuePoint+1;
    			printModus(choosenMode);
    			//sendValue to xmega
    		}else if(input==4){
        		menuTyp=0;
				OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
				DDRAM_access();
				for(j=0;j<9216;j++){
					OLED_C_Color(0xFF,0xFF);
				}
        		first=1;
        	}
    		break;
    	case 4:
    		if(first){
                        //getValues
    			printStatistik(uVal,iVal,lVal);
    			first=0;
    		}
    		if(input==3 || input==5){
    			//refresh Values
                        uVal=1;
                        iVal=1;
                        lVal=1;
    			printStatistik(uVal,iVal,lVal);
    		}else if(input==4){
        		menuTyp=0;
				OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
				DDRAM_access();
				for(j=0;j<9216;j++){
					OLED_C_Color(0xFF,0xFF);
				}
        		first=1;
        	}
    		break;
    	}
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
  
}
