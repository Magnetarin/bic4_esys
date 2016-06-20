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
#include <string> 

#define TASK_SLEEP_UART 100
#define TASK_SLEEP_POLLING 100
#define THRESHOLD_UPPER 2300
#define THRESHOLD_LOWER 1800
#define NEUTRAL 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define CLICK 5
#define F_CPU 16000000

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


void getJoystickDirection() {
	unsigned int up_down, left_right;
	int status;
	//struct StickStatus status;

	while (1) {
		up_down = getAverageADC(0);
		left_right = getAverageADC(1);

		if (up_down < THRESHOLD_LOWER) {
			status = UP;
		} else if (up_down > THRESHOLD_UPPER) {
			status = DOWN;
		} else if (left_right < THRESHOLD_LOWER) {
			status = LEFT;
		} else if (left_right > THRESHOLD_UPPER) {
			status = RIGHT;
		} else {
			status = NEUTRAL;
		}

		if(status > 0){
			//Mailbox_post(mbox, &status, BIOS_WAIT_FOREVER);
                        //Publish  
                        MQTT::Message message;
                        char buf[100];
                        //std::string s = std::to_string(status);
                        char c = (int)status;
                        Serial.print("status: ");
                        Serial.println(status,DEC);
                        sprintf(buf, "%d", status);
                        //buf[0]=status;
                        Serial.println(buf);
                        message.qos = MQTT::QOS0;
                        message.retained = false;
                        message.dup = false;
                        message.payload = (void*)buf;
                        message.payloadlen = strlen(buf)+1;
                        int rc = client.publish(topic, message);
		}

		/*Serial.print("UP_DOWN: ");
                Serial.println(up_down,DEC);
		Serial.print("LEFT_RIGHT: ");
		Serial.println(left_right,DEC);*/
		//System_flush();
		//Task_sleep(TASK_SLEEP_POLLING);
	}
}


unsigned int getAverageADC(unsigned short channel) {
	unsigned int tmp = 0;

	tmp += getADCValue(channel);
	tmp += getADCValue(channel);
	tmp += getADCValue(channel);
	tmp += getADCValue(channel);
	tmp += getADCValue(channel);

	return tmp / 5;
}

unsigned int getADCValue(unsigned short channel) {
	// Returns 0..4095
	uint32_t tmp = 0;
	uint32_t tmp2 = 0;

	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_5, 0);  //set cs low
	SSIDataPut(SSI3_BASE, 0x06);  //send address // SPI communication using 8-bit segments
	channel = channel << 6;  // Bits 7 & 6 define ADC input
	SSIDataPut(SSI3_BASE, channel);
	while (SSIBusy(SSI3_BASE));
	while (SSIDataGetNonBlocking(SSI3_BASE,&tmp));
	tmp2 = tmp & 0x0F;
	tmp2 = tmp2 << 8;  // Shift ADC value by 8
	while (SSIDataGetNonBlocking(SSI3_BASE,&tmp));

	tmp2 |= tmp;

	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_5, GPIO_PIN_5);
	return tmp2;  // Returns 12-bit ADC value
}

void clickInterrupt() {
	int status = CLICK;
	//Mailbox_post(mbox, &status, BIOS_WAIT_FOREVER);
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
}

void InitBoard(){
	//Init Board for OLED_C
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

	//SPI_init();

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


	// Board for Joy Stick
	// Enable inner booster port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
//	SPI_init();
        SPI.begin();


    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);

	GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

	SysCtlPeripheralEnable(GPIO_PORTP_BASE);
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);

	SSIClockSourceSet(SSI3_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(SSI3_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI3_BASE);
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
  
  Serial.println("MQTT Hello example");

                     
  char buf[100];
  sprintf(buf, "Joystick START MUHAHAHA");
    

}

void loop()
{
  
  getJoystickDirection();
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
