#include "StartBIOS.h"


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

void PortJIntHandler(){

	uint32_t status=0;

	status = GPIOIntStatus(GPIO_PORTJ_BASE,true);
	GPIOIntClear(GPIO_PORTJ_BASE,status);

	if(status & GPIO_INT_PIN_0 == GPIO_INT_PIN_0){
		int status = CLICK;
		Mailbox_post(mbox, &status, BIOS_WAIT_FOREVER);
	}
}

// Init MCU function
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

	SPI_init();

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
	SPI_init();

    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);

	GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

	SysCtlPeripheralEnable(GPIO_PORTP_BASE);
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);

	SSIClockSourceSet(SSI3_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(SSI3_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI3_BASE);

/*
    SysCtlPeripheralEnable(GPIO_PORTJ_BASE);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTJ_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);*/

    //GPIOIntRegister(GPIO_PORTJ_BASE,PortJIntHandler);

    //GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);
}


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
}

//Sekvence before writing data to memory
void DDRAM_access(){
	GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 0);//OLED_CS = 0;
	GPIOPinWrite (GPIO_PORTM_BASE, GPIO_PIN_3, 0);//OLED_DC = 0;
    SPI2_Write(0x08);
    GPIOPinWrite (GPIO_PORTH_BASE, GPIO_PIN_2, 4);//OLED_CS = 1;
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

void printStatistik(){
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x09,0x10,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x12,0x19,0x51,0x58,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,0x1B,0x22,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x24,0x2B,0x51,0x58,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x2D,0x34,0x51,0x58,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,0x36,0x3D,0x51,0x58,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,0x3F,0x46,0x51,0x58,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,0x48,0x4F,0x51,0x58,0x00,0x00);
	writeLetter(kLine1,kLine2,kLine3,kLine4,kLine5,kLine6,kLine7,kLine8,0x51,0x58,0x51,0x58,0x00,0x00);

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

void printModus(){
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
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,x1,x2,0x43,0x4A,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);
	writeLetter(bLine1,bLine2,bLine3,bLine4,bLine5,bLine6,bLine7,bLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);
	writeLetter(yLine1,yLine2,yLine3,yLine4,yLine5,yLine6,yLine7,yLine8,(x1+=9),(x2+=9),0x43,0x4A,0x00,0x00);


	x1=0x07;
	x2=0x0E;
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,x1,x2,0x36,0x3d,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(aLine1,aLine2,aLine3,aLine4,aLine5,aLine6,aLine7,aLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);
	writeLetter(pLine1,pLine2,pLine3,pLine4,pLine5,pLine6,pLine7,pLine8,(x1+=9),(x2+=9),0x36,0x3d,0x00,0x00);


	x1=0x07;
	x2=0x0E;
	writeLetter(cLine1,cLine2,cLine3,cLine4,cLine5,cLine6,cLine7,cLine8,x1,x2,0x29,0x30,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(wLine1,wLine2,wLine3,wLine4,wLine5,wLine6,wLine7,wLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(rLine1,rLine2,rLine3,rLine4,rLine5,rLine6,rLine7,rLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x29,0x30,0x00,0x00);


	x1=0x07;
	x2=0x0E;
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,x1,x2,0x1D,0x24,0x00,0x00);
	writeLetter(iLine1,iLine2,iLine3,iLine4,iLine5,iLine6,iLine7,iLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(gLine1,gLine2,gLine3,gLine4,gLine5,gLine6,gLine7,gLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(hLine1,hLine2,hLine3,hLine4,hLine5,hLine6,hLine7,hLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(tLine1,tLine2,tLine3,tLine4,tLine5,tLine6,tLine7,tLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(fLine1,fLine2,fLine3,fLine4,fLine5,fLine6,fLine7,fLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(lLine1,lLine2,lLine3,lLine4,lLine5,lLine6,lLine7,lLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);
	writeLetter(wLine1,wLine2,wLine3,wLine4,wLine5,wLine6,wLine7,wLine8,(x1+=9),(x2+=9),0x1D,0x24,0x00,0x00);


	x1=0x07;
	x2=0x0E;
	writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,x1,x2,0x11,0x18,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x11,0x18,0x00,0x00);
	writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,(x1+=18),(x2+=18),0x11,0x18,0x00,0x00);
	writeLetter(oLine1,oLine2,oLine3,oLine4,oLine5,oLine6,oLine7,oLine8,(x1+=9),(x2+=9),0x11,0x18,0x00,0x00);
	writeLetter(dLine1,dLine2,dLine3,dLine4,dLine5,dLine6,dLine7,dLine8,(x1+=9),(x2+=9),0x11,0x18,0x00,0x00);
	writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,(x1+=9),(x2+=9),0x11,0x18,0x00,0x00);
	writeLetter(sLine1,sLine2,sLine3,sLine4,sLine5,sLine6,sLine7,sLine8,(x1+=9),(x2+=9),0x11,0x18,0x00,0x00);
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
	int input = 0;
	int oldInput = 0;
	time_t t;
	time(&t);
	srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
	int value;
	int menuePoint = 0;
	int menuTyp = 0;
	int helligkeitsValue = 100;
	int modusMenuePoint = 0;
	int isFarbwahl=0;
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
    	Task_sleep(500);
    	Mailbox_pend(mbox,&input,10);
    	if(input == oldInput){
    		Mailbox_pend(mbox,&input,10);
    	}
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
        	}else if(input==3){
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
    			printModus();
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
        	}else if(input==3){
    			if(modusMenuePoint==4){
    				isFarbwahl=1;
    			}else{
    				isFarbwahl=0;
    			}
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
    			printStatistik();
    			first=0;
    		}
    		if(input==3){
    			//refresh Values
    			printStatistik();
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


int setup_Snake_Task()
{
	Task_Params taskLedParams;
	Task_Handle taskLed;
	Error_Block eb;

    Error_init(&eb);
    Task_Params_init(&taskLedParams);
    taskLedParams.stackSize = 1024;/*stack in bytes*/
    taskLedParams.priority = 14;/*0-15 (15 is highest priority on default -> see RTOS Task configuration)*/
    taskLed = Task_create((Task_FuncPtr)snakeFxn, &taskLedParams, &eb);
    if (taskLed == NULL) {
    	System_abort("TaskLed create failed");
    }

    return (0);
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
			Mailbox_post(mbox, &status, BIOS_WAIT_FOREVER);
			Task_sleep(200);
		}

		System_printf("UP_DOWN: %d\n", up_down);
		System_printf("LEFT_RIGHT: %d\n", left_right);
		System_flush();
		Task_sleep(TASK_SLEEP_POLLING);
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
	Mailbox_post(mbox, &status, BIOS_WAIT_FOREVER);
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
}

void main(){
      InitBoard();
      OLED_C_Init();


  	// Configure Mailbox
  	Error_Block eb_mb;
  	Error_init(&eb_mb);
  	Mailbox_Params mboxParams;
  	Mailbox_Params_init(&mboxParams);
  	mbox = Mailbox_create(sizeof(int), 1, &mboxParams, &eb_mb);

  	if (mbox == NULL) {
  		System_abort("Creation of mailbox failed!");
  	}




      unsigned j;

      OLED_C_command(0x1D,0x02);                //Set Memory Read/Write mode

      OLED_C_MemorySize(0x00,0x5F,0x00,0x5F);
      DDRAM_access();
      for(j=0;j<9216;j++){
    	  OLED_C_Color(0xFF,0xFF);
      }

		 //2 - 93(5D) = 90 => Spielfeld
		 //OLED_C_MemorySize(0x02,0x5D,0x02,0x5D);
		writeLetter(mLine1,mLine2,mLine3,mLine4,mLine5,mLine6,mLine7,mLine8,0x1E,0x25,0x51,0x58,0x00,0x00);
		writeLetter(eLine1,eLine2,eLine3,eLine4,eLine5,eLine6,eLine7,eLine8,0x28,0x2F,0x51,0x58,0x00,0x00);
		writeLetter(nLine1,nLine2,nLine3,nLine4,nLine5,nLine6,nLine7,nLine8,0x32,0x39,0x51,0x58,0x00,0x00);
		writeLetter(uLine1,uLine2,uLine3,uLine4,uLine5,uLine6,uLine7,uLine8,0x3C,0x43,0x51,0x58,0x00,0x00);

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

      setup_Snake_Task();


      uint32_t ui32SysClock;
      /* Ruft Board Init funktion auf */
      ui32SysClock = Board_initGeneral(120*1000*1000);

      /* SysMin will only print to the console upon calling flush or exit */
      System_printf("Start BIOS\n");
      System_flush();

	Error_Block eb;
	Error_init(&eb);
	Task_Params taskJoystickParams;
	Task_Params_init(&taskJoystickParams);
	Task_Handle taskJoystick;

	taskJoystickParams.stackSize = 1024;
	taskJoystickParams.priority = 15;

    taskJoystick = Task_create((Task_FuncPtr)getJoystickDirection, &taskJoystickParams, &eb);

    if (taskJoystick == NULL) {
    	System_abort("taskJoystick create failed!");
    }

    // Interrupt for button press
    Error_Block ebHWI;
    Error_init(&ebHWI);
    Hwi_Params HWIParams;
    Hwi_Handle HWIHandle;
    Hwi_Params_init(&HWIParams);
    HWIParams.arg = 0;
    HWIParams.enableInt = true;

    HWIHandle = Hwi_create(INT_GPIOA_TM4C129, clickInterrupt, &HWIParams, &ebHWI);

    if (HWIHandle == NULL) {
    	System_abort("HWI create failed!");
    }

    Hwi_enableInterrupt(INT_GPIOA_TM4C129);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_0);

    // Start BIOS
    BIOS_start();
    Mailbox_delete(&mbox);

    return;
}
