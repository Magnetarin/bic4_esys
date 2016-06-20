/*
 * StartBIOS.h
 *
 *  Created on: 19.01.2016
 *      Author: Maxima
 */



#ifndef STARTBIOS_H_
#define STARTBIOS_H_


void getJoystickDirection();
void clickInterrupt();
unsigned int getAverageADC(unsigned short channel);
unsigned int getADCValue(unsigned short channel);


/*
 * Schickt Daten vom Master(unserem Board) zum Slave(OLED C) �ber die SPI2 schnittstelle.
 *
 * @param input ... Daten die ich schicken m�chte
 */
void SPI2_Write(unsigned char input);

/*
 * Verz�gert das Programm um ms Millisekunden mit hilfe von
 *
 * @param ms ... Millisekunden die ich ein Programm verz�gern m�chte
 */
void delay_ms(double ms);

/*
 * Initialisiert alle Pins am Board die f�r die Kommunikation mit dem Slave(OLED C) n�tig sind.
 * Desweitern wird auch das SPI initialisiert.
 */
void InitBoard();

/*
 * Schickt kommands zum OLED C display
 *
 * @param reg_index ... Wert der zu selectierenden index addr
 * @param reg_value ... Wert f�rs Kommandregister
 */
void OLED_C_command(unsigned char reg_index, unsigned char reg_value);

/*
 * Schickt Daten zum OLED C display
 *
 * @param ... dataValue
 */
void OLED_C_data(unsigned char data_value);

/*
 * Initialisierung des OLED C display
 */
void OLED_C_Init();

/*
 * Selectieren des Indexregisters 0x08 um dann ins DDRAM schreiben zu k�nnen.
 */
void DDRAM_access();

/*
 * Setzt den Memorybereich in dem geschrieben werden soll.
 *
 * @param X1 ... X Koordinate des "Startpunktes"
 * @param X2 ... X Koordinate des "Endpunktes"
 * @param Y1 ... Y Koordinate des "Startpunktes"
 * @param Y2 ... Y Koordinate des "Endpunktes"
 */
void OLED_C_MemorySize(char X1, char X2, char Y1, char Y2);

/*
 * Bestimmt die Farbe mit der ein pixel geschrieben werden soll.
 *
 * @param colorMSB ... erster Teil der Farbe
 * @param colorLSB ... zweiter Teil der Farbe
 */
void OLED_C_Color(char colorMSB, char colorLSB );

/*
 * Schreibt eine Zeile des 8x8 gro�en Buchstaben.
 *
 * @param line ... Zeile die geschrieben werden soll
 * @param forground1,forground2 ... define Color for forground
 */
void writeLetterLine(int line,char forground1,char forground2);

/*
 * Schreibt einen 8x8 gro�en Buchstaben.
 *
 * @param line1-line8 ... Zeile 1 - 8 des Buchstaben
 * @param X1,X2,Y1,Y2 ... Koordinaten des 8x8 Feldes
 * @param forground1,forground2 ... define Color for forground
 */
void writeLetter(int line1,int line2,int line3,int line4,int line5,int line6,int line7,int line8,int X1,int X2,int Y1,int Y2,char forground1,char forground2);

/*
 * Game Task
 */
void gameFxn();

/*
 * Setup Game Task function
 */
int setup_Game_Task();


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

Mailbox_Handle mbox;

#endif /* STARTBIOS_H_ */
