/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM™ FS USB Demo Board,
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.
 
 Software License Agreement:
 
 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.
 
 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
********************************************************************
 File Description:
 
 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/
 
 
//  ========================    INCLUDES    ========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL
 
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
 
#include "mtouch.h"
 
#include "BMA150.h"
 
#include "oled.h"
 
#include "soft_start.h"
 
 
//  ========================    CONFIGURATION   ========================
 
#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
 
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif
 
 
 
//  ========================    Global VARIABLES    ========================
#pragma udata
//You can define Global Data Elements here
 
//  ========================    PRIVATE PROTOTYPES  ========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();
 
BOOL CheckButtonPressed(void);
 
//  ========================    VECTOR REMAPPING    ========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
 
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
 
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
 
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
 
  //To fix this situation, we should always deliberately place a
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"
 
  #pragma code
 
//  ========================    Application Interrupt Service Routines  ========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
 
  } //This return will be a "retfie fast", since this is in a #pragma interrupt section
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
 
  } //This return will be a "retfie", since this is in a #pragma interruptlow section
#endif
 
 
 
 
//  ========================    Board Initialization Code   ========================
#pragma code
#define ROM_STRING rom unsigned char*
 
/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
  /* Initialize the mTouch library */
  mTouchInit();
 
  /* Call the mTouch callibration function */
  mTouchCalibrate();
 
  /* Initialize the accelerometer */
  InitBma150();
 
  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
 
}//end UserInit
 
 
/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    // Soft Start the APP_VDD
   while(!AppPowerReady())
        ;
 
    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
  //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  //This allows the device to power up at a lower initial operating frequency, which can be
  //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
  //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif
 
    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
   
    UserInit();
 
}//end InitializeSystem
 
 
 
//  ========================    Application Code    ========================
 
 
/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
static char toprint[24];
void mainTraverse(int c);

BOOL CheckButtonPressed(void)
{
   if(PORTBbits.RB0 == 0) 
		return TRUE;
	else 
		return FALSE;
}
 
int GetA2D(){   //Set ADCON and start input
    BYTE x;
    int a2d;
    x=2;
    ADCON0 = 0b00010011;
    while(x){
    x=x&ADCON0;
    }
    a2d=(int)ADRESH;
    a2d= a2d<<8;          
    a2d+=(int)ADRESL;
    return a2d;
}
 
int printPbar(int p){   //Print the potentiometer's bar
    int x=p/64;
    int i;
    for(i=0;i<x;i++){
        oledPutROMString("=", 0, (i+5)*6);  //Change the output according to the potentiometer's readings
    }
    return x;
}
 
int CheckLRVolt(unsigned int x){
    if(x>600)
        return 0;
    else return 1;
}
 
void CheckLR(){
    unsigned char RA0='0',RA3='3';;
    sprintf(toprint,"L: %d     R: %d",CheckLRVolt(mTouchReadButton(RA3)),CheckLRVolt(mTouchReadButton(RA0)));
    oledPutString(toprint,5,0,1);
}
 
int CheckUDVolt(unsigned int x,unsigned int y){
    if(x<y && y-x>75 && x < 600){
        return 1;       //Up
	}
    else if (y<x && x-y>75 && y < 600){ 
		return 2;}       //Down
   else return 0;   //Not pushed
}
 
 
int GetAccVal(char c){  //Check the accelerometer's readings
/*
x: msb 3 lsb 2
y: msb 5 lsb 4
z: msb 7 lsb 6
*/
BYTE msb,lsb;
BYTE mask= 0b10000000;
int signextend= 0xFC00;//2^15+2^14+2^13+2^12+2^11+2^10+2^9
int val=0,n1,n2;
    if(c=='x'){n1=3;n2=2;}
    if(c=='y'){n1=5;n2=4;}
    if(c=='z'){n1=7;n2=6;}
    msb=BMA150_ReadByte(n1);
    lsb=BMA150_ReadByte(n2);
    lsb=lsb>>6;
    val+=(int)msb;
    val=val<<2;
    val+=(int)lsb;
    mask= mask&msb;
    if(mask== 0b10000000){
        val|=signextend;
    }
    return val;
}
int truev(int x){
    if(x<0) return -x;
    else return x;
}
 
 
void putmax(int x,int y){   //Print the accelerometer's max vector values
    int mx,my;
    int rowx=8,rowy=8;
    mx=truev(x)/64;
    my=truev(y)/64;
    if(x<0) rowx-=mx;
    else rowx +=mx;
    if(y<0) rowy-=my;
    else rowy +=my;
   
    oledPutROMString("o", 6, rowx*6);
    oledPutROMString("o", 7, rowy*6);
}
void Checkmaxvals(static int x,static int y){
    int z;
    static char toprint[23];
    z=GetAccVal('z');
    if(z<0){
        x=0;
        y=0;
    }
    /*sprintf(toprint,"z%3d",z);
    oledPutString(toprint,4,0);*/
    sprintf(toprint,"x ----------------");//line size =16
    oledPutString(toprint,6,0);
    sprintf(toprint,"y ----------------");
    oledPutString(toprint,7,0);
    putmax(x,y);
}
void CheckBMA(){  
    static char toprint[23];
    int x,y;
    BYTE deg;
    static maxx=0,maxy=0;
    deg=BMA150_ReadByte(8);
    x=GetAccVal('x');
    y=GetAccVal('y');
    if(truev(x)>maxx) maxx=x;
    if(truev(y)>maxy) maxy=y;
    sprintf(toprint,"X:%3d Y:%3d T:%2d~ ",x,y,((deg-32)*5)/9);
    oledPutString(toprint,5,0);
    Checkmaxvals(maxx,maxy);
}
 
void clearScreen(){
	int i;
	for(i=1;i<8;i++){
		oledPutROMString("                                 ", i, 0); 
	}
}
void clearScreen0(){
	int i;
	for(i=0;i<8;i++){
		oledPutROMString("                                 ", i, 0); 
	}
}
void opscreen(int num)
{
    int i;
	unsigned char RA1='1',RA2='2';
clearScreen0();
	while(1)
	{
    	sprintf(toprint,"Operation %d done", num);
    	oledPutString(toprint, 0, 0,1);  
    	sprintf(toprint,"Press up to return");
    	oledPutString(toprint, 1, 0,1);  
 	
  		if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)
		{
			clearScreen0();
			return 0;
		}	


	}
}

void subsubMenu1()
{
    int i , z;
	unsigned char RA1='1',RA2='2';
	int currChoice=1;
	clearScreen();
while(1){
    sprintf(toprint,"Sub Sub menu 1");
    oledPutString(toprint, 0, 0,1);  
   
 	
    for(i=1;i<6;i++)
    {
	    sprintf(toprint, "Execute operation %d",i+4);
	    if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   	else oledPutString(toprint, i ,2*6,1);
		
    }
		

    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)    //Pressed up           
{
	if(currChoice > 1) currChoice--;

} 
    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==2) //Pressed down
{
 	if(currChoice < 5) currChoice++;
}
	//x = GetAccVal('x');
	if( GetAccVal('x') > 60 && GetAccVal('y') > 60 ) // shake to return to exit
{
	clearScreen0();
	return 0;
}
	z = GetAccVal('z');
	if(z > 150 || z <  -150) // tilting the device to select executing
			opscreen(currChoice+4);	
 DelayMs(20);
}
}

void subMenu1()
{
    int i , z;
	unsigned char RA1='1',RA2='2';
	int currChoice=1;
	clearScreen();
while(1){
    sprintf(toprint,"Sub menu 1");
    oledPutString(toprint, 0, 0,1);  
   
 	
    for(i=1;i<6;i++)
    {
		if(i==5) sprintf(toprint, "SubSubMenu");
	    else sprintf(toprint, "Execute operation %d",i);
	 //   sprintf(toprint, "Execute operation %d",i);
	    if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   	else oledPutString(toprint, i ,2*6,1);
		
    }
		

    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)    //Pressed up           
{
	if(currChoice > 1) currChoice--;

} 
    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==2) //Pressed down
{
 	if(currChoice < 5) currChoice++;
}
	//x = GetAccVal('x');
	if( GetAccVal('x') > 60 && GetAccVal('y') > 60 ) // shake to return to main menu
{
	clearScreen0();
	return 0;
}
	z = GetAccVal('z');
	if(z > 150 || z <  -150) // tilting the device to select executing
		if ( currChoice != 5)
			opscreen(currChoice);
		else
			subsubMenu1();
		
 DelayMs(20);
}
}


void subsubMenu2() //potenciometer
{
    int i , z, pot;
	unsigned char RA3='3',RA0='0'; // RA0 is Cancel and RA3 is accept
	int currChoice=1;
	clearScreen();
	while(1)
	{
    	sprintf(toprint,"Sub Sub menu 2");
    	oledPutString(toprint, 0, 0,1);  
   
 	
    	for(i=1;i<6;i++)
    	{
			sprintf(toprint, "Execute operation %d",i+4);
	    	if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   		else oledPutString(toprint, i ,2*6,1);

	    }
		

		pot = GetA2D();
		if(pot < 204)
			currChoice=1;
		else if(pot > 204 && pot < 409)
			currChoice=2;
		else if(pot > 409 && pot < 613)
			currChoice=3;
		else if(pot > 613 && pot < 818)
			currChoice=4;
		else if(pot > 818 && pot < 1023)
			currChoice=5;

		if( CheckLRVolt(mTouchReadButton(RA0)) ) // R to choose
				opscreen(currChoice+4);
		if( CheckLRVolt(mTouchReadButton(RA3)) ) // L to return
		{
			clearScreen0();
			return 0;
		}
	}
}

void subMenu2() //potenciometer
{
    int i , z, pot;
	unsigned char RA3='3',RA0='0'; // RA0 is Cancel and RA3 is accept
	int currChoice=1;
	clearScreen();
	while(1)
	{
    	sprintf(toprint,"Sub menu 2");
    	oledPutString(toprint, 0, 0,1);  
   
 	
    	for(i=1;i<6;i++)
    	{
			if(i==5) sprintf(toprint, "SubSubMenu");
	  	 	else sprintf(toprint, "Execute operation %d",i);
	    	if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   		else oledPutString(toprint, i ,2*6,1);
		}
		

		pot = GetA2D();
		if(pot < 204)
			currChoice=1;
		else if(pot > 204 && pot < 409)
			currChoice=2;
		else if(pot > 409 && pot < 613)
			currChoice=3;
		else if(pot > 613 && pot < 818)
			currChoice=4;
		else if(pot > 818 && pot < 1023)
			currChoice=5;

		if( CheckLRVolt(mTouchReadButton(RA3)) ) // L to return to main menu
		{
		clearScreen0();
		return 0;
		}
		if( CheckLRVolt(mTouchReadButton(RA0)) ) // R to choose
		{
			if ( currChoice != 5)
				opscreen(currChoice);
			else
			{
			 	DelayMs(20);
				subsubMenu2();
			}
		}
		
 		DelayMs(20);
	}
}

void mainTraverse(int c){
	switch(c){
		case 1: subMenu1();break;
		case 2: subMenu2();break;
		//case 3: subMenu3();break;
		//case 4: subMenu4();break;
		default: break;
	}
}
void mainMenu()
{  
    int i;
	unsigned char RA1='1',RA2='2';
	int currChoice=1;
	clearScreen0();
while(1){
    sprintf(toprint,"Main menu");
    oledPutString(toprint, 0, 0,1);  
   
 	
    for(i=1;i<5;i++)
    {
	    sprintf(toprint, "submenu %d",i);
	    if(i == currChoice)oledPutString(toprint, i ,2*6,0);
	   	else oledPutString(toprint, i ,2*6,1);
    }

    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==1)    //Pressed up           
{
	if(currChoice > 1) currChoice--;

} 
    if( CheckUDVolt(mTouchReadButton(RA1),mTouchReadButton(RA2))==2) //Pressed down
{
 	if(currChoice < 4) currChoice++;
}
	if(CheckButtonPressed())
{
	mainTraverse(currChoice);
}
 DelayMs(20);
}
}
 

void main(void)
{
    InitializeSystem();
    mainMenu();
    while(1)                            //Main is Usualy an Endless Loop
    {
 
    }      
}//end main
 
 
/** EOF main.c *************************************************/