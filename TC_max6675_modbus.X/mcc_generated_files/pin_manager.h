/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F46Q10
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set pinTC_sdi aliases
#define pinTC_sdi_TRIS                 TRISBbits.TRISB0
#define pinTC_sdi_LAT                  LATBbits.LATB0
#define pinTC_sdi_PORT                 PORTBbits.RB0
#define pinTC_sdi_WPU                  WPUBbits.WPUB0
#define pinTC_sdi_OD                   ODCONBbits.ODCB0
#define pinTC_sdi_ANS                  ANSELBbits.ANSELB0
#define pinTC_sdi_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define pinTC_sdi_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define pinTC_sdi_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define pinTC_sdi_GetValue()           PORTBbits.RB0
#define pinTC_sdi_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define pinTC_sdi_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define pinTC_sdi_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define pinTC_sdi_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define pinTC_sdi_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define pinTC_sdi_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define pinTC_sdi_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define pinTC_sdi_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set pinTC_sck aliases
#define pinTC_sck_TRIS                 TRISBbits.TRISB1
#define pinTC_sck_LAT                  LATBbits.LATB1
#define pinTC_sck_PORT                 PORTBbits.RB1
#define pinTC_sck_WPU                  WPUBbits.WPUB1
#define pinTC_sck_OD                   ODCONBbits.ODCB1
#define pinTC_sck_ANS                  ANSELBbits.ANSELB1
#define pinTC_sck_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define pinTC_sck_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define pinTC_sck_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define pinTC_sck_GetValue()           PORTBbits.RB1
#define pinTC_sck_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define pinTC_sck_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define pinTC_sck_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define pinTC_sck_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define pinTC_sck_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define pinTC_sck_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define pinTC_sck_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define pinTC_sck_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set pinTC_cs aliases
#define pinTC_cs_TRIS                 TRISBbits.TRISB2
#define pinTC_cs_LAT                  LATBbits.LATB2
#define pinTC_cs_PORT                 PORTBbits.RB2
#define pinTC_cs_WPU                  WPUBbits.WPUB2
#define pinTC_cs_OD                   ODCONBbits.ODCB2
#define pinTC_cs_ANS                  ANSELBbits.ANSELB2
#define pinTC_cs_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define pinTC_cs_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define pinTC_cs_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define pinTC_cs_GetValue()           PORTBbits.RB2
#define pinTC_cs_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define pinTC_cs_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define pinTC_cs_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define pinTC_cs_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define pinTC_cs_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define pinTC_cs_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define pinTC_cs_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define pinTC_cs_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RC6_SetPullup()             do { WPUCbits.WPUC6 = 1; } while(0)
#define RC6_ResetPullup()           do { WPUCbits.WPUC6 = 0; } while(0)
#define RC6_SetAnalogMode()         do { ANSELCbits.ANSELC6 = 1; } while(0)
#define RC6_SetDigitalMode()        do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSELC7 = 0; } while(0)

// get/set pinLed_Test aliases
#define pinLed_Test_TRIS                 TRISDbits.TRISD0
#define pinLed_Test_LAT                  LATDbits.LATD0
#define pinLed_Test_PORT                 PORTDbits.RD0
#define pinLed_Test_WPU                  WPUDbits.WPUD0
#define pinLed_Test_OD                   ODCONDbits.ODCD0
#define pinLed_Test_ANS                  ANSELDbits.ANSELD0
#define pinLed_Test_SetHigh()            do { LATDbits.LATD0 = 1; } while(0)
#define pinLed_Test_SetLow()             do { LATDbits.LATD0 = 0; } while(0)
#define pinLed_Test_Toggle()             do { LATDbits.LATD0 = ~LATDbits.LATD0; } while(0)
#define pinLed_Test_GetValue()           PORTDbits.RD0
#define pinLed_Test_SetDigitalInput()    do { TRISDbits.TRISD0 = 1; } while(0)
#define pinLed_Test_SetDigitalOutput()   do { TRISDbits.TRISD0 = 0; } while(0)
#define pinLed_Test_SetPullup()          do { WPUDbits.WPUD0 = 1; } while(0)
#define pinLed_Test_ResetPullup()        do { WPUDbits.WPUD0 = 0; } while(0)
#define pinLed_Test_SetPushPull()        do { ODCONDbits.ODCD0 = 0; } while(0)
#define pinLed_Test_SetOpenDrain()       do { ODCONDbits.ODCD0 = 1; } while(0)
#define pinLed_Test_SetAnalogMode()      do { ANSELDbits.ANSELD0 = 1; } while(0)
#define pinLed_Test_SetDigitalMode()     do { ANSELDbits.ANSELD0 = 0; } while(0)

// get/set RD6 procedures
#define RD6_SetHigh()            do { LATDbits.LATD6 = 1; } while(0)
#define RD6_SetLow()             do { LATDbits.LATD6 = 0; } while(0)
#define RD6_Toggle()             do { LATDbits.LATD6 = ~LATDbits.LATD6; } while(0)
#define RD6_GetValue()              PORTDbits.RD6
#define RD6_SetDigitalInput()    do { TRISDbits.TRISD6 = 1; } while(0)
#define RD6_SetDigitalOutput()   do { TRISDbits.TRISD6 = 0; } while(0)
#define RD6_SetPullup()             do { WPUDbits.WPUD6 = 1; } while(0)
#define RD6_ResetPullup()           do { WPUDbits.WPUD6 = 0; } while(0)
#define RD6_SetAnalogMode()         do { ANSELDbits.ANSELD6 = 1; } while(0)
#define RD6_SetDigitalMode()        do { ANSELDbits.ANSELD6 = 0; } while(0)

// get/set RD7 procedures
#define RD7_SetHigh()            do { LATDbits.LATD7 = 1; } while(0)
#define RD7_SetLow()             do { LATDbits.LATD7 = 0; } while(0)
#define RD7_Toggle()             do { LATDbits.LATD7 = ~LATDbits.LATD7; } while(0)
#define RD7_GetValue()              PORTDbits.RD7
#define RD7_SetDigitalInput()    do { TRISDbits.TRISD7 = 1; } while(0)
#define RD7_SetDigitalOutput()   do { TRISDbits.TRISD7 = 0; } while(0)
#define RD7_SetPullup()             do { WPUDbits.WPUD7 = 1; } while(0)
#define RD7_ResetPullup()           do { WPUDbits.WPUD7 = 0; } while(0)
#define RD7_SetAnalogMode()         do { ANSELDbits.ANSELD7 = 1; } while(0)
#define RD7_SetDigitalMode()        do { ANSELDbits.ANSELD7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/