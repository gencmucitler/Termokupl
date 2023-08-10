/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F46Q10
        Driver Version    :  2.00
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

#include "mcc_generated_files/mcc.h"
#include "zaman.h"
#include "modbus.h"
#include "max6675.h"

unsigned long once_led_zamani=0;
extern char sprint_temp[64];
extern volatile uint8_t coilReg[COIL_REG_SIZE];
extern volatile uint8_t inputDiscreteReg[INPUT_DISCRETE_REG_SIZE];
extern volatile uint16_t holdingReg[HOLDING_REG_SIZE];
extern volatile uint16_t inputReg[INPUT_REG_SIZE];


/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupt s
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
   TMR1_SetInterruptHandler(_miliSaat);
    
   modbus_init();

   coilReg[0]=0;
   coilReg[1]=0;
   coilReg[2]=0;
   coilReg[3]=0;
   coilReg[4]=1;
   coilReg[5]=1;
   coilReg[6]=1;
   coilReg[7]=1;
   coilReg[8]=0;
   coilReg[9]=1;
   coilReg[10]=0;
   coilReg[11]=1;
   coilReg[12]=0;
   coilReg[13]=1;
   coilReg[14]=0;
   coilReg[15]=1;
 
   inputDiscreteReg[0]=0;
   inputDiscreteReg[1]=0;
   inputDiscreteReg[2]=0;
   inputDiscreteReg[3]=0;
   inputDiscreteReg[4]=1;
   inputDiscreteReg[5]=1;
   inputDiscreteReg[6]=1;
   inputDiscreteReg[7]=1;
   inputDiscreteReg[8]=0;
   inputDiscreteReg[9]=1;
   inputDiscreteReg[10]=0;
   inputDiscreteReg[11]=1;
   inputDiscreteReg[12]=0;
   inputDiscreteReg[13]=1;
   inputDiscreteReg[14]=0;
   inputDiscreteReg[15]=1;
   

   holdingReg[0]=0x1234;
   holdingReg[1]=0x5678;
   holdingReg[2]=0x90AB;
   holdingReg[3]=0xCDEF;
   holdingReg[4]=0x0000;
   holdingReg[5]=0x1111;
   holdingReg[6]=0x2222;
   holdingReg[7]=0x3333;
   holdingReg[8]=0x4444;
   holdingReg[9]=0x5555;
   holdingReg[10]=0x6666;
   holdingReg[11]=0x7777;
   holdingReg[12]=0x8888;
   holdingReg[13]=0x9999;
   holdingReg[14]=0xAAAA;
   holdingReg[15]=0xBBBB;
   holdingReg[16]=0xCCCC;
   holdingReg[17]=0xDDDD;
   holdingReg[18]=0xEEEE;
   holdingReg[19]=0xFFFF;
   
   
   inputReg[0]=0x0034;
   inputReg[1]=0x5678;
   inputReg[2]=0x90AB;
   inputReg[3]=0xCDEF;
   inputReg[4]=0x0000;
   inputReg[5]=0x0011;
   inputReg[6]=0x0022;
   inputReg[7]=0x3333;
   inputReg[8]=0x0044;
   inputReg[9]=0x5555;
   inputReg[10]=0x0066;
   inputReg[11]=0x7777;
   inputReg[12]=0x8888;
   inputReg[13]=0x9999;
   inputReg[14]=0xAAAA;
   inputReg[15]=0xBBBB;
   
   MAX6675_init();
   
    while (1)
    {
        // Add your application code
        if(miliSaniye() >= (once_led_zamani + 500UL))
        {
            once_led_zamani+=500UL;
            pinLed_Test_Toggle();
            sprintf(sprint_temp,"%lu \r\n",once_led_zamani);
            usart2_yaz(sprint_temp);            

            holdingReg[0] = MAX6675_oku();
            holdingReg[1]= holdingReg[0] >>2 ; 

            sprintf(sprint_temp, "Sýcaklýk : %0.2f\r\n", (float) holdingReg[0]*0.25F);
            usart2_yaz(sprint_temp);
                    
        }

        //modbus_gelenFrame_Yazdir();
        modbus_process();

    }
}
/**
 End of File
*/