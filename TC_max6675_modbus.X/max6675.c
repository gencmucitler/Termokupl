#include "mcc_generated_files/mcc.h"
#include "max6675.h"


void MAX6675_init()
{
    pinMax6675_CS_TRIS=0;   //Çýkýþ
    pinMax6675_SCK_TRIS=0;  //Çýkýþ
    pinMax6675_SDI_TRIS=1;  //Giriþ
    
    pinMax6675_CS=1;
    pinMax6675_SCK=0;
}

//*** okunan deðer signed bir karakter aslýnda deðiþken tipide signed yapýlmalý...
uint16_t MAX6675_oku()
{
    unsigned int gelen_veri;
    
    pinMax6675_CS=0;    //chip seç.
    
    __delay_us(1);
    

    for(unsigned char i=0;i<16;i++)
    {
        pinMax6675_SCK=1;
        __delay_us(1);

        
        if(pinMax6675_SDI==1)
        {
            gelen_veri=(gelen_veri <<1) | 1;
        }
        else
        {
            gelen_veri=(gelen_veri <<1);
        }

        pinMax6675_SCK=0;
        __delay_us(1);

        
    }
    
    __delay_us(1);
    
    pinMax6675_CS=1;
    
    
    //return (((gelen_veri)>>3) *0.25F);
    return ((gelen_veri)>>3);
}