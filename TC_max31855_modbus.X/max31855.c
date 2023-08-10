#include "mcc_generated_files/mcc.h"
#include "max31855.h"


void MAX31855_init()
{
    pinMax6675_CS_TRIS=0;   //Çýkýþ
    pinMax6675_SCK_TRIS=0;  //Çýkýþ
    pinMax6675_SDI_TRIS=1;  //Giriþ
    
    pinMax31855_CS=1;
    pinMax31855_SCK=0;
}

sicaklik_t MAX31855_oku()
{
    sicaklik_t _sicaklik;
    uint32_t gelen_veri;
    
    pinMax31855_CS=0;    //chip seç.    
    __delay_us(1);
     pinMax31855_CS=1;        
    __delay_ms(150);    //dönüþüm süresi kadar bekle.. *** bu kod pek verimli olmadý..

    pinMax31855_CS=0;    //chip seç.    
    __delay_us(1);
    
    for(unsigned char i=0;i<32;i++)
    {
        pinMax31855_SCK=1;
        __delay_us(1);

        
        if(pinMax31855_SDI==1)
        {
            gelen_veri=(gelen_veri <<1) | 1;
        }
        else
        {
            gelen_veri=(gelen_veri <<1);
        }

        pinMax31855_SCK=0;
        __delay_us(1);

        
    }
    
    __delay_us(1);
    
    pinMax31855_CS=1;
    
    // sýcaklýk deðeri negatif mi?
//    if(gelen_veri & 0x80000000)
//    {
//        t_sicaklik = 0xFFFFC000 | ((t_sicaklik >> 18) & 0x00003FFF);
//    }
//    else
//    {
//            t_sicaklik =(uint16_t)(gelen_veri>>18);
//    }
    
    _sicaklik.sicaklik = (int16_t)(0xFFFF & (gelen_veri >> 18));
    _sicaklik.ic_sicaklik = (int16_t)(0x0FFF & (gelen_veri >>4));
    
    //hata bitlerini kontrol et.
    if(gelen_veri & 0x00000001)
    {
        _sicaklik.hata_oc=1;
    }
    else
    {
        _sicaklik.hata_oc=0;
    }
    
      if(gelen_veri &  0x00000002)
    {
        _sicaklik.hata_scg=1;
    }
    else
    {
        _sicaklik.hata_scg=0;
    }
    
    if(gelen_veri &  0x00000004)
    {
        _sicaklik.hata_scv=1;
    }
    else
    {
        _sicaklik.hata_scv=0;
    }
    
    if(gelen_veri &  0x00010000)
    {
        _sicaklik.hata=1;
    }
    else
    {
        _sicaklik.hata=0;
    }
        
    return (_sicaklik);
    
     
}