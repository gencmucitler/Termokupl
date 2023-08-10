/*******************************************************************************
 * Kütüphane     : ModbusRTU Slave Kütüphanesi                                 *                 *
 * Yazar         : sigmoid                                                     *
 * Baþlangýç     : 1 Aðustos 2023                                              *
 * Versiyon      : 0.1                                                         *
 ******************************************************************************/


#include <stdint.h>
#include "modbus.h"
#include "mcc_generated_files/mcc.h"
#include "zaman.h"


//https://github.com/whchoi/Modbus-Slave-for-PIC18F

/* Sistemin çalýþma mantýðý.
 * Modbus framelari arasý en az 3.5 karakter boþluk olmalý. Timer3 ile 3.5 karakter 
 * süresince veri gelmediyse yeni frame geldi demektir. 3,5+ilk karakter þeklinde timer3 kesme süresi ayarlanmalý
 * 
 * Usart kesmesinde gelen veriler önce ring buffer'a kaydedilir. Ayrýca ringbuffer dizisindeki kacýncý elemanýnýn modbus frameýn baþlangýcý
 * olduðuda farklý bir diziye kaydedilir. Þuan tüm gelen modbus verisini buffera kaydediyor. ****Ýlerleyen dönemde sadece kendine gelen veriyi
 * kaydedecek þekilde kod deðiþtirilecek.
 * 
 * Modbus progress ile ring bufferda yeni frame varsa iþlemeye baþlar. Ýþlemleri daha kolay yapabilmek için ringbufferdan mb_frame deðiþkenine kopyalama 
 * yapýlýr. Gerisi Modbus spesifikasyonunda yazdýðý gibi iþlemler yapýlýr.
 * 
 */


/*
Eusart1.c içine
 * 
 * #include "../modbus.h"
 * 
 * void EUSART1_RxDataHandler(void) fonksiyonu içine aþaðýdaki kod yazýlmalý..
 *     // modbus için gerekli kod.
    uint8_t temp_rx;        
    temp_rx=RCREG1;        //gelen veriyi gecici belleðe al.    
    gelen_veri_push(temp_rx);  //modbus ring buffer a gönder
 */

//modbus için EUSART1 modülünü kullanýyor..

extern volatile uint16_t ModbusTMRReloadVal;

uint8_t modbus_tamam=0;

volatile uint8_t frame_timeout=0;        //modbus frameleri arasý 3,5 karakterden uzun mu?
volatile uint8_t gelen_veri[GELEN_BUFFER_SIZE];    //usarttan gelen veri ilk önce buraya kaydedilir.
volatile uint8_t frame_baslangic[FRAME_ADRES_SIZE];    //hangi gelen_veri frame ilk adresidir, bu bilgiyi tut
volatile uint8_t gelenHead;      //gelenVeri ringBuffer için
volatile uint8_t gelenTail;  
volatile uint8_t gelen_veri_sayisi;
volatile uint8_t frame_basHead;  //frame_baslangic ringBuffer için
volatile uint8_t frame_basTail;
volatile uint8_t frame_sayisi;

volatile uint8_t mb_frame[MB_FRAME_UZUNLUGU];  //önce bufferden veri bu belleðe transfer oluyor.
volatile uint8_t mb_frame_boyutu; //frame uzunluðunu tutar.
volatile uint8_t mb_cihaz_address; // Cihaz adresi
volatile uint8_t mb_function;// Ýþlev kodu
volatile uint8_t mb_cevap[MB_FRAME_UZUNLUGU];   //cevap dizisi

volatile bool crc_kontroltamam,crc_tamam;

volatile uint8_t inputDiscreteReg[INPUT_DISCRETE_REG_SIZE];      //giriþ bitlerinin deðerlerini saklar.  Read Only 1 bit
volatile uint8_t coilReg[COIL_REG_SIZE];         //Çýkýþ bobinlerinin deðerlerini tutar. Read/Write 1 bit
volatile uint16_t inputReg[INPUT_REG_SIZE];      //Giriþ deðeri  16bit ReadOnly
volatile uint16_t holdingReg[HOLDING_REG_SIZE];  //16bitlik hafýza alanlarý Read-Write

#ifdef MB_DEBUG
char sprint_temp[64];
#endif

//------------------------------------------------------------------------------
//Gelen veri ring buffer fonksiyonlarý
//------------------------------------------------------------------------------
// https://embedjournal.com/implementing-circular-buffer-embedded-c/

//**** Buffer dolarsa ne olacak ??
uint8_t gelen_veri_pop()
{
    uint8_t next;
    uint8_t temp;
    
    if(gelenHead == gelenTail)
    {
        return 255; //buffer boþ
    }
    
    next = gelenTail+1;
    if(next >= GELEN_BUFFER_SIZE)
        next = 0;
    
    temp = gelen_veri[gelenTail];
    gelenTail = next;
    gelen_veri_sayisi--;
    
    return temp;
}

//------------------------------------------------------------------------------

void gelen_veri_push(uint8_t veri)
{
     //modbus için ring buffer.
    uint8_t next;
    next = gelenHead+1;
    
    if (next >= GELEN_BUFFER_SIZE)  //ring sonuna geldiyse baþa dön.
        next=0;
    
    if(next == gelenTail)
    {
        //buffer doldu.. önceki verinin üzerine yazmakla ilgili fonksiyon yazýlacak.
        //todo: gelen veri doldu. çözüm üret.
    }
    
    //usarttan gelen veriyi buffera yaz.
    gelen_veri[gelenHead]=veri;
    gelen_veri_sayisi++;
    
    //Eðer önceki byte ile 3.5 karakter kadar zaman geçtiyse yeni paket baþlýðý yap.
    if(frame_timeout)
    {     
        frame_baslangic_push(gelenHead);
    }
    
    ModbusTMR_Clear();
    gelenHead=next;
}

//------------------------------------------------------------------------------

//frame baþlangýç adresini çeker
uint8_t frame_baslangic_pop()
{
    uint8_t next;
    uint8_t temp;
    
    if(frame_basHead == frame_basTail)
    {
        return 255;  //frame adresi yok, ringbuffer boþ..
    }
    
    next = frame_basTail+1;
    
    if(next >= FRAME_ADRES_SIZE)
        next=0;
    
    temp = frame_baslangic[frame_basTail];
    frame_basTail = next;
    
    frame_sayisi--;
    
    return temp;
}

//------------------------------------------------------------------------------

void frame_baslangic_push(uint8_t adres)
{
        uint8_t next;
        
        frame_timeout=0;
        
        next=frame_basHead+1;
 
        if(next >= FRAME_ADRES_SIZE)
            next=0;
  
        if(frame_basHead == frame_basTail)
        {
            //todo:frame buffer doldu, çözüm üret..
            
        }
        
        frame_baslangic[frame_basHead]=adres;
        frame_sayisi++;
               
        frame_basHead=next;                 
  
}

//------------------------------------------------------------------------------
// USART hýz deðiþiklik fonk. ve Timer Süre deðiþtirme Fonk.
//------------------------------------------------------------------------------
/*
 9600
 19200 
 57600
 115200 
 */
void modbus_hizi_sec(uint8_t hiz)
{
    switch(hiz)
{
        case 0: //9600
            //TMR3H 27; 
            TMR3H = 0x1B;
            //TMR3L 224; 
            TMR3L = 0xE0;
            // Load the TMR value to reload variable
            ModbusTMRReloadVal = (uint16_t) ((TMR3H << 8) | TMR3L);    //3,65ms timeout
            break;
            
        
        case 1: //19200
                //TMR3H 142; 
    TMR3H = 0x8E;

    //TMR3L 64; 
    TMR3L = 0x40;   //1,82ms
            break;
        
        case 2:     //57600
                //TMR3H 218; 
    TMR3H = 0xDA;

    //TMR3L 16; 
    TMR3L = 0x10;   //607us
            break;
        case 3:     //115200
                //TMR3H 237; 
    TMR3H = 0xED;

    //TMR3L 16; 
    TMR3L = 0x10;   //303us
            break;
    }
}

//------------------------------------------------------------------------------
// Modbus fonksiyonlarý
//------------------------------------------------------------------------------

 uint8_t k_crcHigh = 0;
 uint8_t k_crcLow = 0;
    
 //Gelen verideki crc kontrolü için kullanýlýyor..
void crc_kontrol(void)
{
    uint16_t crc = 0xFFFF;

    uint8_t i, j = 0,dongu_sonu;
    dongu_sonu = mb_frame_boyutu - 2;
    
    for (i = 0; i < dongu_sonu ; ++i) {
        crc ^= mb_frame[i];
        for (j = 8; j != 0; --j) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    /* Switch endianess */
    k_crcHigh = (uint8_t)(crc & 0xFF);
    k_crcLow = (uint8_t)((crc & 0xFF00) >> 8);

    if ((k_crcHigh == mb_frame[i]) && (k_crcLow == mb_frame[i + 1]))
        crc_kontroltamam= true;
    else
        crc_kontroltamam= false;
    
    crc_tamam=true;    
}

//------------------------------------------------------------------------------

 uint8_t crcHigh = 0;
 uint8_t crcLow = 0;

void crc_hesapla(uint8_t messageLength)
{
    uint16_t crc = 0xFFFF;
    int i, j = 0;

    for (i = 0; i < messageLength ; ++i) {
        crc ^= mb_cevap[i];
        for (j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    /* Switch endianess */
    crcHigh = (uint8_t)(crc & 0xFF);// << 8;
    crcLow = (uint8_t)((crc & 0xFF00) >> 8);
}

//------------------------------------------------------------------------------

/* Modbus frameleri arasý 3.5 byte'týr. Bu fonksiyon bize frame arasý boþluðu ölçmek için timer tarafýndan çaðýran kesmedir.
 *
 */
void modbus_frame_timeout()
{
    frame_timeout=1;
}

//------------------------------------------------------------------------------
/* 
 *  Modbus baþlangýç için gerekli fonksiyon
 * 
 */
void modbus_init()
{
    modbus_data_clear();
    ModbusTMR_Start();
    ModbusTMR_SetInterruptHandler(modbus_frame_timeout);     //3.5 karakterde bir kesme çalýþýyor.     
    frame_timeout=1;
    
    modbus_gelen_aktif();
}

//------------------------------------------------------------------------------

// gelen veri belleðini sil.
void modbus_data_clear()
{
    uint8_t i;
    
    gelenHead=0;
    gelenTail=0;
    gelen_veri_sayisi=0;
    
    frame_sayisi=0;
    frame_basHead=0;
    frame_basTail=0;
            
    for(i=0; i<GELEN_BUFFER_SIZE;i++)
    {
        gelen_veri[i]=0;
    }
    
    for(i=0; i<FRAME_ADRES_SIZE;i++)
    {
        frame_baslangic[i]=0;
    }        
    
    modbus_cevap_clear();
}


//------------------------------------------------------------------------------

void modbus_cevap_clear()
{
    unsigned char i;
    for(i=0;i<MB_FRAME_UZUNLUGU;i++){ //mb_cevap  uzunluðu
      mb_cevap[i] = 0;
    }
}
//------------------------------------------------------------------------------

void ModbusTMR_Clear()
{
    volatile uint16_t timer3ReloadValue;
    
    //TMR3H 27; 
    TMR3H = 0x1B;

    //TMR3L 224; 
    TMR3L = 0xE0;
    // Load the TMR value to reload variable
    timer3ReloadValue=(uint16_t)((TMR3H << 8) | TMR3L);
    
    PIR4bits.TMR3IF = 0;
    TMR3_WriteTimer(timer3ReloadValue);   //9600 bps hýz için, diðer deðerler yeniden yazýlmalý...
        
    frame_timeout=0;      
}

//------------------------------------------------------------------------------

void modbus_gelen_aktif()
{
    //Modbus gelen veri aktif..
  //  RS485_DE_SetLow();
  //  RS485_RE_SetLow();      
}

//------------------------------------------------------------------------------

void modbus_giden_aktif()
{
    //Modbus giden veri aktif
  //  RS485_RE_SetHigh();      //giden veri aktif.
  //  RS485_DE_SetHigh();    
}

//------------------------------------------------------------------------------

unsigned char gelen_adet=0,paket_uzunlugu;
unsigned long modbus_gelen_zamani=0;
//_Bool gelen_tamam=false;
//_Bool crc_tamam=false;    

//Gelen Frame diðer usarttan basar
//Test amaçlý bir fonksiyon.
void modbus_gelenFrame_Yazdir()
{
    uint8_t frame_basi,frame_sonu,frame_boyutu;
    
    //Gelen frame var mý?    
    if((frame_sayisi==1 && frame_timeout==1) || (frame_sayisi>=2))
    {
        //Frame baþýný bul..
        frame_basi =frame_baslangic_pop();   //frame baþýný bul..
                
        //Frame sonunu bul.
        //Eðer baþka frame yok mevcut veri frame sonudur.
        if(frame_sayisi == 0)
        {
            if(gelenHead == 0)
                frame_sonu = GELEN_BUFFER_SIZE-1;
            else
                frame_sonu = gelenHead-1;
        }
        else if(frame_sayisi>=1)
        {
            //bla bla..
            //eðer baþka frame varsa bir önceki frameden öncesi frame sonudur.
                    
            frame_sonu=frame_baslangic[frame_basTail]-1;
        }
        
        //frame boyutu hesapla
        if(frame_sonu >= frame_basi)
        {
            frame_boyutu=frame_sonu-frame_basi+1;
        }
        else
        {
            frame_boyutu=(GELEN_BUFFER_SIZE-frame_basi)+frame_sonu+1;
        }
                    
        //framei usart2 den yazdýr..
        //gelenTail=frame_basi;
        gelenTail=frame_basi;      
        
        for(uint8_t i=0;i<frame_boyutu;i++)
        {
            uint8_t veri;
                  
            veri=gelen_veri_pop();
                
            //EUSART2_Write(veri);
            hex_yazdir(veri);
      //      gelen veri pop
//            usart2 yazdýr..
        }
        
        EUSART2_Write('\r');
        EUSART2_Write('\n');
    }
    
}

//------------------------------------------------------------------------------

//Bellekte gelen veri varsa, iþle.
//önce gelen_veri ringbuffera, sonra  mb_frame aktarýr.
//sonra crc kontrol den geçenleri iþler.,

void modbus_process() {
    uint8_t frame_basi, frame_sonu, frame_boyutu;
    uint8_t i;

    //Gelen frame var mý?    
    if ((frame_sayisi == 1 && frame_timeout == 1) || (frame_sayisi >= 2)) {
        //Frame baþýný bul..
        frame_basi = frame_baslangic_pop(); //frame baþýný bul..

        //Frame sonunu bul.
        //Eðer baþka frame yok mevcut veri frame sonudur.
        if (frame_sayisi == 0) {
            if (gelenHead == 0)
                frame_sonu = GELEN_BUFFER_SIZE - 1;
            else
                frame_sonu = gelenHead - 1;
        } else if (frame_sayisi >= 1) {
            //bla bla..
            //eðer baþka frame varsa bir önceki frameden öncesi frame sonudur.

            frame_sonu = frame_baslangic[frame_basTail] - 1;
        }

        //frame boyutu hesapla
        if (frame_sonu >= frame_basi) {
            frame_boyutu = frame_sonu - frame_basi + 1;
        } else {
            frame_boyutu = (GELEN_BUFFER_SIZE - frame_basi) + frame_sonu + 1;
        }

        //framei usart2 den yazdýr..
        //gelenTail=frame_basi;
        gelenTail = frame_basi;

        
#ifdef MB_DEBUG
        sprintf(sprint_temp,"Istek(RX): ");
        usart2_yaz(sprint_temp);
#endif        

        //frame modbus temp dizisine kopyala.
        for (i = 0; i < frame_boyutu; i++) {
            uint8_t veri;

            veri = gelen_veri_pop();
            mb_frame[i] = veri;
#ifdef MB_DEBUG
            hex_yazdir(veri);
#endif
        }
#ifdef MB_DEBUG
        EUSART2_Write('\r');
        EUSART2_Write('\n');        
#endif
        
        mb_frame_boyutu = frame_boyutu;
        mb_cihaz_address = mb_frame[0];

#ifdef MB_DEBUG
        sprintf(sprint_temp, "Cihaz Adresi: %u ", mb_cihaz_address);
        usart2_yaz(sprint_temp);

        if (mb_frame[0] == MODBUS_ADRES) 
        {
            sprintf(sprint_temp, "OK\r\n");
            usart2_yaz(sprint_temp);
        } else {
            sprintf(sprint_temp, "Gecersiz.\r\n");
            usart2_yaz(sprint_temp);
        }
#endif


        //Gelen Frame bize mi ait? En az 4 byte mý
        if (mb_frame[0] == MODBUS_ADRES && mb_frame_boyutu > 4) {            
            mb_function = mb_frame[1];
            
#ifdef MB_DEBUG
            sprintf(sprint_temp,"Fonksiyon:");
            usart2_yaz(sprint_temp);
            switch(mb_function)
            {
                case 1:
                    sprintf(sprint_temp,"(1) Read Coils\r\n");
                    usart2_yaz(sprint_temp);                    
                    break;
                case 2:
                    sprintf(sprint_temp,"(2) Read Discrete Inputs\r\n");
                    usart2_yaz(sprint_temp);                    
                    break;
                case 3:
                    sprintf(sprint_temp,"(3) Read Holding Register\r\n");
                    usart2_yaz(sprint_temp);
                    break;
                case 4:
                    sprintf(sprint_temp,"(4) Read Input Register\r\n");
                    usart2_yaz(sprint_temp);                    
                    break;
                case 5:
                    sprintf(sprint_temp,"(5) Write Single Coil\r\n");
                    usart2_yaz(sprint_temp);                    
                    break;
                case 6:
                    sprintf(sprint_temp,"(6) Write Single Register\r\n");
                    usart2_yaz(sprint_temp);
                    break;                    
                case 15:
                    sprintf(sprint_temp,"(15) Write Multiple Coils\r\n");
                    usart2_yaz(sprint_temp);                    
                    break;
                case 16:
                    sprintf(sprint_temp,"(16) Write Multiple Register\r\n");
                    usart2_yaz(sprint_temp);
                    break;                    
                default:
                    sprintf(sprint_temp,"(%u) Diger Fonk.\r\n",mb_function);
                    usart2_yaz(sprint_temp);
                    break;                    
            }            
#endif

            //CRC_Kontrol Et, yanlýþsa iþleme devam etme çýk..
            crc_kontrol();

#ifdef MB_DEBUG

            sprintf(sprint_temp,"CRC : ");
            usart2_yaz(sprint_temp);                  
            hex_yazdir(k_crcHigh);
            hex_yazdir(k_crcLow);
            EUSART2_Write(' ');

            if(crc_kontroltamam)
            {
                    sprintf(sprint_temp,"CRC gecti.\r\n");
                    usart2_yaz(sprint_temp);                
            }
            else
            {
                    sprintf(sprint_temp,"CRC gecersiz.\r\n");
                    usart2_yaz(sprint_temp);                
            }            
#endif
            

            //CRC kontrolü tamamsa diðer iþlemleri yap. modbus iþlemlerini yap..
            if (crc_kontroltamam) {
                switch (mb_function) {
                    case 1:
                        mb_read_coils();
                        break;
                    case 2:
                        mb_read_discrete_inputs();
                        break;
                    case 3: //Read Holding Register
                        mb_read_holding_registers();
                        break;
                    case 4:
                        mb_read_input_registers();
                        break;
                    case 5:
                        mb_write_single_coil();
                        break;
                    case 6: //Write Holding Register
                        mb_write_single_register();
                        break;
                    case 15:
                        mb_write_multiple_coils();
                        break;
                    case 16: //Write Multiple Register 
                        mb_write_multiple_registers();
                        break;
                    default:
                        //böyle bir komut yoktur cevabý
                        //*** yanlýþ çalýþýyor. gerekli düzeltme yapýlacak.
                        mb_cevap[0]=MODBUS_ADRES;
                        mb_cevap[1]=0x80 + mb_function;
                        mb_cevap[2]=0x01;   //desteklenmeyen komut
                        crc_hesapla(3);
                        mb_cevap[3]=crcHigh;
                        mb_cevap[4]=crcLow;

#ifdef MB_DEBUG
                        sprintf(sprint_temp, "Desteklenmeyen Function Code: %u ", mb_function);
                        usart2_yaz(sprint_temp);
#endif       

                        //cevabý gönder..
                        for (i = 0; i< 5; i++) {
                            EUSART1_Write(mb_cevap[i]);
#ifdef MB_DEBUG
                            hex_yazdir(mb_cevap[i]);
#endif
                        }

#ifdef MB_DEBUG
                        EUSART2_Write('\r');
                        EUSART2_Write('\n');
#endif       
                          
                        break;
                }
            }
        }

    }


}

//------------------------------------------------------------------------------

//gelen veriyi eusart2 den hex olarak yazdýrýr.
void hex_yazdir(uint8_t veri)
{
    uint8_t ust_veri, alt_veri;
    
    ust_veri  = (veri >> 4) & 0x0F;
    alt_veri = veri & 0x0F;
    
    if(ust_veri<=9)
    {
        ust_veri= ust_veri + '0';
    }
    else if(ust_veri >= 10 && ust_veri <=15 )
    {
        ust_veri = ust_veri - 10 +'A';
    }
    
    if(alt_veri<=9)
    {
        alt_veri= alt_veri + '0';
    }
    else if(alt_veri >= 10 && alt_veri <=15 )
    {
        alt_veri = alt_veri - 10 +'A';
    }
    
    EUSART2_Write(ust_veri);
    EUSART2_Write(alt_veri);
    EUSART2_Write(' ');
}

//------------------------------------------------------------------------------
//Mevcut Coil durumunu ekrana yazar.
void coil_durum_goster()
{
    uint8_t coilByteAdet;
    uint8_t coilByteKalan;
    uint8_t i,j,k;
    uint8_t onlar,birler,sayi;
    
    k=COIL_REG_SIZE-1;
    coilByteAdet = COIL_REG_SIZE/8;
    coilByteKalan = COIL_REG_SIZE % 8;
    
    sprintf(sprint_temp,"Coil Durumlarý: \r\n");
    usart2_yaz(sprint_temp);
    sprintf(sprint_temp,"------------------------------------------------\r\n");
    usart2_yaz(sprint_temp);
    
    if(coilByteKalan)
    {
        for(i=k; i<coilByteKalan;i++)
        {
            sayi=k--;
            onlar=0;
            birler=0;
            while(sayi>=10)
            {
                sayi-=10;
                onlar++;
            }
            birler=sayi;
            
            EUSART2_Write('0'+onlar);
            EUSART2_Write('0'+birler);
            EUSART2_Write(' ');
        }
        
            EUSART2_Write(' ');

       
    }
    
    if(coilByteAdet)
    {
         for (i = 0; i < coilByteAdet; i++) {
            for (j = 0; j < 8; j++) {
                sayi = k--;
                onlar = 0;
                birler = 0;
                while (sayi >= 10) {
                    sayi -= 10;
                    onlar++;
                }
                birler = sayi;

                EUSART2_Write('0' + onlar);
                EUSART2_Write('0' + birler);
                EUSART2_Write(' ');
            }

            EUSART2_Write(' ');
        }
    }
    
    EUSART2_Write('\r');
    EUSART2_Write('\n');
    
    
    //Coillerin durumunu yaz..
    // ON  of
    
     k=COIL_REG_SIZE-1;
     
     if(coilByteKalan)
    {
        for(i=k; i<coilByteKalan;i++)
        {
            if (coilReg[k]) {
                EUSART2_Write('O' );
                EUSART2_Write('N' );
                EUSART2_Write(' ');
            } else {
                EUSART2_Write('o' );
                EUSART2_Write('f' );
                EUSART2_Write(' ');
            }
            k--;
            

        }
        
            EUSART2_Write(' ');

       
    }

    if (coilByteAdet) {
        for (i = 0; i < coilByteAdet; i++) {
            for (j = 0; j < 8; j++) {
                if (coilReg[k]) {
                    EUSART2_Write('O');
                    EUSART2_Write('N');
                    EUSART2_Write(' ');
                } else {
                    EUSART2_Write('o');
                    EUSART2_Write('f');
                    EUSART2_Write(' ');
                }
                k--;
            }

            EUSART2_Write(' ');
        }
    }
    
        EUSART2_Write('\r');
    EUSART2_Write('\n');
    
}

//------------------------------------------------------------------------------

void usart2_yaz(char * t)
{
    while(*t)
    {
        EUSART2_Write(*t++);
    }
}

/******************************************************************************/
// Modbus  function  deðiþken ve yardýmcý fonksiyonlarý

//Deðiþkenler
volatile uint16_t mb_startAddr; // Register Baþlangýç adresi
volatile uint16_t mb_quantityReg; // Okunacak veya yazýlacak veri noktasý sayýsý

  uint8_t howManyBytes = 0;
  uint8_t remainder = 0;
  uint8_t lsb = 0;
  uint8_t ii ,jj ,kk ,mm = 0;
  
  //fonksiyonlar
  //----------------------------------------------------------------------------
  void mb_startAddr_quantityReg_yukle()
  {
    mb_startAddr =(uint16_t) (mb_frame[2] << 8) | mb_frame[3];
    mb_quantityReg =(uint16_t) (mb_frame[4] << 8) | mb_frame[5];
  
//  //Combine address bytes
//  mb_startAddr = received[2];
//  mb_startAddr <<=8;
//  mb_startAddr |= received[3];
//
//  //Combine number of coils bytes
//  mb_quantityReg = received[4];
//  mb_quantityReg <<= 8;
//  mb_quantityReg |= received[5];
  }
  
//------------------------------------------------------------------------------
  void mb_cevap_gonder()
  {      
    modbus_giden_aktif();

#ifdef MB_DEBUG
    sprintf(sprint_temp, "Cevap(TX): ");
    usart2_yaz(sprint_temp);
#endif       

    //cevabý gönder..
    for (ii = 0; ii != (kk + 2); ii++) {
        EUSART1_Write(mb_cevap[ii]);
#ifdef MB_DEBUG
        hex_yazdir(mb_cevap[ii]);
#endif
    }

#ifdef MB_DEBUG
    EUSART2_Write('\r');
    EUSART2_Write('\n');
#endif       

    modbus_gelen_aktif();

    modbus_cevap_clear();
  }
  
//------------------------------------------------------------------------------
// Function 0x01
  //Test edildi güzel çalýþýyor...
void mb_read_coils()
{
    mb_startAddr = 0;
    mb_quantityReg = 0;
    howManyBytes = 0;
    remainder = 0;
    lsb = 0;
    mm = 0;

    mb_startAddr_quantityReg_yukle();

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x01; //read coil function

    //Gerekli kontrolleri yap.
    //Ýstenen Coil adeti geçerli mi 1 ile 2000 arasýnda mý?
    if (mb_quantityReg >= 1 && mb_quantityReg <= 2000) {

        //Bu adres bende var mý?
        if (mb_startAddr >= COIL_REG_START_ADR && (mb_startAddr + mb_quantityReg) <= (COIL_REG_START_ADR + COIL_REG_SIZE)) {

            howManyBytes = (uint8_t)(mb_quantityReg / 8);
            remainder = mb_quantityReg % 8;

            if (remainder) {
                howManyBytes += 1;
            }

            mb_cevap[2] = howManyBytes; //gönderilecek byte sayýsýný söyle.

            mm =(uint8_t) (mb_startAddr - COIL_REG_START_ADR);
            kk = 3; //start at mb_cevap 3

            //coilleri oku ve deðiþkene yaz.
            //Coil deðeri 1 byte saklanýyor. aslýnda 1 bitlik deðer tutuyor. 
            //Bu deðer byte->bit þeklinde aktarým yapýlýyor.
            for (ii = howManyBytes; ii != 0; ii--) {
                if (ii > 1 || (ii == 1 && remainder == 0)) {
                    for (jj = 0; jj != 8; jj++) {
                        if (coilReg[mm]) {
                            lsb = 1;
                        } else {
                            lsb = 0;
                        }
                        mb_cevap[kk] ^= (lsb << jj);
                        mm++;
                    }
                    kk++;
                } else {
                    for (jj = 0; jj != remainder; jj++) {
                        if (coilReg[mm]) {
                            lsb = 1;
                        } else {
                            lsb = 0;
                        }
                        mb_cevap[kk] ^= (lsb << jj);
                        mm++;
                    }
                    kk++;
                }
            }

            crc_hesapla(kk); //þu ana kadar girilen deðerlerin crc sini hesapla..  k+2 olmayacak..

            mb_cevap[kk] = crcHigh;
            mb_cevap[kk + 1] = crcLow;

            //  crc = generateCRC(k+2);     
            //
            //  mb_cevap[k] = crc >> 8;
            //  mb_cevap[k+1] = crc;
        } else { //Bu adres bende yok..
            mb_cevap[1] = 0x81; //exception code
            mb_cevap[2] = 0x02; //Illegal Data adress
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }

    } else { //quantity of Coil geçersiz 1 ile 2000 arasýnda deðil..
        mb_cevap[1] = 0x81; //exception code
        mb_cevap[2] = 0x03;
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }

    mb_cevap_gonder();

    //**** Coil_Reg deki deðiþiklikleri Çýkýþlara yansýtacak kodlar yazýlmalý...
}

//------------------------------------------------------------------------------
// Function 0x02
//Test edildi. Çalýþýyor..
void mb_read_discrete_inputs() {
    /******************************************************************************/
    /* Reads a coil and then responds                                             */
    /******************************************************************************/
    mb_startAddr = 0;
    mb_quantityReg = 0;

    howManyBytes = 0;
    remainder = 0;
    lsb = 0;
    mm = 0;

    //**** Inputlardaki deðiþiklikleri inputDiscreteReg e yansýtacak kodlar yazýlmalý...

    mb_startAddr_quantityReg_yukle();

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x02;

    //Gerekli kontrolleri yap.
    //Ýstenen discrete input adeti geçerli mi 1 ile 2000 arasýnda mý?
    if (mb_quantityReg >= 1 && mb_quantityReg <= 2000) {

        //Bu adres bende var mý?
        if (mb_startAddr >= INPUT_DISCRETE_START_ADR && (mb_startAddr + mb_quantityReg) <= (INPUT_DISCRETE_START_ADR + INPUT_DISCRETE_REG_SIZE)) {

            howManyBytes =(uint8_t) (mb_quantityReg / 8);
            remainder = mb_quantityReg % 8;

            if (remainder) {
                howManyBytes += 1;
            }

            mb_cevap[2] = howManyBytes;

            mm = (uint8_t) (mb_startAddr - INPUT_DISCRETE_START_ADR);
            kk = 3; //start at mb_cevap 3

            for (ii = howManyBytes; ii != 0; ii--) {
                if (ii > 1 || (ii == 1 && remainder == 0)) {
                    for (jj = 0; jj != 8; jj++) {
                        if (inputDiscreteReg[mm]) {
                            lsb = 1;
                        } else {
                            lsb = 0;
                        }
                        mb_cevap[kk] ^= (lsb << jj);
                        mm++;
                    }
                    kk++;
                } else {
                    for (jj = 0; jj != remainder; jj++) {
                        if (inputDiscreteReg[mm]) {
                            lsb = 1;
                        } else {
                            lsb = 0;
                        }
                        mb_cevap[kk] ^= (lsb << jj);
                        mm++;
                    }
                    kk++;
                }
            }

            crc_hesapla(kk);

            mb_cevap[kk] = crcHigh;
            mb_cevap[kk + 1] = crcLow;

        } else { //Bu adres bende yok..
            mb_cevap[1] = 0x82; //exception code
            mb_cevap[2] = 0x02; //Illegal Data adress
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }

    } else { //quantity of Coil geçersiz 1 ile 2000 arasýnda deðil..
        mb_cevap[1] = 0x82; //exception code
        mb_cevap[2] = 0x03;
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;

    }

    mb_cevap_gonder();
}

//------------------------------------------------------------------------------
// Function 0x03
//Test edildi. çalýþýyor.
void mb_read_holding_registers()
{
  uint8_t byteCount;
  mb_startAddr = 0;
  mb_quantityReg = 0;
  jj = 3;
  ii = 0;

  mb_startAddr_quantityReg_yukle();          

  mb_cevap[0] = MODBUS_ADRES;
  mb_cevap[1] = 0x03;
  
  byteCount=(uint8_t)(mb_quantityReg *2);

  
  //Cevap dizisi tanýmlý diziden daha büyükse hata gönder.
  if(byteCount+5 > MB_FRAME_UZUNLUGU )
  {
        mb_cevap[1] = 0x83; //exception code
        mb_cevap[2] = 0x04; //Devie Failure ,, Bu kadar uzun cevap gönderemiyor, dizi boyutu yetmiyor..
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
  }
  else
  {
  
  //istenilen veri adetini göndermek mümkün mü
  if (mb_quantityReg >= 1 && mb_quantityReg <= 125) {
        
      //Bu adres bende var mý?
        if (mb_startAddr >= HOLDING_REG_START_ADR && (mb_startAddr + mb_quantityReg) <= (HOLDING_REG_START_ADR + HOLDING_REG_SIZE)) {
  
            mb_cevap[2]=byteCount;   //byteCount verisini gönder..
            
            for (ii = (uint8_t)(mb_startAddr - HOLDING_REG_START_ADR); ii < (mb_startAddr - HOLDING_REG_START_ADR + mb_quantityReg); ii++) {
                mb_cevap[jj]= (uint8_t)((holdingReg[ii]>>8) & 0x00FF);
                jj++;
                mb_cevap[jj]=(uint8_t)(holdingReg[ii] & 0x00FF);
                jj++;
//                if (holdingReg[ii] > 255) {
//                    //Need to split it up into 2 bytes
//                    mb_cevap[jj] = holdingReg[ii] >> 8;
//                    jj++;
//                    mb_cevap[jj] = holdingReg[ii];
//                    jj++;
//                } else {
//                    mb_cevap[jj] = 0x00;
//                    jj++;
//                    mb_cevap[jj] = holdingReg[ii];
//                    jj++;
//                }
            }

            kk=jj;
            
            crc_hesapla(kk); //þu ana kadar girilen deðerlerin crc sini hesapla..  k+2 olmayacak..

            mb_cevap[kk] = crcHigh;
            mb_cevap[kk + 1] = crcLow;
            
        } else {
            mb_cevap[1] = 0x83; //exception code
            mb_cevap[2] = 0x02; //
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }
    } else {
        mb_cevap[1] = 0x83; //exception code
        mb_cevap[2] = 0x03; //
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }
  }
  
    mb_cevap_gonder();
}

//------------------------------------------------------------------------------
//Function 0x04
//Test edildi çalýþýyor.
void mb_read_input_registers() {
    uint8_t byteCount;
    mb_startAddr = 0;
    mb_quantityReg = 0;
    jj = 3;
    ii = 0;

    mb_startAddr_quantityReg_yukle();

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x04;

    byteCount = (uint8_t) (mb_quantityReg * 2);


    //Cevap  dizisi tanýmlý diziden daha büyükse hata gönder.
    if (byteCount + 5 > MB_FRAME_UZUNLUGU) {
        mb_cevap[1] = 0x84; //exception code
        mb_cevap[2] = 0x04; //Devie Failure ,, Bu kadar uzun cevap gönderemiyor, dizi boyutu yetmiyor..
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    } else {
        //istenilen veri adetini göndermek mümkün mü
        if (mb_quantityReg >= 1 && mb_quantityReg <= 125) {

            //Bu adres bende var mý?
            if (mb_startAddr >= INPUT_REG_START_ADR && (mb_startAddr + mb_quantityReg) <= (INPUT_REG_START_ADR + INPUT_REG_SIZE)) {

                mb_cevap[2] = byteCount; //byteCount verisini gönder..

                //*** buradaki adres hesaplamayý tekrar gözden geçir, birþeyler eksik.. ama nerede anlamadým.
                for (ii = (uint8_t) (mb_startAddr - INPUT_REG_START_ADR); ii < (mb_startAddr - INPUT_REG_START_ADR + mb_quantityReg); ii++) {
                    mb_cevap[jj] = (uint8_t) ((inputReg[ii] >> 8) & 0x00FF);
                    jj++;
                    mb_cevap[jj] = (uint8_t) (inputReg[ii] & 0x00FF);
                    jj++;

                }

                kk = jj;

                crc_hesapla(kk); //þu ana kadar girilen deðerlerin crc sini hesapla..  k+2 olmayacak..

                mb_cevap[kk] = crcHigh;
                mb_cevap[kk + 1] = crcLow;

            } else {
                mb_cevap[1] = 0x84; //exception code
                mb_cevap[2] = 0x02; //
                crc_hesapla(3);
                mb_cevap[3] = crcHigh;
                mb_cevap[4] = crcLow;
                kk = 3;
            }
        } else {
            mb_cevap[1] = 0x84; //exception code
            mb_cevap[2] = 0x03; //
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }
    }

    mb_cevap_gonder();
}


//------------------------------------------------------------------------------
//Function: 0x05
//Test edildi çalýþýyor.
void mb_write_single_coil() {

    mb_startAddr_quantityReg_yukle();

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x05; //Write Coil Function

    //Bu adres bende var mý?
    if (mb_startAddr >= COIL_REG_START_ADR && mb_startAddr < (COIL_REG_START_ADR + COIL_REG_SIZE)) {

        //adres bilgisi
        mb_cevap[2] = mb_frame[2];
        mb_cevap[3] = mb_frame[3]; //2 bytes per reg

        //Yazýlan deðeri geri gönder//
        mb_cevap[4] = mb_frame[4];
        mb_cevap[5] = mb_frame[5];

        crc_hesapla(6);
        mb_cevap[6] = crcHigh;
        mb_cevap[7] = crcLow;
        kk = 6;

        if (mb_frame[4] == 0xFF && mb_frame[5] == 0x00) {
            //Coil On et.
            coilReg[mb_startAddr - COIL_REG_START_ADR] = 0xFF;

            //*** ilgili çýkýþý aktif edecek kod buraya yazýlmalý

        } else if (mb_frame[4] == 0x00 && mb_frame[5] == 0x00) {
            //Coil off yap
            coilReg[mb_startAddr - COIL_REG_START_ADR] = 0x00;

            //*** ilgili çýkýþý pasif edecek kod buraya yazýlmalý
        }
        else {
            //exception gönder..
            mb_cevap[1] = 0x85; //exception code
            mb_cevap[2] = 0x03; //Illegal Data adress
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }
    } else { //Bu adres bende yok..
        mb_cevap[1] = 0x85; //exception code
        mb_cevap[2] = 0x02; //Illegal Data adress
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }

    mb_cevap_gonder();
    
#ifdef MB_DEBUG
    coil_durum_goster();
#endif
}

//------------------------------------------------------------------------------
//Function: 0x06
//test edildi.
void mb_write_single_register() {

    ii = 0;

    mb_startAddr_quantityReg_yukle();

    //bu adres bende var mý?
    if (mb_startAddr >= HOLDING_REG_START_ADR && mb_startAddr < (HOLDING_REG_START_ADR + HOLDING_REG_SIZE)) {


        holdingReg[mb_startAddr - HOLDING_REG_START_ADR] = (uint16_t) ((mb_frame[4] << 8) | mb_frame[5]);

        mb_cevap[0] = MODBUS_ADRES;
        mb_cevap[1] = 0x06;
        mb_cevap[2] = mb_frame[2]; //adres deðeri
        mb_cevap[3] = mb_frame[3];

        mb_cevap[4] = mb_frame[4]; //yazýlan deðer.
        mb_cevap[5] = mb_frame[5];

        crc_hesapla(6);
        mb_cevap[6] = crcHigh;
        mb_cevap[7] = crcLow;

        kk = 6;

    } else {
        mb_cevap[1] = 0x86; //exception code
        mb_cevap[2] = 0x02; //
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }

    mb_cevap_gonder();
}

//------------------------------------------------------------------------------
//Function 0x0F
//test edildi çalýþýyor..
void mb_write_multiple_coils()
{
    uint8_t coilByte;
    mb_startAddr = 0;
    mb_quantityReg = 0;
    howManyBytes = 0;
    remainder = 0;
    lsb = 0;
    mm = 0;

    mb_startAddr_quantityReg_yukle();

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x0F; //read coil function    

    //Gerekli kontrolleri yap.
    //Ýstenen Coil adeti geçerli mi 1 ile 2000 arasýnda mý?
    if (mb_quantityReg >= 1 && mb_quantityReg <= 1968) {

        //Bu adres bende var mý?
        if (mb_startAddr >= COIL_REG_START_ADR && (mb_startAddr + mb_quantityReg) <= (COIL_REG_START_ADR + COIL_REG_SIZE)) {

            howManyBytes = (uint8_t) (mb_quantityReg / 8);
            remainder = mb_quantityReg % 8;

            if (remainder) {
                howManyBytes += 1;
            }

            mm = 0;

            //verileri yaz
            for (ii = 0; ii < howManyBytes; ii++) {
                coilByte = mb_frame[7 + ii];
                //***beynim yandýý!!!            

                for (jj = 0; jj != 8; jj++) {
                    if ((coilByte >> jj) & 0x01) {
                        coilReg[mb_startAddr] = 0xFF;
                    } else {
                        coilReg[mb_startAddr] = 0x00;
                    }
                    mb_startAddr++;
                    mm++;
                    if (mm == mb_quantityReg)
                        break;
                }

            }



            //modbus cevabý hazýrla.
            mb_cevap[2] = mb_frame[2]; //adres
            mb_cevap[3] = mb_frame[3];
            mb_cevap[4] = mb_frame[4]; //adet
            mb_cevap[5] = mb_frame[5];

            kk = 6;
            crc_hesapla(kk);
            mb_cevap[kk] = crcHigh;
            mb_cevap[kk + 1] = crcLow;

        } else { //Bu adres bende yok..
            mb_cevap[1] = 0x81; //exception code
            mb_cevap[2] = 0x02; //Illegal Data adress
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }

    } else { //quantity of Coil geçersiz 1 ile 2000 arasýnda deðil..
        mb_cevap[1] = 0x81; //exception code
        mb_cevap[2] = 0x03;
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }

    mb_cevap_gonder();

#ifdef MB_DEBUG
    coil_durum_goster();
#endif
}

//------------------------------------------------------------------------------
//Function 0x10
//test edildi çalýþyýor.
void mb_write_multiple_registers() {
    
    mb_startAddr = 0;
    mb_quantityReg = 0;
    jj = 7;
    ii = 0;

    uint8_t byteCount = 0;

    mb_startAddr_quantityReg_yukle();

    byteCount = mb_frame[6];

    mb_cevap[0] = MODBUS_ADRES;
    mb_cevap[1] = 0x10;

    // istenen reg adeti kabul edilebilir mi? 
    if (mb_quantityReg >= 1 && mb_quantityReg <= 123) {

        // bu adres bende var mý?
        if (mb_startAddr >= HOLDING_REG_START_ADR && (mb_startAddr + mb_quantityReg) <= (HOLDING_REG_START_ADR + HOLDING_REG_SIZE)) {

            jj=7;
            //cevabý registera yaz..
            for (ii = (uint8_t) mb_startAddr - HOLDING_REG_START_ADR; ii < (mb_startAddr - HOLDING_REG_START_ADR + mb_quantityReg); ii++)             
            {               
                holdingReg[ii]=(uint16_t)((mb_frame[jj]<<8) | mb_frame[jj+1]);
                jj+=2;
            }

            //modbus cevabý hazýrla
            mb_cevap[2] = mb_frame[2]; //adres
            mb_cevap[3] = mb_frame[3];
            mb_cevap[4] = mb_frame[4]; //adet
            mb_cevap[5] = mb_frame[5];

            kk = 6;
            crc_hesapla(kk);
            mb_cevap[kk] = crcHigh;
            mb_cevap[kk + 1] = crcLow;


        } else {
            mb_cevap[1] = 0x90; //exception code
            mb_cevap[2] = 0x02; //
            crc_hesapla(3);
            mb_cevap[3] = crcHigh;
            mb_cevap[4] = crcLow;
            kk = 3;
        }
    } else {
        mb_cevap[1] = 0x90; //exception code
        mb_cevap[2] = 0x03; //
        crc_hesapla(3);
        mb_cevap[3] = crcHigh;
        mb_cevap[4] = crcLow;
        kk = 3;
    }

    mb_cevap_gonder();
}

