/*******************************************************************************
 * K�t�phane     : miliSaniye                                                  *
 * Yazar         : sigmoid                                                     *
 * Ba�lang��     : 18 Haziran 2017                                             *
 * Versiyon      : 0.1                                                         *
 *                                                                             *
 * PIC ka� miliSaniyedir enerjili oldu�unu sayar. Yakla��k 50 g�n sonra saya�  * 
 * s�f�rlan�r.                                                                 * 
 ******************************************************************************/

#include "zaman.h"


volatile unsigned long _msaniye=0;

//her milisaniyede bir artacak. maksimum 50 g�ne kadar sayabiliyor.
//2^32 /1000 = x saniye  x/60/60/24 =49,71 g�n
void _miliSaat(void)        //Timer2/4/6 kesmesi taraf�ndan tetiklenmelidir.
{
    _msaniye++;
}

//mili saniye cinsinden mikrodenetleyicinin ne kadar s�re �al��t��� bilgisini d�nd�r�r.
unsigned long miliSaniye(void)
{    
    return _msaniye;
}

char sure_gecti_mi(unsigned long onceki_zaman, unsigned int bekleme)
{
    unsigned long zaman;
    zaman=miliSaniye();
    //onceki zaman'dan bekleme s�resi kadar zaman ge�ti mi?
    if((zaman - onceki_zaman) >= (unsigned long)bekleme)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
