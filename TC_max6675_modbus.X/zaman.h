/*******************************************************************************
 * Kütüphane     : miliSaniye                                                  *
 * Yazar         : sigmoid                                                     *
 * Baþlangýç     : 18 Haziran 2017                                             *
 * Versiyon      : 0.1                                                         *
 *                                                                             *
 * PIC kaç miliSaniyedir enerjili olduðunu sayar. Yaklaþýk 50 gün sonra sayaç  * 
 * sýfýrlanýr.                                                                 * 
 ******************************************************************************/

#ifndef ZAMAN_H
#define	ZAMAN_H

//fonksiyon prototipleri
void _miliSaat(void);
unsigned long miliSaniye(void);
char sure_gecti_mi(unsigned long onceki_zaman, unsigned int bekleme);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ZAMAN_H */

