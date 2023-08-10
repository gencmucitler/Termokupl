/* 
 * File:   max6675.h
 * Author: Abd
 *
 * Created on 05 Aðustos 2023 Cumartesi, 15:04
 */

#ifndef MAX6675_H
#define	MAX6675_H

#define pinMax6675_CS   LATB2
#define pinMax6675_SCK  LATB1
#define pinMax6675_SDI  RB0

#define pinMax6675_CS_TRIS  TRISB2
#define pinMax6675_SCK_TRIS TRISB1
#define pinMax6675_SDI_TRIS TRISB0




void MAX6675_init();
uint16_t MAX6675_oku();


#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MAX6675_H */

