/* 
 * File:   max6675.h
 * Author: Abd
 *
 * Created on 05 Aðustos 2023 Cumartesi, 15:04
 */

#ifndef MAX31855_H
#define	MAX31855_H

#define pinMax31855_CS   LATB1
#define pinMax31855_SCK  LATB0
#define pinMax31855_SDI  RB2

#define pinMax6675_CS_TRIS  TRISB1
#define pinMax6675_SCK_TRIS TRISB0
#define pinMax6675_SDI_TRIS TRISB2

typedef struct sicaklik_degerleri {
    int16_t sicaklik;
    int16_t ic_sicaklik;
    char hata:1;            //genel hata varsa
    char hata_scv:1;        //Vcc ye kýsa devre
    char hata_scg:1;        //gnd ye kýsa devre
    char hata_oc:1;          //açýk devre
}sicaklik_t;


void MAX31855_init();
sicaklik_t MAX31855_oku();


#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MAX6675_H */

