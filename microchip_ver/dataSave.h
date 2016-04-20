#ifndef DATASAVE_h_
#define DATASAVE_h_

#include <p30fxxxx.h>
#include <libpic30.h>

//=====EE Data Define ================
#define CARH  0
#define CARMSA  CARH + 1
#define CARRMIN  CARMSA + 1
#define CARTREAD  CARRMIN + 1
#define CARW  CARTREAD + 1
#define CARL  CARW + 1
#define CARLEN  CARL + 1
#define CARWHSP  CARLEN + 1
#define CARLQ  CARWHSP + 1
#define CARLQS  CARLQ + 1
#define CARLG1  CARLQS + 1
#define CARLG2  CARLG1 + 1
#define CARLG3  CARLG2 + 1
#define CARNC  CARLG3 + 1
#define CARK  CARNC + 1
#define CARLIM_L  CARK + 1
#define CARYAW_BACK  CARLIM_L + 1
#define CARC0  CARYAW_BACK + 1
#define CARG0  CARC0 + 1
#define CARK1  CARG0 + 1
#define CAR_POSOFULTRA  CARK1 + 1
#define DUE_TIME_PARKING  CAR_POSOFULTRA + 1
#define NUM_OF_EEPDATA  DUE_TIME_PARKING + 1
//=====================================
void read_data_From_EEP(void);
void write_data_to_EEP(void);



#endif                                 

