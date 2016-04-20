/******************************************************
 * File: dataSave.c
 * Creator: Stanley Li
 * Description: Save data into mircochip eeprom
 * Create Date: 2015/12/23 
 * Latest Date: 2015/12/23 
 * version: 1.0
 *****************************************************/

#include "dataSave.h"
//=================== Parameter Saving through RS232==========================//
extern int tmpArrayUR[32]; 
int urDataInRAM[32]; // only can declare 16 bits per data because microchip _memcpy_p2d16


/*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
//int _EEDATA(32) fooArrayInDataEE[] = {0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};
int _EEDATA(64) apgsArrayInDataEE[64];


/*Declare variables to be stored in RAM*/
//int fooArray1inRAM[] = {0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0xABCD, 0xBCDE,
//                       0xCDEF, 0xDEFA, 0x0000, 0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

//int fooArray2inRAM[16];

void read_data_From_EEP(void)
{
    _prog_addressT EE_addr;    

    // initialize a variable to represent the Data EEPROM address 
    _init_prog_address(EE_addr, apgsArrayInDataEE);
    
    /*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/
    _memcpy_p2d16(urDataInRAM, EE_addr, 64);

}

void write_data_to_EEP(void)
{
    _prog_addressT EE_addr;  
    int i=0;
    // initialize a variable to represent the Data EEPROM address 
    _init_prog_address(EE_addr, apgsArrayInDataEE);

    //Erase a row in Data EEPROM
    //_erase_eedata(EE_addr, _EE_ROW);
    _erase_eedata_all();
    _wait_eedata();

    /*Write a row to Data EEPROM from array "fooArray1inRAM" */
    //_write_eedata_row(EE_addr,tmpArrayUR);
    for(i=0;i<NUM_OF_EEPDATA;i++)
    {
        _write_eedata_word(EE_addr + (i*2),tmpArrayUR[i]);
        _wait_eedata();
    } 

}





