/******************************************************
 * File: main_APGS.c
 * Description: main file for Auto parking Guide System
 * Creator Date: 2015/12/09
 * Latest Date: 2015/12/09 
 * Author: Stanley Li
 * version: 1.0
 *****************************************************/

//------------Include head files ------------------------------------
#include 	<timer.h>
#include 	<p30FXXXX.h>
#include 	<can.h>
#include    <incap.h>
#include	<math.h>
#include	<uart.h>
#include	<stdio.h>
#include	<adc10.h>
#include 	<spi.h>
#include 	<stdlib.h>
#include "APGS.h"
#include "rtwtypes.h"
#include "dataSave.h"
//----------End of Include head files ---------------------------------

//------------ Named constants ----------------------------------------
#define	FCY			10000000*2			// Osc. configuration = XT, PLL * 8 and XT = 7.3728 Mhz
#define R_GEAR 119  // gear position of Revese
#define D_GEAR 81   // gear position of direct
#define LED_FORWARD		LATBbits.LATB11
#define DIR_LED_FORWARD	TRISBbits.TRISB11
#define LED_BACK		LATBbits.LATB12
#define DIR_LED_BACK	TRISBbits.TRISB12
#define Buzzer_Bark		LATBbits.LATB13
#define DIR_Buzzer_Bark	TRISBbits.TRISB13
#define TIME_FOR_END_MSG_DISP 3000 // 3 sec
//#define DUETIME_FOR_PARKING 135000 // 180 sec
//------------ End of Named constants define -----------------------------

//------------ Foward declaration for local functions -----------------
void sys_Reset(void);
void PparkingFlow(void);
void Odometery_Calc(void);
void logInOutEPS(void);
void INTx_IO_Init(void);
void Timer4_Initial( void ) ;
void Timer1_Initial( void ) ;
void Timer5_Initial( void ) ;
void Init_Capture1(void);
void Init_Capture2(void);
void Initial_CAN1( void );
void Initial_CAN2( void );
void Init_ADC(void);
void PID(void);
void Init_UART(void);
void Init_UART2(void);
void Init_SPI2(void);
void DelayNmsec(unsigned int N);
void delay_ms(void);
void Ppathflow(void);
void MsgToFinish(void);
void SerialDataDecrypt(void);
void Load_data_From_EE(void);
void closePeripherals(void);
void APGS_param_reset(void);
//------------ End of Foward declaration for local functions -----------------

//------------ Variables declare -----------------
int sampleTime=0;
float counter1_R=0.0,counter1_L=0.0;
float counter=0.0;
char back_final = 0;

unsigned char t = 0; 
char t_count = 0;
int iii = 0;
unsigned char wheelSpeedData[8] = {0,0,0,0,0,0,0,0};  
unsigned char TiguanUltraData[8] = {0,0,0,0,0,0,0,0};
unsigned char GearRxData[8] = {0,0,0,0,0,0,0,0};  
unsigned char STEERCMD[2] = {0,0};  
unsigned char LOGIN_OUT_EPS_DATA[1] = {0};
unsigned char tiguanAngleData[2] = {0,0};
unsigned char dir_Light[8] = {0,0,0,0,0,0,0,0};
unsigned char right_left_SideData[4] = {0,0,0,0};
unsigned char uiData[9][5] = {{0,0,0,0,0},
                              {5,64,0,0,0},
                              {5,64,32,0,0},
                              {5,64,32,0,0}, 
                              {37,64,0,0,0},
                              {21,96,24,0,0},
                              {21,64,19,0,0},
                              {21,96,19,0,0},
                              {11,65,0,0,0}};  
unsigned char xyData[4] = {0,0,0,0};
unsigned char laserData[1]= {0};
unsigned short wheelL;
unsigned short wheelR;
float wheelSpL;
float wheelSpR;
float x1,y1,yaw_odm,deltaD,avgD;
float SAngle;
float front_right,back_right,front_left,back_left;
unsigned char ucParkingMode; //Parking Mode
unsigned char shiftData;
enum {APGS_STANDBY = 0, APGS_PARALLEL, APGS_BACKIN,APGS_DRIVEOUT};
enum {PARALLEL_MODE = 1, BACKIN_MODE, PS_SEARCH,BS_SEARCH,PS_FOUND,BS_FOUND,PS_FINISH,BS_FINISH,GEAR_F,GEAR_R};
enum {DEFAULT_PARKING = 1,RIGHT_PARKING = 17,LEFT_PARKING = 9,BLINK_LIGHT = 25};
unsigned char buzz_Cnt = 0;
unsigned char HMI_MSG[1] = {0};
int32_T old_steer_cmd = 0; 
int32_T new_steer_cmd = 0; 

float old_str_cmd = 0; 
float new_str_cmd = 0; 
float out_str_cmd = 0; 

unsigned char change_cmd_cnt = 0;
unsigned char ultra_change_cnt = 0;
unsigned char Is_front_back_ultra_update = 0;

int32_T eps_steer_cmd = 0; 

float laser_front_right = 0;
unsigned char dir_Light_Data = 0;
unsigned char old_dir_L = RIGHT_PARKING;
unsigned char new_dir_L = RIGHT_PARKING;
unsigned char dir_light = RIGHT_PARKING;


//=================== Parameter Saving through RS232==========================//

unsigned char urData[NUM_OF_EEPDATA + NUM_OF_EEPDATA];
unsigned char urDataStep = 0; 
unsigned char urDataCnt = 0; 
unsigned char ur_ckSum = 0; 
unsigned char numOfURdata = 0;
unsigned char IsTrigEEpDataSave = 0;
int tmpArrayUR[NUM_OF_EEPDATA]; 
int tmpUR_Cnt = 0;
unsigned char tmpURdata;
unsigned char IsPCsendCMD = 0; 
extern int urDataInRAM[];
//----------------------------------------
unsigned char IsUnexpectChg_Parallel = 0;
unsigned char IsUnexpectChg_BackIn = 0;
extern real_T diff_x_pos;
unsigned short finish_Cnt = 0;
unsigned long duetime_Cnt = 0;

//---------------------------------------------------------------------------
// Configuration bits declaration
//---------------------------------------------------------------------------
   	_FOSC(CSW_FSCM_OFF & XT_PLL8);   //XT with 8xPLL oscillator, Failsafe clock off
   	_FWDT(WDT_OFF);                  //Watchdog timer disabled
  	_FBORPOR(PBOR_OFF & MCLR_EN);    //Brown-out reset disabled, MCLR reset enabled
   	_FGS(CODE_PROT_OFF);             //Code protect disabled
	


//-------------------------------------------------------------------------------------//
//---------------------------Timer1 Interrupt Service Routine---------------------------------//
//-------------------------------------------------------------------------------------//
void __attribute__((__interrupt__)) _T1Interrupt(void)//1ms
{
    //--------- Input for Model Based ------------------//
    APGS_U.wheel_L = wheelL;
    APGS_U.wheel_R = wheelR;
    APGS_U.gear_pos =  shiftData; 
    APGS_U.front_right = front_right; 
    APGS_U.SAngle = SAngle;
    APGS_U.parkMode = ucParkingMode;
    APGS_U.back_right = back_right;
    APGS_U.back_left = back_left;
    APGS_U.front_left = front_left; 
    APGS_U.dir_Light = dir_light;

    //--------- Code of Model Based --------------------//
    APGS_step();
    //---------- Steering Command ----------------------//
    if ( APGS_B.steerCtlParkingFcn)  // Steeing Control
    { 
        if(LOGIN_OUT_EPS_DATA[0] != 1)
        {
            LOGIN_OUT_EPS_DATA[0] = 1; // Log in EPS
            logInOutEPS();
        }
        
        //---------------Log out EPS-----------------------
        if(APGS_DW.ActLevel == 5)
        {
             APGS_B.PparkingFlowFcn = APGS_DW.disable;
             APGS_B.BparkingFlowFcn = APGS_DW.disable;
             LOGIN_OUT_EPS_DATA[0] = 0;
             logInOutEPS(); // Log out EPS control from Tiguan
             
        }

        //--------------Slow down steer speed if backIn_step > 7, just in case steer moving too fast------------
        out_str_cmd = APGS_DW.steer_cmd; // put current cmd to out_str_cmd
        if((ucParkingMode == APGS_BACKIN) && (APGS_DW.backIn_step > 7)) {
            new_str_cmd = out_str_cmd;
            if(new_str_cmd > old_str_cmd)
            {
                new_str_cmd= old_str_cmd + 0.5; // increase command with 0.5 degree per step
                if(new_str_cmd < APGS_DW.steer_cmd)
                out_str_cmd = new_str_cmd;
            }
            else
            {
                new_str_cmd=  old_str_cmd - 0.5;  // reduce ommand with 0.5 degree per step
                if(new_str_cmd > APGS_DW.steer_cmd)
                    out_str_cmd = new_str_cmd;
            }
            old_str_cmd = new_str_cmd; // update old command
        }

              
        // --------------Left or Right parking decision------------------------------
        if (APGS_U.dir_Light == RIGHT_PARKING) 
            eps_steer_cmd = (int_T)((out_str_cmd / 0.043)*-1); // Instead of -1, give 1 for left side
        else if(APGS_U.dir_Light == LEFT_PARKING)
            eps_steer_cmd = (int_T)((out_str_cmd / 0.043)*1); 

        //--------------Transfer steer command into real EPS command --------------
        if(eps_steer_cmd > 0) // Postive and nagtive command
        {        
            STEERCMD[0] = eps_steer_cmd & 0x00FF; // Low byte
            STEERCMD[1] = (eps_steer_cmd & 0xFF00) >> 8; // hi byte
        }
        else
        {
            STEERCMD[0] = abs(eps_steer_cmd) & 0x00FF; // Low byte
            STEERCMD[1] = ((abs(eps_steer_cmd) & 0xFF00) >> 8) | 0x80; //hi byte with negative
        }

        //---------------Send steer command to gateway1 for EPS control
        CAN2SendMessage( ( CAN_TX_SID(0x3E6)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ , 
				   	  ( CAN_TX_EID(0x3E6)) & CAN_NOR_TX_REQ , STEERCMD, 2 , 0 ) ;//Send Steer command to ID 998
        while(!CAN2IsTXReady(0));
    }

    //------------------- UI Information -------------------------//    
    HMI_MSG[0] = (unsigned char)APGS_DW.MsgForUI; // Get UI index
    CAN1SendMessage( ( CAN_TX_SID(0x5D6)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ , 
				   	  ( CAN_TX_EID(0x5D6)) & CAN_NOR_TX_REQ , *(uiData + (unsigned char)APGS_DW.MsgForUI) , 5 , 0 ) ; //Send UI command to Tiguan's HMI 
    while(!CAN1IsTXReady(0));

    //---------------Counts for finish message diplay------------------------
    if(APGS_DW.ActLevel == 5)
        finish_Cnt++;
    //---------------Counts for duetime of parking action -------------------
    if(APGS_DW.ActLevel >= 4 && duetime_Cnt < APGS_EE.due_time_parking)
        duetime_Cnt++;
    if(duetime_Cnt >= APGS_EE.due_time_parking) 
    {
        //----------Reset and jump to finish level ---------------------
        APGS_param_reset();
        APGS_DW.ActLevel =5;
        APGS_DW.MsgForUI =8;
        duetime_Cnt = 0;
        APGS_DW.is_active_c1_APGS = 1;
        APGS_DW.is_c1_APGS = 7;
        
    }
    //------------------------------------------------------------------------

	IFS0bits.T1IF = 0 ; //Reset Timer 1 interrupt
}

//--------------------------------------------------------------------------------------//
//---------------------------CAN1 Interrupt Service Routine---------------------------------//
//--------------------------------------------------------------------------------------//
void _ISR _C1Interrupt(void)
{
	if(C1INTFbits.RX0IF)
	{
		if(C1RX0CONbits.FILHIT0==0) //Get wheel speed from Tiguan
		{
			CAN1ReceiveMessage( wheelSpeedData,8,0 ) ;
            wheelL = (wheelSpeedData[4] | (wheelSpeedData[5] << 8)); // Left wheel speed
            wheelR = (wheelSpeedData[6] | (wheelSpeedData[7] << 8)); // Right wheel speed
            APGS_DW.car_Speed = wheelL/APGS_EE.whsp_factor; // Transfer wheel into car speed, 198 is the gain for get real speed
            Odometery_Calc(); // Function of odometery calculation, it is to get x y coordinate.
 
		}
		else if(C1RX0CONbits.FILHIT0==1) // Get steer angle from Tiguan
		{
			CAN1ReceiveMessage( tiguanAngleData,2,0 ) ;	
            if((tiguanAngleData[1] & 0x80) == 0x80)// 0x80 is for negative
                SAngle = -(tiguanAngleData[0] | ((tiguanAngleData[1]&0x7F) << 8));
            else
                SAngle = tiguanAngleData[0] | ((tiguanAngleData[1]&0x7F) << 8); 
            SAngle = SAngle*0.043; //Times the gain 0.043 to  tranfer digital into real angle.

		}
		else
			;

		C1INTFbits.RX0IF= 0;
		C1RX0CONbits.RXFUL = 0 ;
	}

	if(C1INTFbits.RX1IF)
	{		
		if(C1RX1CONbits.FILHIT==2) // Get gear posiition from Tiguan
		{
			CAN1ReceiveMessage( GearRxData,8,1 ) ;
            shiftData = GearRxData[1];
		}
		else if(C1RX1CONbits.FILHIT==3) // Get direction light from Tiguan
		{
            CAN1ReceiveMessage( dir_Light,8,1 ) ;
            //---- direction of light
            if(dir_Light[1] == BLINK_LIGHT) // Right and blink is the same as right parking
                dir_Light[1] = RIGHT_PARKING; 
			dir_Light_Data = dir_Light[1];
            //----- Decision for direction of light  
            new_dir_L = dir_Light_Data;
            if(new_dir_L !=1)
            {
               dir_light = dir_Light_Data; // give a value for current status
               old_dir_L = new_dir_L; // Store new value for old
            } else {
               dir_light = old_dir_L; // Give old status if dir is 1
            }
		}
		else
			;
		C1INTFbits.RX1IF= 0;
		C1RX1CONbits.RXFUL = 0 ;
	}

	IFS1bits.C1IF= 0;
}

//--------------------------------------------------------------------------------------//
//---------------------------CAN2 Interrupt Service Routine---------------------------------//
//--------------------------------------------------------------------------------------//
void _ISR _C2Interrupt(void)
{
	if(C2INTFbits.RX0IF)
	{
		if(C2RX0CONbits.FILHIT0==0)  //Get right side distance from laser scanner
		{

            //CAN2ReceiveMessage( laserData,1,0 ) ;
            //laser_front_right = laserData[0];
            //APGS_U.update_Ultra = 1;

		}
		else if(C2RX0CONbits.FILHIT0==1) // Get front/rear ultrasonic
		{
            CAN2ReceiveMessage( TiguanUltraData,8,0 ) ;
            //---- Give model-based input from Ultrasonic infomation 
            if(APGS_U.dir_Light == RIGHT_PARKING)
            { 
            //----For right side
			    APGS_U.ultraSonic_RCL = TiguanUltraData[4];
			    APGS_U.ultraSonic_RML = TiguanUltraData[5];
			    APGS_U.ultraSonic_RMR = TiguanUltraData[6];
			    APGS_U.ultraSonic_RCR = TiguanUltraData[7];
			    APGS_U.ultraSonic_FCL = TiguanUltraData[0];
			    APGS_U.ultraSonic_FML = TiguanUltraData[1];
			    APGS_U.ultraSonic_FMR = TiguanUltraData[2];
			    APGS_U.ultraSonic_FCR = TiguanUltraData[3];
            } else { 
            //----For left side
                APGS_U.ultraSonic_RCL = TiguanUltraData[7];
			    APGS_U.ultraSonic_RML = TiguanUltraData[6];
			    APGS_U.ultraSonic_RMR = TiguanUltraData[5];
			    APGS_U.ultraSonic_RCR = TiguanUltraData[4];
			    APGS_U.ultraSonic_FCL = TiguanUltraData[3];
			    APGS_U.ultraSonic_FML = TiguanUltraData[2];
			    APGS_U.ultraSonic_FMR = TiguanUltraData[1];
			    APGS_U.ultraSonic_FCR = TiguanUltraData[0];
            }
            Is_front_back_ultra_update = 1; // update flag
		}
		else
			;
		C2INTFbits.RX0IF= 0;
		C2RX0CONbits.RXFUL = 0 ;
	}
    if(C2INTFbits.RX1IF)
	{ 
		if(C2RX1CONbits.FILHIT==2) // Get front left side /front right side/rear left side /rear right side distance from ultrasonic
		{

            CAN2ReceiveMessage(right_left_SideData,4,1 ) ;
            if(APGS_U.dir_Light == RIGHT_PARKING)
            {
            //----For right side
               front_right = right_left_SideData[0];
               front_left = right_left_SideData[2]; 
               back_right = right_left_SideData[1];
               back_left = right_left_SideData[3];
            } else {   
            //----For left side 
               front_left = right_left_SideData[0];
               front_right = right_left_SideData[2];  
               back_right = right_left_SideData[3];
               back_left = right_left_SideData[1];
            }
            APGS_U.update_Ultra = 1;
	}
		else
			;
		C2INTFbits.RX1IF= 0;
		C2RX1CONbits.RXFUL = 0 ;        
    }
    
	IFS2bits.C2IF= 0;
}
//---------------------------------------------------------------------------------------//
//---------------------------Timer5 Interrupt Service Routine---------------------------------//
//---------------------------------------------------------------------------------------//
void _ISR _T5Interrupt(void)
{ 
    if(APGS_DW.vld_status == 1)    
    {
        if(buzz_Cnt < 1)
        {
            Buzzer_Bark = 0;
            buzz_Cnt++;
        }
        else
        {  
            Buzzer_Bark = 1;
        }
    }
    else
        buzz_Cnt = 0;

    IFS1bits.T5IF = 0 ;
}
//---------------------------------------------------------------------------------------//
//---------------------------External Interrupt Service Routine--------------------------------//
//---------------------------------------------------------------------------------------//
void __attribute__((__interrupt__)) _INT3Interrupt(void)
{
    APGS_param_reset(); //Reset parameters for next parking 
    ucParkingMode++; // add one once botton triggered
    sys_Reset(); // Reset EPS information, just in case eps keep old action
    
    if(ucParkingMode < APGS_DRIVEOUT && ucParkingMode != APGS_STANDBY) // Start to scan parkng space
        APGS_U.edgDetStartFlg = 1;

    if(ucParkingMode == APGS_PARALLEL) // parallel mode
        HMI_MSG[0] = PARALLEL_MODE; 
    if(ucParkingMode == APGS_BACKIN)  // back in mode
        HMI_MSG[0] = BACKIN_MODE; 

    if(ucParkingMode >= APGS_DRIVEOUT) // reset to stand by mode 
    {
        ucParkingMode = APGS_STANDBY;
        APGS_U.edgDetStartFlg = 0;
    }

    //--------Make sure steer control is free ------------------------
    LOGIN_OUT_EPS_DATA[0] = 0;
    logInOutEPS();
    //--------------------------------------------------------- 
   

	IFS2bits.INT3IF = 0;    //Clear the INT1 interrupt flag or else
                            //the CPU will keep vectoring back to the ISR
}
//----------------------------------------------------------------------------------------//
void _ISR _U1RXInterrupt(void)            // UART1 RX
{
    tmpURdata = ReadUART1();           // Read data
    SerialDataDecrypt();               // Request decrytion
   	IFS0bits.U1RXIF= 0;                // Clear the ur1 interrupt flag
}
//---------------------------------------------------------------------------------------//
//---------------------------Main function -------------------------------------------------//
//---------------------------------------------------------------------------------------//
int	main( void )
{	

	SRbits.IPL=4;
    sampleTime = 1; // Set Timer 1 to 1ms
    DIR_LED_FORWARD	= 0;
    DIR_LED_BACK	= 0;
    DIR_Buzzer_Bark = 0; // Set B13 to output
    Buzzer_Bark = 1;
    LED_FORWARD = 1;
    LED_BACK = 1; 
    //----- Initialize model parameters ------------------------
    APGS_initialize();
    //----- Load data from eeprom ------------------------------
    Load_data_From_EE();
    //-----Firmware Initialization -----------------
    Timer1_Initial();
    Timer5_Initial();
    Init_UART();
	Initial_CAN1();
	Initial_CAN2();
    INTx_IO_Init();   

    //--------- Print system work message ------------------------------
    printf("Waiting for System Start.....\r\n");    
    //-------- Waiting for system start ---------------------------------
    while(1)
    {
        while(!APGS_U.edgDetStartFlg)
        {
            if(IsPCsendCMD == 0)
            {
            //printf("\r\n%f  %f  %f  %f  %f  %f  %f  %f\r\n",APGS_U.ultraSonic_RCL,APGS_U.ultraSonic_RML,APGS_U.ultraSonic_RMR,APGS_U.ultraSonic_RCR,APGS_U.ultraSonic_FCL,APGS_U.ultraSonic_FML,APGS_U.ultraSonic_FMR,APGS_U.ultraSonic_FCR) ;
            //printf("%ld\r\n",APGS_EE.due_time_parking);
            }
            //-----------Start to close peripheral if user is going to rewrite eeprom
            if(IsPCsendCMD==1)
            {
                closePeripherals(); //close peripherals
                IsPCsendCMD = 2;    //Set flag 2 to avoid execute repeatedly
            }
            if(IsTrigEEpDataSave)
            {
                MsgToFinish(); // Send message for finish verification
                IsTrigEEpDataSave = 0; //reset
            }      
        }//---End of while(!APGS_U.edg..........
 
        //---------- Processing of parking space searching ---------------------
        while(!APGS_B.steerCtlParkingFcn)
        {
            printf("%f %f %f %f %f %f %f %f\r\n",APGS_DW.x_pos,APGS_DW.y_pos,APGS_DW.first_edg_pos,APGS_DW.second_edg_pos, APGS_DW.parking_space,APGS_DW.parking_n,APGS_U.front_right,APGS_DW.yaw_odm);
            //printf("%d\r\n",ucParkingMode);
            if(ucParkingMode == APGS_STANDBY)
                return;
        }    

        APGS_DW.yaw_odm = 0; // Reset yaw angle once parking space found
        //---------- Idle for parking action ---------------------
        while(APGS_B.steerCtlParkingFcn)
        {
           Ppathflow(); // To locate where host car is and give the proper steer command by following defined strategy.  
           //printf("dir=%d\r\n",dir_light); 
           printf("%f %ld\r\n",APGS_DW.turningStep,duetime_Cnt);
           //printf(" %f %f %f\r\n",APGS_DW.backIn_step,APGS_DW.multiShift,APGS_U.back_left);
        }
        //------------ Log out EPS once system is finished---------------------------
        LOGIN_OUT_EPS_DATA[0] = 0;
        logInOutEPS();   
        //---------------------------------------------------------------------
        while(finish_Cnt< TIME_FOR_END_MSG_DISP){printf("%d\r\n",APGS_DW.is_c1_APGS);} // Waiting for 5 sec
        APGS_param_reset(); //Reset parameters for next parking
        ucParkingMode = 0;
        
    } // End of while(1)

}  // main

/******************************************************
 * Function: Odometery_Calc
 * Description: Get x/y coordinate of host car by using odometery.
 * Parameters: None
 * Return value: None
 *****************************************************/
void Odometery_Calc(void)
{
   real_T Rs;
   real_T L;
   real_T L_theta;
   int16_T pos_sign;
   int16_T o_count; 
   real_T avgD;
   int16_T ckRltPos;
   int16_T stepOneAngle; 

   //---------Start calculation if true -----------------------------------------------
    if ((((APGS_B.edgDetFcn != ((uint16_T)0U)) || (APGS_B.PparkingFlowFcn !=
           ((uint16_T)0U))) || (APGS_B.BparkingFlowFcn != ((uint16_T)0U)))) {
     
      stepOneAngle = (int16_T)1; // initial the sign for x/y direction

      
  
      o_count = (int16_T)1; // initial the sign for yaw direction

      if (APGS_DW.ActLevel == ((uint16_T)6U)) { // System is going to do parallel or back-in parking 
        stepOneAngle = (int16_T)-1;  // Change x/y direction
        if (!((APGS_U.gear_pos == ((uint16_T)119U)) || (!(APGS_U.gear_pos ==
               ((uint16_T)81U))))) {
         
          o_count = (int16_T)-1; //change the sign for yaw direction
        } else {
          
        }
      }

      if (APGS_U.dir_Light == 17.0) { //if direction light is right
       
        ckRltPos = (int16_T)1;
      } else if (APGS_U.dir_Light == 9.0) { //if direction light is left
        
        ckRltPos = (int16_T)-1;
      } else {
        
        ckRltPos = (int16_T)1;
      }

       
      Rs = APGS_U.wheel_L / (APGS_EE.whsp_factor*360); // Left wheel speed 71280 = (198*3600)/10

      
      L = APGS_U.wheel_R / (APGS_EE.whsp_factor*360); // Right wheel speed 71280 = (198*3600)/10

      
      L_theta = (Rs + L) / 2.0; //Average

      
      APGS_DW.yaw_odm += (((atan((Rs - L) / APGS_P.carTread) * ((real_T)ckRltPos))
                           * 57.325) * ((real_T)stepOneAngle)) * ((real_T)
        o_count); // Get yaw angle by accumulating per step

      
      APGS_DW.angle = -(APGS_DW.yaw_odm / 57.325); // transfer degree into radius
      if (APGS_U.gear_pos == ((uint16_T)119U)) { // For revers 
        
        APGS_DW.x_pos += ((((real_T)stepOneAngle) * L_theta) * cos(APGS_DW.angle))
          * -100.0; // get x position by using odometery, uint:cm

        
        APGS_DW.y_pos += ((((real_T)stepOneAngle) * L_theta) * sin(APGS_DW.angle))
          * -100.0; // get y position by using odometery, uint:cm

        
      } else {// For forward
        if ((APGS_U.gear_pos == ((uint16_T)81U)) || (APGS_U.gear_pos ==
             ((uint16_T)82U))) {
          
          APGS_DW.x_pos += ((((real_T)stepOneAngle) * L_theta) * cos
                            (APGS_DW.angle)) * 100.0;  //get x position by using odometery, uint:cm

          
          APGS_DW.y_pos += ((((real_T)stepOneAngle) * L_theta) * sin
                            (APGS_DW.angle)) * 100.0;  //get y position by using odometery, uint:cm

          
        }
      }

      
    }


 
}
/******************************************************
 * Function: logInOutEPS
 * Description: Get x/y coordinate of host car by using odometery.
 * Parameters: None
 * Return value: None
 *****************************************************/
void logInOutEPS(void)
{
        CAN2SendMessage( ( CAN_TX_SID(0x3E5)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ , 
				   	  ( CAN_TX_EID(0x3E5)) & CAN_NOR_TX_REQ , LOGIN_OUT_EPS_DATA, 1 , 0 ) ;//Send Steer command to ID 998
        while(!CAN2IsTXReady(0));
}

/******************************************************
 * Function: INTx_IO_Init
 * Description: Initial IO for external interrupt.
 * Parameters: None
 * Return value: None
 *****************************************************/
void INTx_IO_Init(void)
{
        INTCON2 = 0x0008;       /*Setup INT3pins to interupt */
        IPC9bits.INT3IP = 6;    /*on falling edge and set up INT3 pin to interupt */
        IFS2bits.INT3IF = 0;    /*Reset INT3 interrupt flag */
        IEC2bits.INT3IE = 1;    /*Enable INT3 Interrupt Service Routine */
		//ConfigINT1(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_0);
}

/******************************************************
 * Function: Timer1_Initial
 * Description: Timer1 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void	Timer1_Initial( void )
{
		ConfigIntTimer1( T1_INT_PRIOR_6 & T1_INT_ON ) ;
		OpenTimer1( T1_ON & T1_IDLE_STOP & T1_GATE_OFF & T1_PS_1_256 & T1_SYNC_EXT_OFF & T1_SOURCE_INT ,
					((long)(FCY/ 1000 )* sampleTime / 256) ) ;  //(FCY/ 1000 )* 300 / 64);
}													  
/******************************************************
 * Function: Timer4_Initial
 * Description: Timer4 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void	Timer4_Initial( void )
{
		ConfigIntTimer4( T4_INT_PRIOR_7 & T4_INT_ON ) ;
		OpenTimer4( T4_ON & T4_IDLE_STOP & T4_GATE_OFF & T4_PS_1_1 & T4_SOURCE_INT ,
					((long)(FCY/ 1000)*1/1)) ;
}
/******************************************************
 * Function: Init_Capture1
 * Description: Capture1 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_Capture1(void)
{
      OpenCapture1(IC_IDLE_STOP & IC_TIMER3_SRC &
                IC_INT_1CAPTURE & IC_EVERY_EDGE);
      ConfigIntCapture1(IC_INT_PRIOR_7 & IC_INT_ON);
}
/******************************************************
 * Function: Init_Capture2
 * Description: Capture2 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_Capture2(void)
{
      OpenCapture2(IC_IDLE_STOP & IC_TIMER2_SRC &
                IC_INT_1CAPTURE & IC_EVERY_EDGE);
      ConfigIntCapture2(IC_INT_PRIOR_7 & IC_INT_ON);
}
/******************************************************
 * Function: Initial_CAN1
 * Description: CAN1 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Initial_CAN1( void )
{
	// Set request for configuration mode
	CAN1SetOperationMode( 	CAN_IDLE_CON & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_CONFIG ) ;
	while ( C1CTRLbits.OPMODE <= 3 );

	// Load Configuration register
	CAN1Initialize( CAN_SYNC_JUMP_WIDTH1 & CAN_BAUD_PRE_SCALE(1),
			CAN_WAKEUP_BY_FILTER_DIS & CAN_PHASE_SEG2_TQ(6) &
			CAN_PHASE_SEG1_TQ(8) & CAN_PROPAGATIONTIME_SEG_TQ(5) & 
			CAN_SEG2_FREE_PROG & CAN_SAMPLE1TIME ) ;

	// Load acceptance filter resister
	CAN1SetFilter ( (char) 0 , CAN_FILTER_SID( 0x4A0 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));//1088 gear
	CAN1SetFilter ( (char) 1 , CAN_FILTER_SID( 0xC2 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 )) ;//194 angle//

	CAN1SetFilter ( (char) 2 , CAN_FILTER_SID( 0x440 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));
	CAN1SetFilter ( (char) 3 , CAN_FILTER_SID( 0x392 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));

	// Load Mask filter register 
	CAN1SetMask ( (char) 0 , CAN_MASK_SID ( 0x7ff) & CAN_MATCH_FILTER_TYPE , CAN_MASK_EID( 0x00) ); 
	CAN1SetMask ( (char) 1 , CAN_MASK_SID ( 0x7ff) & CAN_MATCH_FILTER_TYPE , CAN_MASK_EID( 0x00) );

	// Set TX/RX mode
	CAN1SetTXMode( (char) 0 , CAN_TX_STOP_REQ & CAN_TX_PRIORITY_HIGH ) ;
	CAN1SetTXMode( (char) 1 , CAN_TX_STOP_REQ & CAN_TX_PRIORITY_HIGH ) ;

	CAN1SetRXMode( (char) 0 , CAN_RXFUL_CLEAR & CAN_BUF0_DBLBUFFER_DIS ) ;
	CAN1SetRXMode( (char) 1 , CAN_RXFUL_CLEAR) ;


	// Set request for normal mode
	CAN1SetOperationMode( 	CAN_IDLE_CON & CAN_CAPTURE_DIS & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_NOR );
	while ( C1CTRLbits.OPMODE != 0 );
			
	ConfigIntCAN1(CAN_INDI_INVMESS_DIS & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS & CAN_INDI_TXB2_DIS & 
		      CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_DIS & CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,
		      CAN_INT_PRI_7 & CAN_INT_ENABLE); 							
}
/******************************************************
 * Function: Initial_CAN2
 * Description: CAN2 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void	Initial_CAN2( void )
{
	// Set request for configuration mode
	CAN2SetOperationMode( 	CAN_IDLE_CON & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_CONFIG ) ;
	while ( C2CTRLbits.OPMODE <= 3 );

	// Load Configuration register
	CAN2Initialize( CAN_SYNC_JUMP_WIDTH1 & CAN_BAUD_PRE_SCALE(1),
			CAN_WAKEUP_BY_FILTER_DIS & CAN_PHASE_SEG2_TQ(6) &
			CAN_PHASE_SEG1_TQ(8) & CAN_PROPAGATIONTIME_SEG_TQ(5) & 
			CAN_SEG2_FREE_PROG & CAN_SAMPLE1TIME ) ;

	// Load acceptance filter resister
	CAN2SetFilter ( (char) 2 , CAN_FILTER_SID( 0x3E7 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));//Right side Ultrasonic
	CAN2SetFilter ( (char) 1 , CAN_FILTER_SID( 0x7DF ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));//Front and back Ultrasonic from Tiguan
    CAN2SetFilter ( (char) 0 , CAN_FILTER_SID( 0x01 ) & CAN_RX_EID_DIS , CAN_FILTER_EID( 0x00 ));//laser data

	// Load Mask filter register 
	CAN2SetMask ( (char) 0 , CAN_MASK_SID ( 0x7ff) & CAN_MATCH_FILTER_TYPE , CAN_MASK_EID( 0x00) ) ; 
	CAN2SetMask ( (char) 1 , CAN_MASK_SID ( 0x7ff) & CAN_MATCH_FILTER_TYPE , CAN_MASK_EID( 0x00) ) ;

	// Set TX/RX mode
	CAN2SetTXMode( (char) 0 , CAN_TX_STOP_REQ & CAN_TX_PRIORITY_HIGH ) ;
	CAN2SetTXMode( (char) 1 , CAN_TX_STOP_REQ & CAN_TX_PRIORITY_HIGH ) ;

	CAN2SetRXMode( (char) 0 , CAN_RXFUL_CLEAR & CAN_BUF0_DBLBUFFER_DIS ) ;
	CAN2SetRXMode( (char) 1 , CAN_RXFUL_CLEAR) ;


	// Set request for normal mode
	CAN2SetOperationMode( 	CAN_IDLE_CON & CAN_CAPTURE_DIS & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_NOR );
	while ( C2CTRLbits.OPMODE != 0 );
			
	ConfigIntCAN2(CAN_INDI_INVMESS_DIS & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS & CAN_INDI_TXB2_DIS & 
		      CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_DIS & CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,
		      CAN_INT_PRI_7 & CAN_INT_ENABLE); 							
}
/******************************************************
 * Function: Init_ADC
 * Description: ADC initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_ADC(void)
{
	unsigned int Channel, PinConfig, Scanselect, Adcon3_reg, Adcon2_reg, Adcon1_reg;

	ADCON1bits.ADON = 0; /* turn off ADC */

	PinConfig = ENABLE_AN3_ANA &		// Select port pins as analog inputs ADPCFG<15:0>
				ENABLE_AN2_ANA ;

	Adcon1_reg = ADC_MODULE_ON &		// Turn on A/D module (ADON)
		ADC_IDLE_STOP &					// ADC turned off during idle (ADSIDL)
		ADC_FORMAT_INTG &				// Output in integer format (FORM)
		ADC_CLK_AUTO &					//*Conversion trigger automatically (SSRC)
		ADC_SAMPLE_SIMULTANEOUS &		//Sample channels simultaneously (SIMSAM)
		ADC_AUTO_SAMPLING_ON &			//Sample trigger automatically (ASAM)	
		ADC_SAMP_ON ;					//										

	Adcon2_reg = ADC_VREF_AVDD_AVSS &	// Voltage reference : +AVdd, -AVss (VCFG)
		ADC_SCAN_OFF &					// Scan off (CSCNA)
		ADC_ALT_BUF_OFF &				// Use fixed buffer (BUFM)
		ADC_ALT_INPUT_OFF &				// Does not alternate between MUX A & MUX B (ALTS)
		ADC_CONVERT_CH_0A &				// Convert only channel 0 (CHPS)
		ADC_SAMPLES_PER_INT_1;			//G2 samples between interrupt (SMPI)

	Adcon3_reg = ADC_SAMPLE_TIME_31 &	// Auto-Sample time,31 TAD(SAMC)
		ADC_CONV_CLK_SYSTEM &			// Use system clock (ADRC)
		ADC_CONV_CLK_32Tcy;				// Conversion clock = 32 Tcy (ADCS)
										// ADCS = 2*(154ns)/(1/Fcy)-1 = 3.5416
										// TAD = (ADCS+1)/(2*Fcy) = 169.54ns

	Scanselect = SCAN_NONE;				// ADC scan no channel (ADCSSL)

	OpenADC10(Adcon1_reg, Adcon2_reg, Adcon3_reg, PinConfig, Scanselect);

	Channel = ADC_CH0_POS_SAMPLEA_AN2 & //*­«­n­×§ï¡GCH0 Pos. : AN6, Neg. : Nominal Vref- Defined in ADCON2
			ADC_CH0_NEG_SAMPLEA_NVREF &
			ADC_CHX_POS_SAMPLEA_AN3AN4AN5 &
			ADC_CHX_NEG_SAMPLEA_NVREF ;		// (ADCHS)

	SetChanADC10(Channel);

	ConfigIntADC10(ADC_INT_ENABLE);	// Disable ADC interrupt
}
/******************************************************
 * Function: Init_UART
 * Description: UART1 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_UART(void)
{
   /* Holds the value of baud register */
   unsigned int baudvalue;
   /* Holds the value of uart config reg */
   unsigned int U1MODEvalue;
   /* Holds the information regarding uart
   TX & RX interrupt modes */
   unsigned int U1STAvalue;
   /* Turn off UART1module */
   CloseUART1();
   IPC2bits.U1TXIP = 1;
   IPC2bits.U1RXIP = 1;
   /* Configure uart1 receive and transmit interrupt */
   ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR6 &
   UART_TX_INT_DIS & UART_TX_INT_PR2);
   /* Setup the Buad Rate Generator */   //BaudRate 57600
   baudvalue = 21;         //UxBRG = ( (FCY/Desired Baud Rate)/16) ¡V 1
                     //UxBRG = ( (7372800*2/9600)/16-1) = 95
   /* Configure UART1 module to transmit 8 bit data with one stopbit.
   Also Enable loopback mode */
   U1MODEvalue = UART_EN & UART_IDLE_CON &
            UART_DIS_WAKE & UART_DIS_LOOPBACK &
            UART_DIS_ABAUD & UART_NO_PAR_8BIT &
            UART_1STOPBIT;
   U1STAvalue = UART_INT_TX_BUF_EMPTY &
            UART_TX_PIN_NORMAL &
            UART_TX_ENABLE & UART_INT_RX_CHAR &
            UART_ADR_DETECT_DIS &
            UART_RX_OVERRUN_CLEAR;
   OpenUART1(U1MODEvalue, U1STAvalue, baudvalue);

   return;

}
/******************************************************
 * Function: Init_UART2
 * Description: UART2 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_UART2(void)
{
   /* Holds the value of baud register */
   unsigned int baudvalue;
   /* Holds the value of uart config reg */
   unsigned int U2MODEvalue;
   /* Holds the information regarding uart
   TX & RX interrupt modes */
   unsigned int U2STAvalue;
   /* Turn off UART1module */
   CloseUART2();
   IPC6bits.U2TXIP = 1;
   IPC6bits.U2RXIP = 1;
   /* Configure uart1 receive and transmit interrupt */
   ConfigIntUART2(UART_RX_INT_EN & UART_RX_INT_PR6 &
   UART_TX_INT_DIS & UART_TX_INT_PR2);
   /* Setup the Buad Rate Generator */   //BaudRate 57600
   baudvalue = 130;         //UxBRG = ( (FCY/Desired Baud Rate)/16) ¡V 1
                     //UxBRG = ( (7372800*2/9600)/16-1) = 95
   /* Configure UART1 module to transmit 8 bit data with one stopbit.
   Also Enable loopback mode */
   U2MODEvalue = UART_EN & UART_IDLE_CON &
            UART_DIS_WAKE & UART_DIS_LOOPBACK &
            UART_DIS_ABAUD & UART_NO_PAR_8BIT &
            UART_1STOPBIT;
   U2STAvalue = UART_INT_TX_BUF_EMPTY &
            UART_TX_PIN_NORMAL &
            UART_TX_ENABLE & UART_INT_RX_CHAR &
            UART_ADR_DETECT_DIS &
            UART_RX_OVERRUN_CLEAR;
   OpenUART2(U2MODEvalue, U2STAvalue, baudvalue);

   return;

}
/******************************************************
 * Function: Init_SPI2
 * Description: SPI2 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Init_SPI2(void)
{
	unsigned int SPICONValue;
	unsigned int SPISTATValue;
	CloseSPI2();
	//ConfigIntSPI2(SPI_INT_DIS & SPI_INT_PRI_6);

	SPICONValue = FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT &
				  ENABLE_SDO_PIN & SPI_MODE16_ON &
				  SPI_SMP_ON & SPI_CKE_OFF &
				  SLAVE_ENABLE_OFF &
				  CLK_POL_ACTIVE_HIGH &
				  MASTER_ENABLE_ON &
				  SEC_PRESCAL_4_1 &
				  PRI_PRESCAL_16_1;
	SPISTATValue = SPI_ENABLE & SPI_IDLE_CON &
				   SPI_RX_OVFLOW_CLR;
	OpenSPI2(SPICONValue,SPISTATValue);
}

/******************************************************
 * Function: Timer5_Initial
 * Description: Timer5 initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void Timer5_Initial( void )
{
		ConfigIntTimer5( T5_INT_PRIOR_5 & T5_INT_ON ) ;
		OpenTimer5( T5_ON & T5_IDLE_STOP & T5_GATE_OFF & T5_PS_1_256 & T5_SOURCE_INT ,
					(long)(FCY/ 1000)* 500 / 256) ;
}
/******************************************************
 * Function: sys_Reset
 * Description: parameter initialization
 * Parameters: None
 * Return value: None
 *****************************************************/
void sys_Reset(void)
{
    APGS_initialize(); 
}
/******************************************************
 * Function: Ppathflow
 * Description: Parallel parking flow action control
 * Parameters: None
 * Return value: None
 *****************************************************/
void Ppathflow(void)
{
  real_T Rs;
  real_T L;
  real_T mcarAlpha;
  int16_T o_count;
  int16_T ckRltPos;
  real_T ddiff[100];
  boolean_T exitg1;
  int16_T pos_sign;
  uint16_T tmp;
  int16_T stepOneAngle;

      if (APGS_B.PparkingFlowFcn > ((uint16_T)0U)) {

        /*  ----- Initial Parameter ------------------- */

        o_count = (int16_T)0; // local view point count

        ckRltPos = (int16_T)0; //global view point count

        memset(&ddiff[0L], 0, ((uint16_T)100U) * (sizeof(real_T))); //Initial diff buffer for store path point

        // -------------Parallel check by Ultrasonic --------------------- ---//
        if ((((APGS_U.ultraSonic_FML < 100.0) || (APGS_U.ultraSonic_FMR < 100.0))
             && (APGS_U.ultraSonic_FML > 32.0)) && (APGS_U.ultraSonic_FMR > 32.0))
        { // Check wheather host car is parallel or not by using FML and FMR
          
          if (((fabs(APGS_U.ultraSonic_FML - APGS_U.ultraSonic_FMR) < 3.0) &&
               (((int16_T)APGS_DW.IsParallelOk) != ((int16_T)1))) &&
              (APGS_DW.yaw_odm > -5.0)) { // The distances deteced by FML and FMR are close to 3 cm

            APGS_DW.IsParallelOk = 1U; // Parallel parking Ok
          }

          if ((APGS_DW.yaw_odm >= -1.0) && (fabs(APGS_U.ultraSonic_FML -
                APGS_U.ultraSonic_FMR) > 20.0)) { // Check wheather host car is parallel or not by using yaw angle

            APGS_DW.IsParallelOk = 1U; // Parallel parking Ok
          }
        }

        if (((APGS_U.ultraSonic_FML == 30.0) && (APGS_U.ultraSonic_FMR == 30.0))
            && (APGS_DW.yaw_odm >= -1.0)) { // FML & FMR too close to front obstacle

          APGS_DW.IsParallelOk = 1U; // Parallel parking Ok
        }

        if ((((APGS_DW.turningStep != 0.0) && (APGS_DW.IsMultiTurn != 0.0)) &&
             ((APGS_U.ultraSonic_FML > 100.0) || (APGS_U.ultraSonic_FMR > 100.0)))
            && (APGS_DW.yaw_odm >= -1.0)) { // Check wheather host car is parallel or not by using yaw angle

          APGS_DW.IsParallelOk = 1U; // Parallel parking Ok
        }
        // -------------------------------------------------------------------- ---//

        // -------------Avoid right corner hit front car--------------------- ------------//
        if ((((((((APGS_DW.turningStep != 0.0) && (APGS_U.ultraSonic_FCL ==
                   250.0)) && (APGS_U.ultraSonic_FML == 250.0)) &&
                (APGS_U.ultraSonic_FMR == 250.0)) && (APGS_U.front_right <= 30.0))
              && (APGS_DW.turningStep != 0.0)) && (APGS_DW.IsMultiTurn == 0.0)) &&
            (APGS_DW.turningStep != 4.0)) { // Avoid right bumper hit front car

          APGS_DW.turningStep = 3.0; // Force steer command to zero
        }

        if (((APGS_DW.IsMultiTurn == 0.0) && (APGS_DW.turningStep == 3.0)) &&
            (APGS_U.front_right > 30.0)) { // Make sure only work one time 

          APGS_DW.turningStep = 0.0; 

          APGS_DW.multiShift = 0.0;          
        }
        // -------------------------------------------------------------------- ---//
        // -------------Multi-Turn strategy------------------------------ ------------//
        if (((((APGS_U.ultraSonic_RCL <= 30.0) || (APGS_U.ultraSonic_RML <= 30.0))
              || (APGS_U.ultraSonic_RMR <= 30.0)) || (APGS_U.ultraSonic_RCR <=
              30.0)) && (APGS_DW.multiShift == 0.0)) {
          /*  Objects detected from rear when backing */
          if (APGS_DW.turningStep == 0.0) {

            APGS_DW.IsMultiTurn = 0.0; //Two Turn

          } else {
            APGS_DW.IsMultiTurn = 1.0;// Multi-Turn 

          }

          if (APGS_U.gear_pos != ((uint16_T)81U)) {  // if gear mode is not R

            tmp = ((uint16_T)APGS_DW.steerTurnCnt) + ((uint8_T)1U); //Count for gear position change
            if (tmp > ((uint16_T)255U)) {
              tmp = (uint16_T)255U;
            }

            APGS_DW.steerTurnCnt = (uint8_T)tmp;

            /*  multi-turn counts in case of parking askew     */

            APGS_DW.multiShift = 1.0;

            /*  Set flg to forward */

            APGS_DW.MsgForUI = 6.0;
          }
        } else {
          if ((((((APGS_U.ultraSonic_FCL <= 30.0) || (APGS_U.ultraSonic_FML <=
                   30.0)) || (APGS_U.ultraSonic_FMR <= 30.0)) ||
                (APGS_U.ultraSonic_FCR <= 30.0)) && (APGS_DW.multiShift == 1.0))
              && (APGS_DW.turningStep != 3.0)) {
            /*  Objects detected from rear when direct */
            if (APGS_DW.turningStep == 0.0) {

              APGS_DW.IsMultiTurn = 0.0;
              /*  Two Turn */
            } else {

              APGS_DW.IsMultiTurn = 1.0;

              /*  Multi-Turn    */
            }

            if (APGS_U.gear_pos != ((uint16_T)119U)) { // if gear mode is not D

              tmp = ((uint16_T)APGS_DW.steerTurnCnt) + ((uint8_T)1U);
              if (tmp > ((uint16_T)255U)) {
                tmp = (uint16_T)255U;
              }

              APGS_DW.steerTurnCnt = (uint8_T)tmp;
              //  multi-turn counts in case of parking askew          

              APGS_DW.multiShift = 0.0;
              // Set flg to back 

              APGS_DW.MsgForUI = 7.0;

            }
          }
        }

        //-----------Machanism for hit road side -----------------
        if(APGS_DW.turningStep >= 1.0)
        {
            
            if (APGS_U.gear_pos == ((uint16_T)81U) && IsUnexpectChg_Parallel == 0)  // if gear mode is not R
            {  
                // Parknig gets into final circle, reverse status and two turn mode
                APGS_DW.IsMultiTurn = 1; //Force to multi-turn  
                APGS_DW.multiShift = 1.0;
                //Set flg to forward 
                APGS_DW.MsgForUI = 6.0;
                IsUnexpectChg_Parallel = 1;
            }

        }// End of Machanism for hit road side

        // -------------------------------------------------------------------- ---//
        // -------------------------Keep display in standby screen------------------ ---//       
        /*if ((APGS_DW.MsgForUI == 6.0) && (APGS_U.gear_pos == ((uint16_T)81U))) {

          //  Clear disply when get right position 
          APGS_DW.MsgForUI = 0.0;
        } else {
          if ((APGS_DW.MsgForUI == 7.0) && (APGS_U.gear_pos == ((uint16_T)119U)))
          {

            //Clear disply when get right position 
            APGS_DW.MsgForUI = 0.0;
          }
        }*/
        // -------------------------------------------------------------------- ---//
        //  --------------- Find point for steering change (Path Tracking)------------------------------ //
        if (APGS_DW.IsMultiTurn != 1.0) { // still in first turn

          stepOneAngle = (int16_T)0;
          exitg1 = (boolean_T)false;
          while ((!exitg1) && (stepOneAngle < 128)) {

            if (((fabs(APGS_B.mtp[((int16_T)1) + stepOneAngle] -
                       APGS_B.mtp[stepOneAngle]) <= 1.0) && (APGS_B.mtp[128 +
                  stepOneAngle] > APGS_DW.y_pos)) || (APGS_B.mtp[stepOneAngle] >
                 APGS_DW.x_pos)) { // Compare current poistion with the trajectory by planning

              ckRltPos = stepOneAngle; // Find the closest point in planning trajectory
              exitg1 = (boolean_T)true; // Jump out
            } else {

              stepOneAngle++;
            }
          }


          for (stepOneAngle = (int8_T)0; stepOneAngle <= ((int8_T)9);
               stepOneAngle++) {

            Rs = APGS_B.mtp[ckRltPos + stepOneAngle] - APGS_DW.x_pos;

            /*  expected path coordinate transform  shift */

            L = APGS_B.mtp[(ckRltPos + stepOneAngle) + 128] - APGS_DW.y_pos;

            /*  expected path coordinate transform shift */

            mcarAlpha = (Rs * cos(-APGS_DW.angle)) - (L * sin(-APGS_DW.angle));

            /*  expected path coordinate transform rotation */

            Rs = (Rs * sin(-APGS_DW.angle)) + (L * cos(-APGS_DW.angle));

            /*  expected path coordinate transform rotation       */

            ddiff[stepOneAngle] = fabs(sqrt((mcarAlpha * mcarAlpha) + (Rs * Rs))
              - 40.0); // 40.0 is length of head point


          }

          /*  ------ find the point that is the closest to the lookahead point on the path. */

          Rs = ddiff[(int8_T)0];

          for (stepOneAngle = (int8_T)0; stepOneAngle <= ((int8_T)9);
               stepOneAngle++) {

            if ((Rs > ddiff[stepOneAngle]) || (Rs == ddiff[stepOneAngle])) {

              Rs = ddiff[stepOneAngle];


              o_count = stepOneAngle + ((int16_T)1); 
            }


          }


          APGS_DW.IsMultiTurn = 0.0;


          APGS_DW.LHPResult = (real_T)((int16_T)(o_count + ckRltPos));// Current position of host car where  is closest to the path by planning 
        }

        // -------------------------------------------------------------------- ---//
        //  --------------- Final step check----------- ------------------------------ //
        if (APGS_DW.IsMultiTurn == 0.0) { // Two turn

          if ((APGS_DW.yaw_odm >= -1.0) && (APGS_DW.turningStep != 0.0)) {// is parallel

            APGS_DW.final_steer = 1.0;
            if (((APGS_U.SAngle > -10.0) && (APGS_U.SAngle < 10.0)) &&
                (APGS_DW.steer_cmd == 0L)) { // Make sure steer position is close to zero

              APGS_DW.MsgForUI = 8.0; // Display finish message

              APGS_DW.ActLevel = (uint16_T)5U; // Set parking level to final step
            }
          }
        } else { // Multi-Turn
          if (((APGS_DW.IsMultiTurn == 1.0) && (((int16_T)APGS_DW.IsParallelOk) ==
                ((int16_T)1))) && (APGS_DW.turningStep != 0.0)) {

            APGS_DW.turningStep = 2.0;


            APGS_DW.final_steer = 1.0;
            if (((APGS_U.SAngle > -10.0) && (APGS_U.SAngle < 10.0)) &&
                (APGS_DW.steer_cmd == 0L)) { // Make sure steer position is close to zero

              APGS_DW.MsgForUI = 8.0;  // Display finish message


              APGS_DW.ActLevel = (uint16_T)5U; // Set parking level to final step
            }
          }
        }


      }


}

/******************************************************
 * Function: MsgToFinish
 * Description: Send finish message to PC
 * Parameters: None
 * Return value: None
 *****************************************************/
void MsgToFinish(void)
{
    char tmpdata[] = {'f','i','n','i','s','h'}; 
    int i,j,iCnt;
    iCnt = 1; // Counts for repeat send check code
    for(j=0;j<iCnt;j++)
    {
        for(i=0;i<sizeof(tmpdata);i++)
        {
            WriteUART1(tmpdata[i]);
            while(BusyUART1());
        }
    }
}

/******************************************************
 * Function: SerialDataDecrypt
 * Description: Decrypt data which sent from PC
 * Return value: None
 *****************************************************/
void SerialDataDecrypt(void)
{
    if(urDataStep == 2) // Start receive data
    {
       if(urDataCnt < numOfURdata)
       {
           if(((urDataCnt + 2) % 2) == 0)
           {  
               tmpArrayUR[tmpUR_Cnt] = tmpURdata; //Lo 
           }
           else
           {
               tmpArrayUR[tmpUR_Cnt] |= tmpURdata << 8; //Hi
               tmpUR_Cnt++; // count for update tmpURdata
           }
           ur_ckSum = ur_ckSum + tmpURdata; // check sum calculation

       }
       


       urData[urDataCnt++] = tmpURdata; // data received from PC        
    }

    if(urDataStep == 1)
    { 
        numOfURdata = tmpURdata;
        urDataStep++;
    }


    if(tmpURdata == 53) // ascii 53 = 'S'
    {
        IsPCsendCMD = 1;
        urDataStep++;   // Get into 1 step
    }
    else if(tmpURdata == 69) // ascii 53 = 'E'
    {
        if(ur_ckSum == urData[urDataCnt-2]) //check sum validation  
        {     
            tmpArrayUR[0] =urData[0] | (urData[1] << 8); // Bug for tmpArrayUR[0] was being replaced by cksum
            urDataStep++;
        }
        else
        {
            urDataStep = 0; // Reset
            urDataCnt = 0; //  Reset
            ur_ckSum = 0;
        }
    }
    if(urDataStep==3) // finish
    {
        urDataStep = 0;
        urDataCnt = 0;
        ur_ckSum = 0;
        write_data_to_EEP();
        IsTrigEEpDataSave = 1;
    }

}
/******************************************************
 * Function:  Load_data_From_EE
 * Description: Initial to load data from dspic eeprom
 * Return value: None
 *****************************************************/
void Load_data_From_EE(void)
{
    read_data_From_EEP();
    //-------Parameters Assignment ----------- 
    APGS_EE.carH = (float)urDataInRAM[CARH]/1000;//m
    APGS_EE.carMSA = (float)urDataInRAM[CARMSA]/1000;
    APGS_EE.carRmin = (float)urDataInRAM[CARRMIN];
    APGS_EE.carTread = (float)urDataInRAM[CARTREAD]/1000;
    APGS_EE.carW = (float)urDataInRAM[CARW]/1000;
    APGS_EE.carl = (float)urDataInRAM[CARL]/1000;
    APGS_EE.carL = (float)urDataInRAM[CARLEN]/1000;
    APGS_EE.whsp_factor = (float)urDataInRAM[CARWHSP];
    APGS_EE.Lq = (float)urDataInRAM[CARLQ]/1000;
    APGS_EE.Lq_s = (float)urDataInRAM[CARLQS]/1000;
    APGS_EE.Lg1 = (float)urDataInRAM[CARLG1]/1000;
    APGS_EE.Lg2 = (float)urDataInRAM[CARLG2]/1000;
    APGS_EE.Lg3 = (float)urDataInRAM[CARLG3]/1000;
    APGS_EE.nC = (float)urDataInRAM[CARNC]/1000;
    APGS_EE.k = (float)urDataInRAM[CARK]/1000;
    APGS_EE.Lim_L_backIn = (float)urDataInRAM[CARLIM_L];
    APGS_EE.Lim_YawAngle_backIn = (float)urDataInRAM[CARYAW_BACK];
    APGS_EE.c0 = (float)urDataInRAM[CARC0]/100;//cm
    APGS_EE.g0 = (float)urDataInRAM[CARG0]/1000;
    APGS_EE.k1 = (float)urDataInRAM[CARK1]/1000;
    APGS_EE.pos_of_ultra = (float)urDataInRAM[CAR_POSOFULTRA];//cm
    APGS_EE.due_time_parking = urDataInRAM[DUE_TIME_PARKING];//cm
   APGS_EE.due_time_parking = APGS_EE.due_time_parking*750;
}
/******************************************************
 * Function:  closePeripherals
 * Description: Close peripherals for eeprom data saving
 * Return value: None
 *****************************************************/
void closePeripherals(void)
{
    // Disable Timer1
    ConfigIntTimer1( T1_INT_PRIOR_6 & T1_INT_OFF ) ;
    // Disable CAN1
  	ConfigIntCAN1(CAN_INDI_INVMESS_DIS & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS & CAN_INDI_TXB2_DIS & 
		      CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_DIS & CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,
		      CAN_INT_PRI_7 & CAN_INT_DISABLE); 	
    // Disable CAN2
	ConfigIntCAN2(CAN_INDI_INVMESS_DIS & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS & CAN_INDI_TXB2_DIS & 
		      CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_DIS & CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,
		      CAN_INT_PRI_7 & CAN_INT_DISABLE); 
}
/******************************************************
 * Function:  APGS_param_reset
 * Description: Reset variables
 * Return value: None
 *****************************************************/
void APGS_param_reset(void)
{
    APGS_initialize();
    diff_x_pos = 0;
    IsUnexpectChg_Parallel = 0;
    finish_Cnt = 0;
    APGS_U.edgDetStartFlg = 0;
}
/*History 
*  -------------------------------------------------------------
*  Add function APGS_param_reset in EXT_INT to reset paremeter for next parking
*  -------------------------------------------------------------
*  02/17/105 modified by Stanley Li
*  -------------------------------------------------------------
*  a) Verfiy the machanism of hit road side. (Success!!)
*  b) Display the tip of forward and back all the time in parallel mode
*  c) Add while out all of parking step in main for system repeat to park. 
*  d) Put 5 sec count to wait for finish message.
*  -------------------------------------------------------------
*  02/15/105 modified by Stanley Li
*  -------------------------------------------------------------
*  Add machanism for hit road side ,find the key word "road side"
*  --------------------------------------------------------------
*  01/11/105 modified by Stanley Li
*  
*  02/01/10 ceated by Stanley Li 
*/

