/******************************************************
 * File: APGS.c
 * Creator: Stanley Li
 * Description: Do the parallel and back-in action
 * Create Date: 2015/12/09 
 * Latest Date: 2015/12/09 
 * version: 1.0
 *****************************************************/


#include "APGS.h"
#include "APGS_private.h"

// Named constants for Chart: 'APGS_FlowAction'
#define APGS_IN_Action1                ((uint8_T)1U)
#define APGS_IN_Action2                ((uint8_T)2U)
#define APGS_IN_Action2_back_in        ((uint8_T)3U)
#define APGS_IN_Action3                ((uint8_T)4U)
#define APGS_IN_Action3_back_in        ((uint8_T)5U)
#define APGS_IN_Action4                ((uint8_T)6U)
#define APGS_IN_Action5                ((uint8_T)7U)
#define APGS_IN_NO_ACTIVE_CHILD        ((uint8_T)0U)
#define APGS_IN_Parameter              ((uint8_T)8U)

// Parameters for EEPROM
tag_EE_APGS APGS_EE;

// Block signals (auto storage) 
B_APGS_T APGS_B;

// Block states (auto storage) 
DW_APGS_T APGS_DW;

// External inputs (root inport signals with auto storage) 
ExtU_APGS_T APGS_U;

// Real-time model 
RT_MODEL_APGS_T APGS_M_;
RT_MODEL_APGS_T *const APGS_M = &APGS_M_;

// Parallel parking_n offset
real_T diff_x_pos;

extern float laser_front_right; // Declared variable from main_APGS.c
extern unsigned char IsUnexpectChg_Parallel; // Declared variable from main_APGS.c
extern unsigned char IsUnexpectChg_BackIn;// Declared variable from main_APGS.c

// Forward declaration for local functions 
static void APGS_validate_parkng_space(real_T parkmode);
static void APGS_get_second_pos(void);
static void APGS_edge1_Det(real_T x1, real_T x1_old, real_T b_y1, real_T y1_old,
  real_T right);
static void APGS_get_first_pos(void);
static void APGS_edge0_Det(real_T x1, real_T x1_old, real_T b_y1, real_T y1_old,
  real_T right, real_T parkMode);
static void APGS_loopForParkingAction(real_T x1, uint16_T gearPos, real_T
  parkmode);


/******************************************************
 * Function: APGS_validate_parkng_space
 * Description: Parking space validation
 * Parameters: 
 *                    parkmode - parking mode 
 * Return value: None
 *****************************************************/
static void APGS_validate_parkng_space(real_T parkmode)
{
  int16_T validation_space;
  real_T cnt;
  real_T tmp;
  int16_T max_pos;
  real_T tmp_max;
  real_T tmp_min;
  int16_T i;

  validation_space = (int16_T)0;

  // ------------Check Parking Mode --------------------------------- 
  if (parkmode == 1.0) {

    validation_space = 400; // Limit smallest parkin space for parallel mode
  } else {
    if (parkmode == 2.0) {

      validation_space = 150; // Limit smallest parkin space for back-in mode
    }
  }
  // ------------One Edge Mode, limit parking space --------------------------------- */
  if(APGS_DW.IsOneEdgeFound > 0)
  {
      if (parkmode == 1.0) {    
          APGS_DW.parking_space = 610;
      } else {
          if (parkmode == 2.0) {
             APGS_DW.parking_space = 350;
         }
      }
  }

  
  if (APGS_DW.parking_space > 610.0) {
    APGS_DW.parking_space = 610.0; // Limit max parking space 
  }

  if (APGS_DW.parking_space > ((real_T)validation_space)) { // Speace is big enough to park
    APGS_DW.vld_status = 0.0;


    cnt = 0.0;

    tmp = 0.0;

    validation_space = (int16_T)-1;

    max_pos = (int16_T)-1;

    tmp_max = 0.0;

    tmp_min = 100000.0;
    //-------------Calculate average m (Distance from right side of host car to left side of front car) ---------------------------------------- 

    for (i = (int8_T)0; i <= ((int8_T)29); i++) {

      if (APGS_DW.edg_info[(((int16_T)3) * i) + ((int16_T)1)] < 200.0) { // Only smaller than 200 cm will be recognized

        tmp += APGS_DW.edg_info[(((int16_T)3) * i) + ((int16_T)1)];

        cnt++;
        if (APGS_DW.edg_info[((int16_T)3) * i] > tmp_max) {

          tmp_max = APGS_DW.edg_info[((int16_T)3) * i];

          max_pos = i;
        }

        if (APGS_DW.edg_info[((int16_T)3) * i] < tmp_min) {

          tmp_min = APGS_DW.edg_info[((int16_T)3) * i];

          validation_space = i;
        }
      }


    }


    APGS_DW.angleOfSecCar = ((APGS_DW.edg_info[(((int16_T)3) * max_pos) +
      ((int16_T)2)] + APGS_DW.parking_m) - (APGS_DW.edg_info[(((int16_T)3) *
      validation_space) + ((int16_T)2)] + APGS_DW.parking_m)) /
      (APGS_DW.edg_info[((int16_T)3) * max_pos] - APGS_DW.edg_info[((int16_T)3) *
       validation_space]);
    APGS_DW.angleOfSecCar = atan(APGS_DW.angleOfSecCar); //get the angle of correspont to front car
    APGS_DW.parking_m = (tmp / cnt) / 100.0; //Get m

    //------------------------------------------------------------------------------

    APGS_DW.parking_edge_pos = APGS_DW.second_edg_pos; // Get the position of parking edge

    APGS_DW.IsSpaceOk = 1.0; // Set flag to ok

    APGS_DW.edg_flg = 8U; // Jump out the edge calculation logic
    
    if (parkmode == 2.0)
        APGS_B.BpathCalcFcn = APGS_DW.enable; // 
    else if (parkmode == 1)
        APGS_B.PpathCalcFcn = APGS_DW.enable;
       
    //  Jump to out of edge detection.  ##Only for now, just in case the false space detected which is caused by noise 
  } else {

    APGS_DW.edg_flg = 0U; // Go back to step if space is too small.
    // To find next parking space  
  }

  //----------Reset parameters ----------------------------------------------------
  memset(&APGS_DW.edg_info[0L], 0, ((uint16_T)90U) * (sizeof(real_T)));

  APGS_DW.edg1_cnt = 0U;

  APGS_DW.edg2_cnt = 0U;

  APGS_DW.ultra_edge2_cnt = (uint16_T)0U;

}

/******************************************************
 * Function: APGS_get_second_pos
 * Description: Get second edge position
 * Parameters: None
 * Return value: None
 *****************************************************/
static void APGS_get_second_pos(void)
{
  int16_T ix;
  real_T xbar;
  int16_T k;
  real_T r;
  real_T b_r;

  //-------------Find position where closest to the rear corner of front car-------------------------------//
  if (APGS_DW.edg_count < ((uint16_T)30U)) {
 
    APGS_DW.second_edg_pos = APGS_DW.edg_info[(((int16_T)APGS_DW.edg_count) -
      ((int16_T)1)) * ((int16_T)3)];
  } else {

    APGS_DW.second_edg_pos = APGS_DW.edg_info[(int8_T)0]; // because over 30
  }


  APGS_DW.parking_space = APGS_DW.second_edg_pos - APGS_DW.first_edg_pos; // Get parking space

  APGS_DW.edg_count = (uint16_T)1U; // Reset count

  APGS_DW.edg_flg = 4U; // Jump to next step 
}

/******************************************************
 * Function: APGS_edge1_Det
 * Description: Edge1 detection
 * Parameters: x1 -> n step of global x coordinate 
 *                    x1_old  -> n-1 step of global x coordinate 
 *                    b_y1  -> n step of global y coordinate
 *                    y1_old -> n-1 step of global ycoordinate
 *                    right -> Distance of front right side ultrasonic
 * Return value: None
 *****************************************************/
static void APGS_edge1_Det(real_T x1, real_T x1_old, real_T b_y1, real_T y1_old,
  real_T right)
{
  uint16_T tmp_count1;
  real_T a;
  real_T b_a;
  uint32_T tmp;

  a = x1 - x1_old;
  b_a = b_y1 - y1_old;
  //  Distance between current and last step  

  APGS_DW.edg_info[((int16_T)3) * (((int16_T)APGS_DW.edg_count) - ((int16_T)1))]
    = x1;  //  Store x poistion 

  APGS_DW.edg_info[((int16_T)2) + (((int16_T)3) * (((int16_T)APGS_DW.edg_count)
    - ((int16_T)1)))] = b_y1;   //Store y poistion 

  APGS_DW.edg_info[((int16_T)1) + (((int16_T)3) * (((int16_T)APGS_DW.edg_count)
    - ((int16_T)1)))] = right;  // Store right_side 

  if (sqrt((a * a) + (b_a * b_a)) > 0.0) { // Only true when car is moving
    tmp_count1 = APGS_DW.edg_count - ((uint16_T)1U); // Store last step
    if (tmp_count1 > APGS_DW.edg_count) { 
      tmp_count1 = (uint16_T)0U; // Avoid tmp_count1 over 30 
    }

    if (tmp_count1 == ((uint16_T)0U)) {
      tmp_count1 = (uint16_T)30U; // Avoid tmp_count1 over 30 
    }

    //---------------------Find the edge which dramatical change from far to small (ex: 250 to 100)
    if (((APGS_DW.edg_info[((((int16_T)APGS_DW.edg_count) - ((int16_T)1)) *
                            ((int16_T)3)) + ((int16_T)1)] <= 220.0) &&
         (APGS_DW.edg_info[((((int16_T)tmp_count1) - ((int16_T)1)) * ((int16_T)3))
          + ((int16_T)1)] >= 220.0)) && (((int16_T)APGS_DW.edg1_stage) ==
         ((int16_T)0))) {

      APGS_DW.edg1_stage = 1U; // move to count stage
    }

    /* --------Get into second stage----------------------------- */
    if (((int16_T)APGS_DW.edg1_stage) == ((int16_T)1)) {
      tmp = ((uint32_T)APGS_DW.ultra_edge2_cnt) + 1UL; // Count for slow down timer to save ultrasonic data    
      if (((int32_T)tmp) > 65535L) {
        tmp = 65535UL;
      }

      APGS_DW.ultra_edge2_cnt = (uint16_T)tmp;

       //--------Reset if small parking space---------------------------  
      if (((int16_T)APGS_DW.edg2_cnt) < ((int16_T)10)) {       

        APGS_DW.edg2_cnt = 0U;

        APGS_DW.edg_flg = 0U;

        memset(&APGS_DW.edg_info[0L], 0, ((uint16_T)90U) * (sizeof(real_T)));

        APGS_DW.edg_count = (uint16_T)1U;

        APGS_DW.first_edg_pos = 0.0;

        APGS_DW.second_edg_pos = 0.0;

        APGS_DW.parking_space = 0.0;

        APGS_DW.edg1_stage = 0U;

        APGS_DW.x_pos = 0.0;

        APGS_DW.y_pos = 0.0;

        APGS_DW.edg1_cnt = 0U;
      }
      //------------------------------------------------------------------

      
      if (APGS_DW.ultra_edge2_cnt == ((uint16_T)4U)) { // Count for slow down timer to save ultrasonic data   ,4 can be adjust see as what we need

        tmp = ((uint32_T)APGS_DW.int_count) + 1UL; //    Start count to 30 once edge 1 detected  
        if (((int32_T)tmp) > 65535L) {
          tmp = 65535UL;
        }

        APGS_DW.int_count = (uint16_T)tmp;

      }

      if (APGS_DW.int_count == ((uint16_T)30U)) { // Store 30 point into variable edge_Info

        APGS_DW.edg_flg = 3U; // Jump to next step

        APGS_DW.edg1_stage = 0U;
      }
    }

    if (((int16_T)APGS_DW.edg1_stage) == ((int16_T)1)) {

      if (APGS_DW.ultra_edge2_cnt == ((uint16_T)4U)) { 
        tmp = ((uint32_T)APGS_DW.edg_count) + 1UL;// Count for slow down timer to save ultrasonic data 
        if (((int32_T)tmp) > 65535L) {
          tmp = 65535UL;
        }

        APGS_DW.edg_count = (uint16_T)tmp;

        APGS_DW.ultra_edge2_cnt = (uint16_T)1U; // Reset to 1, once edge_info is updated
      }
    } else {

      tmp = ((uint32_T)APGS_DW.edg_count) + 1UL;
      if (((int32_T)tmp) > 65535L) {
        tmp = 65535UL;
      }

      APGS_DW.edg_count = (uint16_T)tmp;
    }

    //---------------Count for small space check------------------------------------- 
    if (((int16_T)APGS_DW.edg2_cnt) < ((int16_T)40)) { // 40 can be adjusted, see what we need

      tmp_count1 = ((uint16_T)APGS_DW.edg2_cnt) + ((uint8_T)1U);
      if (tmp_count1 > ((uint16_T)255U)) {
        tmp_count1 = (uint16_T)255U;
      }

      APGS_DW.edg2_cnt = (uint8_T)tmp_count1;
    }
    //-----------------------------------------------------------------------------

    if (APGS_DW.edg_count > ((uint16_T)30U)) {
      APGS_DW.edg_count = (uint16_T)1U; //Reset index to zero if over max range 30 
    }
  }

}

/******************************************************
 * Function: APGS_get_first_pos
 * Description: Get first edge position
 * Parameters: None
 * Return value: None
 *****************************************************/
static void APGS_get_first_pos(void)
{
  uint16_T tmp_indx;
  uint32_T tmp;

  //----------Find last 2 step position which stored in edg_info  (ex: edg_info[edg_count - 2])-----
  if (APGS_DW.edg_count <= ((uint16_T)2U)) {

    tmp = 30UL + ((uint32_T)APGS_DW.edg_count);
    if (((int32_T)tmp) > 65535L) {
      tmp = 65535UL;
    }

    tmp_indx = ((uint16_T)tmp) - ((uint16_T)2U);
  } else {

    tmp_indx = APGS_DW.edg_count - ((uint16_T)2U);
    if (tmp_indx > APGS_DW.edg_count) {
      tmp_indx = (uint16_T)0U;
    }
  }
  //---------------------------------------------------------

  APGS_DW.first_edg_pos = APGS_DW.edg_info[(((int16_T)tmp_indx) - ((int16_T)1)) *
    ((int16_T)3)]; // Get first edge position

  APGS_DW.first_edg_pos = APGS_DW.first_edg_pos - 70; // first edge correction  

  APGS_DW.int_count = (uint16_T)1U; // Reset

  APGS_DW.edg_flg = 2U; // Jump to find edge1
}

/******************************************************
 * Function: APGS_edge0_Det
 * Description: Edge0 detection
 * Parameters: x1 -> n step of global x coordinate 
 *                    x1_old  -> n-1 step of global x coordinate 
 *                    b_y1  -> n step of global y coordinate
 *                    y1_old -> n-1 step of global ycoordinate
 *                    right -> Distance of front right side ultrasonic
 *                    parkMode -> parallel or back-in mode (1 or 2)
 * Return value: None
 *****************************************************/
static void APGS_edge0_Det(real_T x1, real_T x1_old, real_T b_y1, real_T y1_old,
  real_T right, real_T parkMode)
{
  real_T D;
  uint16_T edg_ck_count;
  uint16_T one_edge_ck_count; 
  real_T b_a;
  int16_T exitg1;
  uint32_T tmp;
  uint16_T qY;
  uint16_T dis_ck;

  // Counts for the number of front right side ultrasonic information
  APGS_DW.int_count = (uint16_T)1U;

  D = x1 - x1_old; // x distance of vehicle moving in the time of ultrasonic information updated
  b_a = b_y1 - y1_old; // y distance of vehicle moving in the time of ultrasonic information updated
  D = sqrt((D * D) + (b_a * b_a)); // Distance of vehicle moving in the time of ultrasonic information updated

  // edg_info is a 30x3 matrix for storing x1, right ,D
  APGS_DW.edg_info[((int16_T)3) * (((int16_T)APGS_DW.edg_count) - ((int16_T)1))]
    = x1;
  APGS_DW.edg_info[((int16_T)1) + (((int16_T)3) * (((int16_T)APGS_DW.edg_count)
    - ((int16_T)1)))] = right;

  APGS_DW.edg_info[((int16_T)2) + (((int16_T)3) * (((int16_T)APGS_DW.edg_count)
    - ((int16_T)1)))] = D;

  APGS_DW.MsgForUI = parkMode; // Show the parking mode in the GUI
  //-------------One Edge check -------------------------------------------//
  if(right > 200 || APGS_DW.edg1_cnt < 4) // One edge detect, and use edg1_cnt to count short distance
  {
      APGS_DW.dis_For_OneEdge = APGS_DW.dis_For_OneEdge + D; // distance accumulate
      if(APGS_DW.dis_For_OneEdge > 10000) // Avoid the distance over 10000cm
      {
          APGS_DW.dis_For_OneEdge = 0;//Reset
          APGS_DW.dis_OverFlow_Cnt++;//Overflow count          
      }
     if(parkMode == 1)
         dis_ck = 611; // parallel parking space max limitation
     else if(parkMode == 2)
         dis_ck = 351; // back-In parking space max limitation
  
     //---------Firs Step is to check edge1 whether is exist?
     one_edge_ck_count = APGS_DW.edg_count-1;
     if(one_edge_ck_count == 0) // Pointthe count to max range 31, in case less than zero
         one_edge_ck_count = 30;
    
     /* ---------Firs Step is to check edge1 whether is exist? */
     if (((APGS_DW.edg_info[((((int16_T)APGS_DW.edg_count) - ((int16_T)1)) *
                             ((int16_T)3)) + ((int16_T)1)] <= 220.0) &&
          (APGS_DW.edg_info[((((int16_T)one_edge_ck_count) - ((int16_T)1)) * ((int16_T)3))
           + ((int16_T)1)] >= 220.0)) && (((int16_T)APGS_DW.edg1_stage) ==
          ((int16_T)0)) && (APGS_DW.dis_For_OneEdge > dis_ck || APGS_DW.dis_OverFlow_Cnt > 0 )) {
         APGS_DW.edg1_stage  = 1; // Make sure get into edge1
         APGS_DW.edg_flg = 2; // Jump to edge1 step
         APGS_DW.edg2_cnt = 11; // Make sure get into edge1, not noise
         APGS_DW.IsOneEdgeFound = 1;
         return;
     }
  
  }
  //-------------------------------------------------------------------------

  // Make sure first car is being deteced by ultrasonic
  if ((right < 200.0) && (((int16_T)APGS_DW.edg1_cnt) <= ((int16_T)5))) {

    edg_ck_count = ((uint16_T)APGS_DW.edg1_cnt) + ((uint8_T)1U);
    if (edg_ck_count > ((uint16_T)255U)) {
      edg_ck_count = (uint16_T)255U; // avoid edg_ck_count > 255
    }

    APGS_DW.edg1_cnt = (uint8_T)edg_ck_count; // update edg1_cnt
  }

   
  if ((D > 0.0) && (((int16_T)APGS_DW.edg1_cnt) >= ((int16_T)5))) {
    // Get into the logic if current right distance > 250 
    if ((APGS_DW.edg_info[((((int16_T)APGS_DW.edg_count) - ((int16_T)1)) *
                           ((int16_T)3)) + ((int16_T)1)] >= 250.0) && (((int16_T)
          APGS_DW.edg_flg) == ((int16_T)0))) {

      edg_ck_count = APGS_DW.edg_count;
      do {
        exitg1 = (int16_T)0;

        /*  find first edge */

        qY = edg_ck_count - ((uint16_T)1U); // Back counts to find edge0
        if (qY > edg_ck_count) {
          qY = (uint16_T)0U;
        }

        edg_ck_count = qY;

        if (qY == ((uint16_T)0U)) {
          //In case qY smaller than zero 
          edg_ck_count = (uint16_T)30U;
        }

        if (APGS_DW.edg_info[((((int16_T)edg_ck_count) - ((int16_T)1)) *
                              ((int16_T)3)) + ((int16_T)1)] <= 250.0) { //First edge being found 

          APGS_DW.edg_flg = 1U; //  set flag to edge 1 

          if ((APGS_DW.int_count >= ((uint16_T)30U)) || (APGS_DW.edg_info
               [((((int16_T)edg_ck_count) - ((int16_T)1)) * ((int16_T)3)) +
               ((int16_T)2)] == 0.0)) {
            exitg1 = (int16_T)1; //  Jump out of while if index over than max range 30
          } else {
            tmp = ((uint32_T)APGS_DW.int_count) + 1UL; // Store information into buffer edg_info
            if (((int32_T)tmp) > 65535L) {
              tmp = 65535UL;
            }

            APGS_DW.int_count = (uint16_T)tmp;
          }
        } else {
          exitg1 = (int16_T)1;
        }
      } while (exitg1 == ((int16_T)0));

      // end while 
    }

    /*tmp = ((uint32_T)APGS_DW.edg_count) + 1UL; //Count for update buffer edg_info
    if (((int32_T)tmp) > 65535L) {
      tmp = 65535UL;
    }

    APGS_DW.edg_count = (uint16_T)tmp;


    if (APGS_DW.edg_count > ((uint16_T)30U)) {
      //  Reset index to zero if over max range 31 
      APGS_DW.edg_count = (uint16_T)1U;
    }*/
  }
  tmp = ((uint32_T)APGS_DW.edg_count) + 1UL; //Count for update buffer edg_info
    if (((int32_T)tmp) > 65535L) {
      tmp = 65535UL;
    }

    APGS_DW.edg_count = (uint16_T)tmp;


    if (APGS_DW.edg_count > ((uint16_T)30U)) {
      //  Reset index to zero if over max range 31 
      APGS_DW.edg_count = (uint16_T)1U;
    }

}

/******************************************************
 * Function: APGS_loopForParkingAction
 * Description: Make sure host car has enough distance to park into the space
 * Parameters: x1 -> n step of global x coordinate 
 *                   gearPos  -> Gear position (N D R)
 *                    parkMode -> parallel or back-in mode (1 or 2)
 * Return value: None
 *****************************************************/
static void APGS_loopForParkingAction(real_T x1, uint16_T gearPos, real_T
  parkmode)
{
  real_T legal_parking_n;

  if (parkmode == 1.0) { // parallel mode

    /*legal_parking_n = (((((APGS_DW.parking_space / 100.0) - 5.2) * APGS_EE.g0) + 1.0) *
                       ((APGS_DW.parking_m - 0.5) * APGS_EE.k1)) + (((APGS_DW.car_Speed
      * 0.15) + APGS_EE.c0) + (APGS_DW.parking_m - 0.5)); // Calculation the distance which is far enough to park
    */
    legal_parking_n = APGS_EE.c0 + diff_x_pos + APGS_DW.car_Speed*0.15 ;
  } else { // back-In mode

    legal_parking_n = APGS_DW.backIn_backN; // Get from back-In path planning
  }

  APGS_DW.parking_n = ((x1 - APGS_DW.parking_edge_pos) - APGS_EE.pos_of_ultra) / 100.0; // To calculate related position from host car to front car

  //-----------Display parking space found -------------------------------
  if (APGS_DW.parking_n > legal_parking_n) {

    APGS_DW.vld_status = 1.0;
    if (parkmode == 1.0) {

      APGS_DW.MsgForUI = 4.0;
    } else {

      APGS_DW.MsgForUI = 4.0;
    }
  }
 //-------------------------------------------------------------------
 
  if ((APGS_DW.vld_status == 1.0) && (gearPos == ((uint16_T)119U))) { // Waiting for reverse 

    APGS_DW.edg_flg = 8U; //  Avoid fales action  

    if (parkmode == 1.0) {

      APGS_DW.ActLevel = (uint16_T)4U; // Log in EPS, and keep steer command at zero 
      if (APGS_DW.parking_n < legal_parking_n) { // Start Parallel Action

        APGS_DW.IsSteerZeroCmd = 1.0; 

        APGS_DW.ActLevel = (uint16_T)3U; // Jump to parallel path calculation
        
        APGS_DW.MsgForUI = 5.0; // Clear UI
      }
    } else {
      if (parkmode == 2.0) {

        APGS_DW.ActLevel = (uint16_T)4U;// Log in EPS, and keep steer command at zero 
        if (APGS_DW.parking_n < APGS_DW.backIn_backN) { // Start back-In Action

          APGS_DW.IsSteerZeroCmd = 1.0;

          APGS_DW.ActLevel = (uint16_T)6U; //Jump to park level

          APGS_DW.MsgForUI = 5.0; // Clear UI
        }
      }
    }
  }
}

real_T rt_roundd(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/******************************************************
 * Function: APGS_step
 * Description: main parking tracking control
 * Parameters: None
 * Return value: None
 *****************************************************/

void APGS_step(void)
{
  real_T Rs;
  real_T L;
  int16_T stepOneAngle;
  real_T mcarPHIS;
  real_T mcarAlpha;
  real_T mT2;
  real_T mscan;
  real_T mcot_Phi2;
  real_T mcot_op_m2;
  real_T msin_op2;
  real_T mg;
  real_T L_theta;
  real_T mTurn_cmd2;
  real_T mPHI;
  int16_T o_count;
  int16_T ckRltPos;
  real_T ddiff[100];
  boolean_T exitg1;
  uint16_T tmp;
  real_T min_Ld;
  real_T min_Lx;
  real_T min_Ly;



  // ---------TaskScheduler ------------------------------------------
  if (((int16_T)APGS_DW.temporalCounter_i1) < ((int16_T)1)) {
    APGS_DW.temporalCounter_i1 = (uint8_T)((int16_T)(((int16_T)
      APGS_DW.temporalCounter_i1) + ((int16_T)1)));
  }

  if (APGS_DW.is_active_c3_APGS == ((uint8_T)0U)) {
    // Entry: SchedulerAPGS/TaskScheduler 
    APGS_DW.is_active_c3_APGS = 1U;

    //Entry Internal: SchedulerAPGS/TaskScheduler 

    APGS_DW.is_active_shcheduler = 1U;
    APGS_DW.temporalCounter_i1 = 0U;
  } else {

    if (((int16_T)APGS_DW.temporalCounter_i1) == ((int16_T)1)) {
      //---------------------------------------------------------------------------//
      //------------APGS_FlowAction ------------------------------------------------//
      //---------------------------------------------------------------------------//
      if (APGS_DW.is_active_c1_APGS == ((uint8_T)0U)) {
        // Entry: SchedulerAPGS/mainTask_function/AGPS_Action/APGS_FlowAction 
        APGS_DW.is_active_c1_APGS = 1U;

        //Entry Internal: SchedulerAPGS/mainTask_function/AGPS_Action/APGS_FlowAction 
        APGS_DW.is_c1_APGS = APGS_IN_Parameter;

        //--------- Entry 'Parameter-------------------------------------------------
        APGS_DW.waitingMode = (uint16_T)1U;
        APGS_DW.edgDetMode = (uint16_T)2U;
        APGS_DW.pathCalcMode = (uint16_T)3U;
        APGS_DW.steerMode = (uint16_T)4U;
        APGS_DW.APGS_FINISH = (uint16_T)5U;
        APGS_DW.parkingMode = (uint16_T)6U;
        APGS_DW.enable = (uint16_T)1U;
        APGS_DW.disable = (uint16_T)0U;
        APGS_DW.ActLevel = APGS_DW.waitingMode;
        APGS_B.PpathCalcFcn = APGS_DW.disable;
        APGS_B.BpathCalcFcn = APGS_DW.disable;
        APGS_B.edgDetFcn = APGS_DW.disable;
        APGS_B.PparkingFlowFcn = APGS_DW.disable;
        APGS_B.BparkingFlowFcn = APGS_DW.disable;
      } else {
        switch (APGS_DW.is_c1_APGS) {
         case APGS_IN_Action1:
          if ((APGS_DW.ActLevel == APGS_DW.steerMode) && (APGS_U.parkMode == 1.0))
          { // parallel mode

            APGS_DW.is_c1_APGS = APGS_IN_Action2;

            APGS_B.steerCtlParkingFcn = APGS_DW.enable; // EPS Login enable
          } else {
            if ((APGS_DW.ActLevel == APGS_DW.steerMode) && (APGS_U.parkMode ==
                 2.0)) {// backIn mode

              APGS_DW.is_c1_APGS = APGS_IN_Action2_back_in;

              APGS_B.steerCtlParkingFcn = APGS_DW.enable;// EPS Login enable
              APGS_B.BpathCalcFcn = APGS_DW.enable; // Back In path calculation enable
            }
          }
          break;

         case APGS_IN_Action2:

          if (APGS_DW.ActLevel == APGS_DW.pathCalcMode) { // Reset x/y after path planning finished

            APGS_DW.is_c1_APGS = APGS_IN_Action3;

            APGS_DW.x_pos = 0.0; //reset x
            APGS_DW.y_pos = 0.0; // reset y
            APGS_B.edgDetFcn = APGS_DW.disable;// Disable parking space scan
            APGS_DW.ActLevel = (uint16_T)6U; // Start parking
            //APGS_B.PpathCalcFcn = APGS_DW.enable; // Parallel path planning enable 
          }
          break;

         case APGS_IN_Action2_back_in:
 
          if (APGS_DW.ActLevel == APGS_DW.parkingMode) { // Back-in parking start

            APGS_DW.is_c1_APGS = APGS_IN_Action3_back_in;

            APGS_DW.x_pos = 0.0; //reset x
            APGS_DW.y_pos = 0.0; // reset y
            APGS_B.edgDetFcn = APGS_DW.disable; // Disable parking space scan 
            APGS_B.BpathCalcFcn = APGS_DW.disable; // Back In path calculation disable
            APGS_B.BparkingFlowFcn = APGS_DW.enable;  // Enable back In parking flow control
          }
          break;

         case APGS_IN_Action3:

          if (APGS_DW.ActLevel == APGS_DW.parkingMode) { // Parallel parking start

            APGS_DW.is_c1_APGS = APGS_IN_Action4;

            APGS_B.PpathCalcFcn = APGS_DW.disable; // Parallel path calculation disable
            APGS_B.PparkingFlowFcn = APGS_DW.enable;// Enable Parallel parking flow control
          }
          break;

         case APGS_IN_Action3_back_in:

          if (APGS_DW.ActLevel == APGS_DW.APGS_FINISH) { // APGS finish

            APGS_DW.is_c1_APGS = APGS_IN_Action5;

            APGS_B.steerCtlParkingFcn = APGS_DW.disable; // EPS Log out
          }
          break;

         case APGS_IN_Action4:

          if (APGS_DW.ActLevel == APGS_DW.APGS_FINISH) { // APGS finish

            APGS_DW.is_c1_APGS = APGS_IN_Action5;

            APGS_B.steerCtlParkingFcn = APGS_DW.disable; // EPS Log out
          }
          break;

         case APGS_IN_Action5:

          break;

         default:  
          if ((APGS_U.edgDetStartFlg == APGS_DW.enable) && (APGS_DW.ActLevel ==
               APGS_DW.waitingMode)) {

            APGS_DW.is_c1_APGS = APGS_IN_Action1;

            APGS_DW.ActLevel = APGS_DW.edgDetMode; // Start parking space scan
            APGS_B.edgDetFcn = APGS_DW.enable; // parking space scan function enable
          }
          break;
        }
      }

      // End of APGS_FlowAction ------------------------------------------------------

      //----------------------------------------------------------------------
      //------------CalParkingPath_BackIn: BackIn path planing----------------------
      //----------------------------------------------------------------------
      if (APGS_B.BpathCalcFcn > ((uint16_T)0U)) { // Back-In path Fcn is triggered 

        Rs = APGS_EE.carRmin - APGS_EE.carW;

        L = Rs - APGS_DW.parking_m;

        APGS_DW.backIn_backN = (sqrt((Rs * Rs) - (L * L)) + 0.35) +
          (APGS_DW.car_Speed * 0.05); // Compensate error by using car speed , 0.35 is the value to estimate error
      }

      //----------------------------------------------------------------------
      //------------End of CalParkingPath_BackIn: BackIn path planing----------------------
      //----------------------------------------------------------------------

	   
      //------------------------------------------------------------------------  
      //-------------------Flow Control for Parking space detection--------------------
      //------------------------------------------------------------------------  
      if ((APGS_B.edgDetFcn != ((uint16_T)0U)) && (APGS_U.update_Ultra != 0.0)) // Start to find edge if edgDetFcn was enabled and ultrasonic was updated
      {
        //n step of global x coordinate of the vehicle
        Rs = APGS_DW.x_pos;

        //n step of global y coordinate of the vehicle
        L = APGS_DW.y_pos;

        if (((int16_T)APGS_DW.edg_flg) == ((int16_T)0)) {
          //edge0 detection if edg_flg == 0
          APGS_edge0_Det(APGS_DW.x_pos, APGS_DW.Delay_DSTATE, APGS_DW.y_pos,
                         APGS_DW.Delay1_DSTATE, APGS_U.front_right,
                         APGS_U.parkMode);

        } else {     
        
          if (((int16_T)APGS_DW.edg_flg) == ((int16_T)1)) {
            //To get first edge position  if edg_flg == 1
            APGS_get_first_pos();
			
          } else {   
          
            if (((int16_T)APGS_DW.edg_flg) == ((int16_T)2)) {
              //edge1 detection  if edg_flg == 2
              APGS_edge1_Det(APGS_DW.x_pos, APGS_DW.Delay_DSTATE, APGS_DW.y_pos,
                             APGS_DW.Delay1_DSTATE, APGS_U.front_right);

            } else {

              if (((int16_T)APGS_DW.edg_flg) == ((int16_T)3)) {
                //To get second edge position  if edg_flg == 3
                APGS_get_second_pos();

              } else {

                if (((int16_T)APGS_DW.edg_flg) == ((int16_T)4)) {
                  //Parking space validation  if edg_flg == 4
                  APGS_validate_parkng_space(APGS_U.parkMode);

                } else {
 
                }
              }
            }
          }
        }
        APGS_U.update_Ultra = 0; // Reset ultrasoic update flag
      
        if (APGS_DW.IsSpaceOk == 1.0) {
          // To check parking_n is enough to do parallel or back-in parking
          APGS_loopForParkingAction(APGS_DW.x_pos, APGS_U.gear_pos,
            APGS_U.parkMode);

        } else {

        }
        // End of loopForParking

        //n-1 step of global x coordinate of the vehicle
        APGS_DW.Delay_DSTATE = Rs;
        //n-1 step of global y coordinate of the vehicle
        APGS_DW.Delay1_DSTATE = L;
      }

      //------------------------------------------------------------------------  //
      //-------------------End of flow Control for Parking space detection---------------//
      //------------------------------------------------------------------------//

      //------------------------------------------------------------------------  //
      //-------------------BparkingFow: Back-In parking flow-------------------------//
      //------------------------------------------------------------------------  //
      if (APGS_B.BparkingFlowFcn > ((uint16_T)0U)) { // Back In function is triggered

        if (APGS_DW.parking_space > 330.0) {
          stepOneAngle = (int16_T)-80; // Stop angle for step 1 - Big space
        } else {
          stepOneAngle = (int16_T)-70; // Stop angle for step 1 - small space
        }

        Rs = APGS_DW.parking_m * 100.0; // m -> cm
        
        if (APGS_DW.y_pos > (Rs + 350.0)) {
          //---------------------- Final correction     
          // ------Correct yaw_odm to 90 degree ----------------------------  
          if ((APGS_DW.backIn_step == 8.0) || (APGS_DW.backIn_step == 7.0)) {

            if ((APGS_DW.yaw_odm > -92.0) && (APGS_DW.yaw_odm < -88.0)) {

              APGS_DW.backIn_step = 11.0; // case for unexpect
            } else {

              APGS_DW.backIn_step = 7.0; // case for yaw angle correction
            }

            if ((APGS_U.back_right > 200.0) || (APGS_DW.y_pos > (Rs + 480.0))) {

              APGS_DW.MsgForUI = 6.0;

              //  For Big space, just in case the rear is over back.

              APGS_DW.multiShift = 1.0;
            }

            if (APGS_U.front_right > 200.0) { //  Go back if forward too much 

              APGS_DW.MsgForUI = 7.0;

              APGS_DW.multiShift = 0.0;

            }
          }

          if (APGS_DW.backIn_step > 8.0) { //  make sure do the right action 

            if ((APGS_DW.y_pos < (Rs + 480.0)) && ((APGS_DW.multiShift == 0.0) ||
                 (APGS_DW.backIn_step == 11.0))) {

              APGS_DW.MsgForUI = 7.0;

              //  For Big space, just in case the over forward

              APGS_DW.multiShift = 0.0;
            } else {
  
              APGS_DW.MsgForUI = 6.0;

              //  For Big space, just in case the rear is over back. 
              APGS_DW.multiShift = 1.0;
            }

            if (APGS_U.front_right > APGS_U.back_right) {
              APGS_DW.backIn_step = 9.0;// counterclockwise if front_right > back_right
            } else {
              APGS_DW.backIn_step = 10.0; //clockwise
            }
          }
        }

        //  ------------------ unexpect case for big space ----------------------------------------
        if (((((APGS_DW.backIn_step == 7.0) && (APGS_U.front_right > 140.0)) &&
              (APGS_U.back_right > 140.0)) && (APGS_DW.yaw_odm > -92.0)) &&
            (APGS_DW.yaw_odm < -88.0)) {
          APGS_DW.backIn_step = 4.0; // Direct set to finish
        }

        //  ----In case of correct too much -----------------------------------------------
        if ((APGS_DW.backIn_step == 8.0) && ((APGS_DW.yaw_odm < -100.0) ||
             (APGS_DW.yaw_odm > -80.0))) {
          APGS_DW.backIn_step = 7.0;
        }

        //   -- -Forward if detect obstacle---------------------------- --------------------
        if (((APGS_U.ultraSonic_RML <= 30.0) || (APGS_U.ultraSonic_RMR <= 30.0))
            && (((APGS_DW.backIn_step == 9.0) || (APGS_DW.backIn_step == 10.0)) ||
                (APGS_DW.backIn_step == 7.0))) {

          APGS_DW.MsgForUI = 6.0;

          APGS_DW.multiShift = 1.0;
        }

        // -------- back if detect obstacle---------------------------------------------
        if ((APGS_U.ultraSonic_FMR <= 30.0) || (APGS_U.ultraSonic_FML <= 30.0))
        {

          APGS_DW.MsgForUI = 7.0;

          APGS_DW.backIn_step = 2.0;
        }

        //  -- inform driver back if front right ultra can not detect anything  -----------------
        if (((((APGS_DW.backIn_step == 9.0) || (APGS_DW.backIn_step == 10.0)) ||
              (APGS_DW.backIn_step == 7.0)) && (APGS_DW.MsgForUI == 6.0)) &&
            (APGS_U.front_right > 150.0)) {

          APGS_DW.MsgForUI = 7.0;

          APGS_DW.multiShift = 0.0;
        }

        if (((APGS_DW.backIn_step == 9.0) || (APGS_DW.backIn_step == 10.0)) &&
            (fabs(APGS_U.back_right - APGS_U.front_right) < 8.0)) { // final parallel check by using ultrasonic

          APGS_DW.backIn_step = 4.0; // Jump to finish
        }

        if ((APGS_DW.backIn_step == 6.0) && (APGS_DW.y_pos > (Rs + 350.0))) { // For big space
          APGS_DW.backIn_step = 4.0; // Jump to finish
        }

        if (((APGS_DW.backIn_step == 4.0) && (APGS_U.SAngle > -10.0)) &&
            (APGS_U.SAngle < 10.0)) {

          //  make sure steer is reset to zero

          if(APGS_DW.y_pos > (Rs + 400.0)) // Make sure host car park park depth enough into parking space
          {
              APGS_DW.ActLevel = (uint16_T)5U; // Set level to final

              APGS_DW.MsgForUI = 8.0; // Display finish message
          }else {
              APGS_DW.MsgForUI = 7.0; // prompt back message because host car is not depth enough to finish
          }
        }

        // ------ Forced Forward in case left range is too big.--------------- 
        if ((APGS_DW.backIn_step == 0.0) && (APGS_DW.y_pos > (Rs + 150.0))) { // 150 can be adjusted

          APGS_DW.MsgForUI = 6.0;

          APGS_DW.multiShift = 1.0;

          APGS_DW.backIn_step = 1.0;
 

        }

        //-------------- Objects detected from rear when backing -------------------------------- 
        if (((((((APGS_U.ultraSonic_RCL <= 30.0) || (APGS_U.ultraSonic_RML <=
                  30.0)) || (APGS_U.ultraSonic_RMR <= 30.0)) ||
               (APGS_U.ultraSonic_RCR <= 30.0)) && (APGS_DW.multiShift == 0.0)) &&
             (APGS_DW.backIn_step == 0.0)) && (APGS_U.gear_pos != ((uint16_T)81U)))
        { 
          APGS_DW.multiShift = 1.0;

          // Set flg to forward 
          APGS_DW.MsgForUI = 6.0;

          APGS_DW.backIn_step = 1.0;          

        }

        // ---------------Make the host car back in with right yaw angle------------------------- 
        if ((APGS_DW.backIn_step == 1.0) && (APGS_DW.yaw_odm < ((real_T)
              stepOneAngle))) {

          APGS_DW.MsgForUI = 7.0;

          APGS_DW.multiShift = 0.0;

          APGS_DW.backIn_step = 2.0;
        }

        // ------------To keep car close to right side--------------------------------- --------
        if ((((APGS_DW.backIn_step == 2.0) && (APGS_DW.y_pos > (Rs + 130.0))) &&
             (APGS_DW.yaw_odm > -92.0)) && (APGS_DW.yaw_odm < -88.0)) {

          if (APGS_U.back_right <= 105.0) {

            APGS_DW.backIn_step = 8.0; // Jump to step 8 to keep car close to right side

            APGS_DW.MsgForUI = 7.0;
          } else {

            APGS_DW.backIn_step = 6.0;
          }
        }
        
        // -----------In case of hitting right side----------------------------- -------------
        if (((APGS_U.gear_pos == 119)&&((APGS_U.back_right <= 32.0)) &&
             (APGS_DW.backIn_step > 1.0)) && (APGS_DW.backIn_step != 3.0)) {

          APGS_DW.backIn_tmp_yaw_odm = APGS_DW.yaw_odm; // save current yaw angle

          APGS_DW.backIn_step = 3.0;

          APGS_DW.MsgForUI = 6.0;

          APGS_DW.multiShift = 1.0;

          //  Set flg to forward
        }
        unsigned short hitLim = 0; // Declare the limitation
        hitLim = (APGS_DW.parking_space - APGS_EE.carW*100)/2 - 28; // Calculae hit limitation 
        if(hitLim < 32)
            hitLim = 32; // min is 32 cm
        if(hitLim > 50)
            hitLim = 50; // max is 50cm
        // -----------In case of hitting left side----------------------------- -------------
        if (((APGS_U.gear_pos == 119)&&((APGS_U.back_left <= hitLim) ) &&
             (APGS_DW.backIn_step > 1.0)) && (APGS_DW.backIn_step != 5.0)) {

          APGS_DW.backIn_tmp_yaw_odm = APGS_DW.yaw_odm; // save current yaw angle

          APGS_DW.backIn_step = 5.0;

          APGS_DW.MsgForUI = 6.0;

          APGS_DW.multiShift = 1.0;

          //  Set flg to forward 
        }
        // ------------------ Special action for step 5 and 3,  --------------------------------
        if (((APGS_DW.backIn_step == 5.0) || (APGS_DW.backIn_step == 3.0)) &&
            (APGS_U.gear_pos == ((uint16_T)81U))) {

          if (APGS_DW.multiShift == 1.0) {

            APGS_DW.backIn_tmp_xpos = APGS_DW.x_pos; // save current x 

            APGS_DW.backIn_tmp_ypos = APGS_DW.y_pos; // save current y

            APGS_DW.multiShift = 2.0;
          }

          L = APGS_DW.x_pos - APGS_DW.backIn_tmp_xpos;
          Rs = APGS_DW.y_pos - APGS_DW.backIn_tmp_ypos;
          if ((sqrt((L * L) + (Rs * Rs)) > APGS_EE.Lim_L_backIn) && (APGS_DW.y_pos < (APGS_DW.parking_m*100 + 150))) {  // go straight about 30 cm

            APGS_DW.backIn_IsOkToChg = 1.0; // Finish Straight line 
          }
        }

        //---------Step 3 Continue ---Stop if yaw angle is 8 degree bigger than  backIn_tmp_yaw_odm
        if ((APGS_DW.backIn_step == 3.0) && (APGS_DW.yaw_odm >
             (APGS_DW.backIn_tmp_yaw_odm + APGS_EE.Lim_YawAngle_backIn))) {

          APGS_DW.backIn_step = 2.0;

          APGS_DW.MsgForUI = 7.0;

          APGS_DW.backIn_IsOkToChg = 0.0;

          APGS_DW.multiShift = 0.0;
        }
        //---------Step 5 Continue ---Stop if yaw angle is 8 degree bigger than  backIn_tmp_yaw_odm
        if ((APGS_DW.backIn_step == 5.0) && (APGS_DW.yaw_odm <
             (APGS_DW.backIn_tmp_yaw_odm - APGS_EE.Lim_YawAngle_backIn))) {

          APGS_DW.backIn_step = 2.0;

          APGS_DW.MsgForUI = 7.0;

          APGS_DW.backIn_IsOkToChg = 0.0;

          APGS_DW.multiShift = 0.0;
        }

        //--------Gear position unexcept change -----------------------
        if(APGS_DW.backIn_step >=7)
        {

            if(APGS_U.gear_pos==119 && IsUnexpectChg_BackIn == 1)
                IsUnexpectChg_BackIn = 0; //reset

            if(APGS_DW.multiShift == 0)
            {
               if(APGS_U.gear_pos==81 && IsUnexpectChg_BackIn == 0)
               {
               // Set flg to forward 
                   APGS_DW.MsgForUI = 6.0;  
                   APGS_DW.multiShift = 1; 
                   IsUnexpectChg_BackIn = 1;
               }
            }
     
        }


      }
      //------------------------------------------------------------------------  //
      //-------------------End of BparkingFow: Back-In parking flow-------------------//
      //------------------------------------------------------------------------  //

      //----------------------------------------------------------------// 
      //CalParkingPath_Parallel: Parallel path planning--------------------------//
      //----------------------------------------------------------------// 
      if (APGS_B.PpathCalcFcn > ((uint16_T)0U)) { // Parallel Path calculation Fcn is triggered

        Rs = (((((((APGS_EE.carH + 1.7) + APGS_P.carb0) * ((APGS_EE.carH + 1.7) +
                   APGS_P.carb0)) - (((0.5 + APGS_P.carD) - APGS_P.carb1) * (2.0
                   * APGS_EE.carRmin))) + (((0.5 + APGS_P.carD) - APGS_P.carb1) *
                 ((0.5 + APGS_P.carD) - APGS_P.carb1))) / 2.0) / ((0.5 +
                APGS_P.carD) - APGS_P.carb1)) + (APGS_EE.carW / 2.0);

        mcarPHIS = (((((5.2 - (APGS_DW.parking_space / 100.0)) * APGS_EE.k) + APGS_EE.nC) *
                     3.1415926535897931) / 180.0) + atan(APGS_EE.carl /
          ((APGS_EE.carTread / 2.0) + Rs)); //steering angle


        mcarAlpha = asin(((APGS_EE.carH + 1.7) + APGS_P.carb0) / (APGS_EE.carRmin
          + Rs)); // The angle of frist turn


        Rs = fabs((Rs * mcarAlpha) / APGS_P.carV);


        mT2 = fabs((APGS_EE.carRmin * mcarAlpha) / APGS_P.carV); // The time of S2


        mscan = (Rs + mT2) / APGS_P.pathPoint; //The time of whole path need to take


        mcot_Phi2 = cos(mcarPHIS) / sin(mcarPHIS);


        mcot_op_m2 = APGS_EE.carl * mcot_Phi2;


        msin_op2 = (APGS_P.carV * sin(mcarPHIS)) / APGS_EE.carl;


        mg = 1.0; // Initial for path point count 


        memset(&APGS_B.mtp[0L], 0, (sizeof(real_T)) << ((uint8_T)8U)); // Buffer mtp initialization


        stepOneAngle = (int16_T)((real_T)((Rs + mscan) / mscan));// // The time of S1

        //------------------Start to calculate the trajectory of first turn ---------------------------
        for (o_count = (int16_T)0; o_count <= (stepOneAngle - ((int16_T)1));
             o_count++) {

          Rs = ((real_T)o_count) * mscan;


          APGS_B.mtp[((int16_T)mg) - ((int16_T)1)] = -(sin(msin_op2 * Rs) *
            mcot_op_m2);

          APGS_B.mtp[((int16_T)mg) + ((int16_T)127)] = (cos(msin_op2 * Rs) *
            (-mcot_op_m2)) + (APGS_EE.carl * mcot_Phi2);

          mg++;

        }
        //------------------End first turn trajectory calculation -----------------------------------
        
        L_theta = atan((APGS_B.mtp[((int16_T)((real_T)(mg - 2.0))) + ((int16_T)
          127)] - APGS_B.mtp[((int16_T)((real_T)(mg - 1.0))) + ((int16_T)127)]) /
                       (APGS_B.mtp[((int16_T)((real_T)(mg - 2.0))) - ((int16_T)1)]
                        - APGS_B.mtp[((int16_T)((real_T)(mg - 1.0))) - ((int16_T)
          1)])); // The angle of L in CLC

        APGS_DW.angleOfLine = ((L_theta * -180.0) / 3.1415926535897931) + 2.0; // Transfer radius into degree, 2.0 is to adjust the error because speed

        if (APGS_DW.parking_space <= 610.0) {

          Rs = APGS_EE.Lq; // For small space 
        } else {

          Rs = APGS_EE.Lq_s; // For big space
        }

        L = ((APGS_DW.parking_space / 100.0) - 5.2) * APGS_EE.Lg2;

        min_Ld = Rs + sqrt(L*L*2)/31.0;
        min_Lx = APGS_B.mtp[(int16_T)mg - 2];
        min_Ly = APGS_B.mtp[(int16_T)mg + 126];

        Rs = ((sqrt(((APGS_DW.parking_m - 0.5) * (APGS_DW.parking_m - 0.5)) +
                    ((APGS_DW.parking_m - 0.5) * (APGS_DW.parking_m - 0.5))) /
               31.0) + (((APGS_DW.parking_m - 0.5) * APGS_EE.Lg3) + Rs)) +
          (((((APGS_DW.parking_m - 0.5) * APGS_EE.Lg1) + 1.0) * sqrt((L * L) * 2.0)) /
           31.0);// Ld
        //------------------Start to calculate the trajectory of L length---------------------------
        for (stepOneAngle = (int8_T)0; stepOneAngle <= ((int8_T)30); // Loop for 30 point
             stepOneAngle++) {

          APGS_B.mtp[((int16_T)mg) - ((int16_T)1)] = APGS_B.mtp[((int16_T)
            ((real_T)(mg - 1.0))) - ((int16_T)1)] + (Rs * cos(L_theta));

          APGS_B.mtp[((int16_T)mg) + ((int16_T)127)] = APGS_B.mtp[((int16_T)
            ((real_T)(mg - 1.0))) + ((int16_T)127)] + (Rs * sin(L_theta));

          min_Lx = min_Lx + min_Ld * cos(L_theta);
          min_Ly = min_Ly + min_Ld * sin(L_theta);           
 
          mg++;

        }
        diff_x_pos = APGS_B.mtp[(int16_T)mg - 2]- min_Lx; 
        //------------------End of calculate the trajectory of L length ---------------------------     

        mTurn_cmd2 = mg;

        APGS_B.miiii_2 = mg; // Save the second turn point into miiii_2

        //----------------- Set steer angle to max ------------------------------------------- 
        if (mcarPHIS > 0.0) {

          mPHI = -APGS_EE.carMSA;
        } else {

          mPHI = APGS_EE.carMSA;
        }
        //---------------------------------------------------------------------------------
        //------------------Start to calculate the trajectory of second turn -------------------------

        mcot_Phi2 = cos(mPHI) / sin(mPHI);

        mcot_op_m2 = APGS_EE.carl * mcot_Phi2;

        msin_op2 = (APGS_P.carV * sin(mPHI)) / APGS_EE.carl;

        stepOneAngle = (int16_T)((real_T)((mT2 + mscan) / mscan));

        for (o_count = (int16_T)0; o_count <= (stepOneAngle - ((int16_T)1));
             o_count++) {

          Rs = ((real_T)o_count) * mscan;

          mT2 = -(sin(msin_op2 * Rs) * mcot_op_m2);

          Rs = (cos(msin_op2 * Rs) * (-mcot_op_m2)) + (APGS_EE.carl * mcot_Phi2);
          if (mPHI > 0.0) {

            L = -mcarAlpha;
          } else {

            L = mcarAlpha;
          }

          L_theta = (cos(L) * mT2) - (sin(L) * Rs);

          Rs = fabs((sin(L) * mT2) + (cos(L) * Rs)) + APGS_B.mtp[((int16_T)
            ((real_T)(mTurn_cmd2 - 1.0))) + ((int16_T)127)];

          APGS_B.mtp[((int16_T)mg) - ((int16_T)1)] = APGS_B.mtp[((int16_T)
            ((real_T)(mTurn_cmd2 - 1.0))) - ((int16_T)1)] + L_theta;

          APGS_B.mtp[((int16_T)mg) + ((int16_T)127)] = Rs;

          mg++;

        }
        //------------------End of calculate the trajectory of second turn -------------------------      

        //---------------------------------------------------------------------------------
        //------------------Transfer uint to cm------------------------------------------------
        for (stepOneAngle = (int16_T)0; stepOneAngle <= (((int16_T)((real_T)(mg
                 - 1.0))) - ((int16_T)1)); stepOneAngle++) {

          Rs = rt_roundd(APGS_B.mtp[stepOneAngle] * 100.0);
          if (Rs < 2.147483648E+9) {
            if (Rs >= -2.147483648E+9) {
              APGS_B.mtp[stepOneAngle] = (real_T)((int32_T)Rs);
            } else {
              APGS_B.mtp[stepOneAngle] = -2.147483648E+9;
            }
          } else {
            APGS_B.mtp[stepOneAngle] = 2.147483647E+9;
          }

          Rs = rt_roundd(APGS_B.mtp[128 + stepOneAngle] * 100.0);
          if (Rs < 2.147483648E+9) {
            if (Rs >= -2.147483648E+9) {
              APGS_B.mtp[128 + stepOneAngle] = (real_T)((int32_T)Rs);
            } else {
              APGS_B.mtp[128 + stepOneAngle] = -2.147483648E+9;
            }
          } else {
            APGS_B.mtp[128 + stepOneAngle] = 2.147483647E+9;
          }

        }
         //------------------End of Transfer uint to cm------------------------------------------------

        //---------------------------------------------------------------------------------
        //------------------Extend trajectory------------------------------------------------        
        for (stepOneAngle = (int16_T)0; stepOneAngle <= (((int16_T)((real_T)
                ((1.0 - mg) + 128.0))) - ((int16_T)1)); stepOneAngle++) {

          Rs = mg + ((real_T)stepOneAngle);

          APGS_B.mtp[((int16_T)Rs) + ((int16_T)127)] = APGS_B.mtp[((int16_T)
            ((real_T)(mg - 1.0))) + ((int16_T)127)];

          APGS_B.mtp[((int16_T)Rs) - ((int16_T)1)] = APGS_B.mtp[((int16_T)
            ((real_T)(Rs - 1.0))) - ((int16_T)1)] + 40.0;

        }
        //---------------------------------------------------------------------------------
        //------------------End of Extend trajectory------------------------------------------------           

        //APGS_DW.ActLevel = (uint16_T)6U; // Start parking
        APGS_B.PpathCalcFcn = 0;
        APGS_B.mangle_cmd = (mcarPHIS * 180.0) / 3.1415926535897931; // Radius to degree

      }
      //----------------------------------------------------------------// 
      //End of CalParkingPath_Parallel: Parallel path planning--------------------//
      //----------------------------------------------------------------// 



      //-----------------  mainTask_function-----------------------------------------------
    }

    //----------------------------------------------------------------// 
    //----- steerCtrlTask_function: Steer command strategy-------------------//
    //----------------------------------------------------------------//     
    if ((((int16_T)APGS_DW.temporalCounter_i1) == ((int16_T)1)) &&
        (APGS_B.steerCtlParkingFcn > ((uint16_T)0U))) { //steer controll function is triggered
 
      if (APGS_DW.IsSteerZeroCmd == 0.0) {

        APGS_DW.steer_cmd = 0L; //  Keep steer zero if parking_n is not in the defined range 

      } else if (APGS_U.parkMode == 1.0) { // parallel mode
        if (APGS_DW.IsMultiTurn == 0.0) { // Two turn

          if ((APGS_DW.yaw_odm >= APGS_DW.angleOfLine) &&
              (APGS_DW.IsPassFirstTurn == 0.0)) { // keep steer command at first turn until yaw angle is bigger than angleOfLine => C1 of CLC

            Rs = rt_roundd(((((APGS_B.mangle_cmd * APGS_B.mangle_cmd) * 0.0012)
                             + (-0.0012 * pow(APGS_B.mangle_cmd, 3.0))) +
                            (16.0559 * APGS_B.mangle_cmd)) + 0.52737); // Transfer car angle into steer angle
            if (Rs < 2.147483648E+9) {
              if (Rs >= -2.147483648E+9) {
                APGS_DW.steer_cmd = (int32_T)Rs;
              } else {
                APGS_DW.steer_cmd = MIN_int32_T;
              }
            } else {
              APGS_DW.steer_cmd = MAX_int32_T;
            }
          } else if ((APGS_B.miiii_2 < APGS_DW.LHPResult) &&
                     (APGS_DW.turningStep == 0.0)) { // second turn => C2 of  CLC

            APGS_DW.turningStep = 1.0;
          } else {
            if ((APGS_DW.yaw_odm <= APGS_DW.angleOfLine) && (APGS_B.miiii_2 >
                 APGS_DW.LHPResult)) { // Go straight line => L of  CLC

              APGS_DW.steer_cmd = 0L; //  make car go straight

              APGS_DW.IsPassFirstTurn = 1.0;
            }
          }

          if (APGS_DW.turningStep == 3.0) {// step 3 = > avoid hit right front car at first turn

            APGS_DW.steer_cmd = 0L;
          }

          if (APGS_DW.turningStep == 1.0) {// Get into second turn, set steer command to max 450

            APGS_DW.steer_cmd = -450L;
          }


          if (APGS_DW.final_steer == 1.0) { // Final step, set command to zero

            APGS_DW.steer_cmd = 0L;
          }
        } else {
          if (APGS_DW.IsMultiTurn == 1.0) { // Multi-Turn

            APGS_DW.turningStep = 2.0; 
            //------------Keep old command until driver follow ui prompt to change gear position----------
            if ((APGS_DW.multiShift == 1.0) && (APGS_U.gear_pos == ((uint16_T)
                  81U))) {

              APGS_DW.steer_cmd = 450L;
            } else {
              if ((APGS_DW.multiShift == 0.0) && (APGS_U.gear_pos == ((uint16_T)
                    119U))) {
                IsUnexpectChg_Parallel = 0; // Reset for unexpect change reuse
                APGS_DW.steer_cmd = -450L;
              }
            }
            //------------- Set command to zero when final_steer = 1.0---------------------
            if ((APGS_DW.final_steer == 1.0) && ((APGS_U.gear_pos == ((uint16_T)
                   81U)) || (APGS_U.gear_pos == ((uint16_T)119U)))) {

              APGS_DW.steer_cmd = 0L;
            }
          }
        }
      } else {
        if (APGS_U.parkMode == 2.0) { // back-in mode

          if (APGS_DW.backIn_step == 0.0) { //First turn, give max steer angle

            APGS_DW.steer_cmd = 450L;
          } else if (APGS_DW.backIn_step == 1.0) { //Rear obstacle detected, inform driver forward

            if ((APGS_DW.multiShift == 1.0) && (APGS_U.gear_pos == ((uint16_T)
                  81U))) {

              APGS_DW.steer_cmd = -450L;
            }
          } else if (APGS_DW.backIn_step == 2.0) {
  
            if (APGS_DW.y_pos > ((APGS_DW.parking_m * 100.0) + 20.0)) { // 20 cm can be adjusted

              if ((APGS_DW.multiShift == 0.0) && (APGS_U.gear_pos == ((uint16_T)
                    119U))) {

                if (APGS_DW.yaw_odm < -92.0) { // Give negative command  if yaw angle < -92
 
                  APGS_DW.steer_cmd = -400L; 
                } else if (APGS_DW.yaw_odm > -88.0) { // Give positive command  if yaw angle > -88 
    
                  APGS_DW.steer_cmd = 400L;
                } else {

                  APGS_DW.steer_cmd = 0L; // Set command to zero if yaw angle = 90
                }
              } else {

                APGS_DW.steer_cmd = 0L;
              }
            } else {

              APGS_DW.steer_cmd = 0L;
            }
          } else if (APGS_DW.backIn_step == 3.0) {

            if (((APGS_DW.multiShift == 1.0) || (APGS_DW.multiShift == 2.0)) &&
                (APGS_U.gear_pos == ((uint16_T)81U))) { // Avoid hit right car

              if (APGS_DW.backIn_IsOkToChg == 1.0) {

                APGS_DW.steer_cmd = 450L;
              } else {

                APGS_DW.steer_cmd = 0L; // Go straigh line until over 30 cm
              }
            }
          } else if (APGS_DW.backIn_step == 5.0) {

            if (((APGS_DW.multiShift == 1.0) || (APGS_DW.multiShift == 2.0)) &&
                (APGS_U.gear_pos == ((uint16_T)81U))) {// Avoid hit left car

              if (APGS_DW.backIn_IsOkToChg == 1.0) {

                APGS_DW.steer_cmd = -450L;
              } else {

                APGS_DW.steer_cmd = 0L; // Go straigh line until over 30 cm
              }
            }
          } else if (APGS_DW.backIn_step == 6.0) { // Spece is big enough to park with one turn

            APGS_DW.steer_cmd = 0L;
          } else if (APGS_DW.backIn_step == 7.0) { // Keep host car yaw angle at about 90 degree

            if (APGS_DW.yaw_odm < -92.0) {

              if (APGS_U.gear_pos == ((uint16_T)119U)) {

                APGS_DW.steer_cmd = -300L;
              } else {
                if (APGS_U.gear_pos == ((uint16_T)81U)) {

                  APGS_DW.steer_cmd = 300L;
                }
              }
            } else if (APGS_DW.yaw_odm > -88.0) {

              if (APGS_U.gear_pos == ((uint16_T)119U)) {

                APGS_DW.steer_cmd = 300L;
              } else {
                if (APGS_U.gear_pos == ((uint16_T)81U)) {

                  APGS_DW.steer_cmd = -300L;
                }
              }
            } else {
 
              APGS_DW.steer_cmd = 0L;
            }
          } else if (APGS_DW.backIn_step == 8.0) { // Keep host car close to right

            APGS_DW.paraOfBackInCorrect = ((APGS_DW.parking_space) -
              APGS_EE.carW*100) / 2.0; //  180.9cm is width of Tiguan 

            //------Keep variable paraOfBackInCorrect in  50~100 cm
            if (APGS_DW.paraOfBackInCorrect < 50.0) {

              APGS_DW.paraOfBackInCorrect = 50.0;
            } else {
              if (APGS_DW.paraOfBackInCorrect > 100.0) {

                APGS_DW.paraOfBackInCorrect = 100.0;
              }
            }

            
                        
            if (APGS_U.back_right > APGS_DW.paraOfBackInCorrect) {

              Rs = APGS_U.back_right - APGS_DW.paraOfBackInCorrect;
              //-------Give different command with paraOfBackInCorrect  change, make steer control smooth.
              if ((Rs > 5.0) && (Rs < 15.0)) {

                APGS_DW.steer_cmd = 180L;
              } else if ((Rs > 15.0) && (Rs < 30.0)) {

                APGS_DW.steer_cmd = 300L;
              } else if (Rs > 30.0) {

                APGS_DW.steer_cmd = 450L;
              } else {
                if (Rs < 5.0) {

                  APGS_DW.steer_cmd = 0L;
                }
              }
            } else {
 
              Rs = APGS_DW.paraOfBackInCorrect - APGS_U.back_right;
              //-------Give different command with paraOfBackInCorrect  change, make steer control smooth.
              if ((Rs > 5.0) && (Rs < 15.0)) {

                APGS_DW.steer_cmd = -180L;
              } else if ((Rs > 15.0) && (Rs < 30.0)) {

                APGS_DW.steer_cmd = -300L;
              } else if (Rs > 30.0) {

                APGS_DW.steer_cmd = -450L;
              } else {
                if (Rs < 5.0) {

                  APGS_DW.steer_cmd = 0L;
                }
              }
            }
            //---------Avoid steer command over 450 or -450
            if (APGS_DW.steer_cmd > 450L) {

              APGS_DW.steer_cmd = 450L;
            } else {
              if (APGS_DW.steer_cmd < -450L) {

                APGS_DW.steer_cmd = -450L;
              }
            }
            //--------------------------------------------
          } else if (APGS_DW.backIn_step == 9.0) { // Parallel correction by using ulltrasonic ,if front_right > back_right

            if ((APGS_DW.multiShift == 0.0) && (APGS_U.gear_pos == ((uint16_T)
                  119U))) {

              if (fabs(APGS_U.front_right - APGS_U.back_right) > 30.0) {

                APGS_DW.steer_cmd = 0L;
              } else {

                APGS_DW.steer_cmd = -90L;
              }
            } else if (fabs(APGS_U.front_right - APGS_U.back_right) > 30.0) {

              APGS_DW.steer_cmd = 0L;
            } else {
 
              APGS_DW.steer_cmd = 90L;
            }
          } else if (APGS_DW.backIn_step == 10.0) {
            // Parallel correction by using ulltrasonic ,if front_right < back_right
            if ((APGS_DW.multiShift == 0.0) && (APGS_U.gear_pos == ((uint16_T)
                  119U))) {

              if (fabs(APGS_U.front_right - APGS_U.back_right) > 30.0) {

                APGS_DW.steer_cmd = 0L;
              } else {
 
                APGS_DW.steer_cmd = 90L;
              }
            } else if (fabs(APGS_U.front_right - APGS_U.back_right) > 30.0) {

              APGS_DW.steer_cmd = 0L;
            } else {

              APGS_DW.steer_cmd = -90L;
            }
          } else if (APGS_DW.backIn_step == 11.0) { // upnexpect case

            APGS_DW.steer_cmd = 0L;
          } else {
            if (APGS_DW.backIn_step == 4.0) { // Final step

              APGS_DW.steer_cmd = 0L;
            }
          }
        }
      }


    }
    //----------------------------------------------------------------// 
    //----- End of steerCtrlTask_function: Steer command strategy-------------------//
    //----------------------------------------------------------------// 



  }

  if (((int16_T)APGS_DW.temporalCounter_i1) == ((int16_T)1)) {
    APGS_DW.temporalCounter_i1 = 0U;
  }

  // End of TaskScheduler
}

/******************************************************
 * Function: APGS_initialize
 * Description: APGS model-based parameter initializatoin
 * Parameters: None
 * Return value: None
 *****************************************************/
void APGS_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(APGS_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &APGS_B), 0,
                sizeof(B_APGS_T));

  /* states (dwork) */
  (void) memset((void *)&APGS_DW, 0,
                sizeof(DW_APGS_T));

  /* external inputs */
  (void) memset((void *)&APGS_U, 0,
                sizeof(ExtU_APGS_T));

  /* Start for S-Function (fcncallgen): '<S2>/TriggerSrc' incorporates:
   *  Start for Chart: '<S1>/TaskScheduler'
   */
  /* Start for Chart: '<S1>/TaskScheduler' incorporates:
   *  Start for SubSystem: '<S1>/mainTask_function'
   */
  /* InitializeConditions for Enabled SubSystem: '<S4>/edgDetCalc1' */
  /* InitializeConditions for Delay: '<S11>/Delay' incorporates:
   *  Start for Delay: '<S11>/Delay'
   */
  APGS_DW.Delay_DSTATE = APGS_P.Delay_InitialCondition;

  /* InitializeConditions for Delay: '<S11>/Delay1' incorporates:
   *  Start for Delay: '<S11>/Delay1'
   */
  APGS_DW.Delay1_DSTATE = APGS_P.Delay1_InitialCondition;

  /* End of InitializeConditions for SubSystem: '<S4>/edgDetCalc1' */

  /* Start for DataStoreMemory: '<S1>/ActLevel' */
  APGS_DW.ActLevel = APGS_P.ActLevel_InitialValue;

  /* Start for DataStoreMemory: '<S1>/IsMultiTurn' */
  APGS_DW.IsMultiTurn = APGS_P.IsMultiTurn_InitialValue;

  /* Start for DataStoreMemory: '<S1>/IsParallelOk' */
  APGS_DW.IsParallelOk = APGS_P.IsParallelOk_InitialValue;

  /* Start for DataStoreMemory: '<S1>/IsPassFirstTurn' */
  APGS_DW.IsPassFirstTurn = APGS_P.IsPassFirstTurn_InitialValue;

  /* Start for DataStoreMemory: '<S1>/IsSpaceOk' */
  APGS_DW.IsSpaceOk = APGS_P.IsSpaceOk_InitialValue;

  /* Start for DataStoreMemory: '<S1>/IsSteerZeroCmd' */
  APGS_DW.IsSteerZeroCmd = APGS_P.IsSteerZeroCmd_InitialValue;

  /* Start for DataStoreMemory: '<S1>/Is_bal_parallel_ok' */
  APGS_DW.Is_bal_parallel_ok = APGS_P.Is_bal_parallel_ok_InitialValue;

  /* Start for DataStoreMemory: '<S1>/LHPResult' */
  APGS_DW.LHPResult = APGS_P.LHPResult_InitialValue;

  /* Start for DataStoreMemory: '<S1>/MsgForUI' */
  APGS_DW.MsgForUI = APGS_P.MsgForUI_InitialValue;

  /* Start for DataStoreMemory: '<S1>/angle' */
  APGS_DW.angle = APGS_P.angle_InitialValue;

  /* Start for DataStoreMemory: '<S1>/angleOfLine' */
  APGS_DW.angleOfLine = APGS_P.angleOfLine_InitialValue;

  /* Start for DataStoreMemory: '<S1>/angleOfSecCar' */
  APGS_DW.angleOfSecCar = APGS_P.angleOfSecCar_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_IsOkToChg' */
  APGS_DW.backIn_IsOkToChg = APGS_P.backIn_IsOkToChg_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_backN' */
  APGS_DW.backIn_backN = APGS_P.backIn_backN_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_step' */
  APGS_DW.backIn_step = APGS_P.backIn_step_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_tmp_xpos' */
  APGS_DW.backIn_tmp_xpos = APGS_P.backIn_tmp_xpos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_tmp_yaw_odm' */
  APGS_DW.backIn_tmp_yaw_odm = APGS_P.backIn_tmp_yaw_odm_InitialValue;

  /* Start for DataStoreMemory: '<S1>/backIn_tmp_ypos' */
  APGS_DW.backIn_tmp_ypos = APGS_P.backIn_tmp_ypos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_left_cnt' */
  APGS_DW.back_bal_left_cnt = APGS_P.back_bal_left_cnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_left_ref' */
  APGS_DW.back_bal_left_ref = APGS_P.back_bal_left_ref_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_left_sum' */
  APGS_DW.back_bal_left_sum = APGS_P.back_bal_left_sum_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_right_cnt' */
  APGS_DW.back_bal_right_cnt = APGS_P.back_bal_right_cnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_right_ref' */
  APGS_DW.back_bal_right_ref = APGS_P.back_bal_right_ref_InitialValue;

  /* Start for DataStoreMemory: '<S1>/back_bal_right_sum' */
  APGS_DW.back_bal_right_sum = APGS_P.back_bal_right_sum_InitialValue;

  /* Start for DataStoreMemory: '<S1>/bal_mode' */
  APGS_DW.bal_mode = APGS_P.bal_mode_InitialValue;

  /* Start for DataStoreMemory: '<S1>/car_Speed' */
  APGS_DW.car_Speed = APGS_P.car_Speed_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg1_cnt' */
  APGS_DW.edg1_cnt = APGS_P.edg1_cnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg1_stage' */
  APGS_DW.edg1_stage = APGS_P.edg1_stage_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg2_cnt' */
  APGS_DW.edg2_cnt = APGS_P.edg2_cnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edgEndFlg' */
  APGS_DW.edgEndFlg = APGS_P.edgEndFlg_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg_count' */
  APGS_DW.edg_count = APGS_P.edg_count_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg_flg' */
  APGS_DW.edg_flg = APGS_P.edg_flg_InitialValue;

  /* Start for DataStoreMemory: '<S1>/edg_info' */
  memcpy(&APGS_DW.edg_info[0L], &APGS_P.edg_info_InitialValue[0L], ((uint16_T)
          90U) * (sizeof(real_T)));

  /* Start for DataStoreMemory: '<S1>/final_steer' */
  APGS_DW.final_steer = APGS_P.final_steer_InitialValue;

  /* Start for DataStoreMemory: '<S1>/first_edg_pos' */
  APGS_DW.first_edg_pos = APGS_P.first_edg_pos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/int_count' */
  APGS_DW.int_count = APGS_P.int_count_InitialValue;

  /* Start for DataStoreMemory: '<S1>/moving_distance' */
  APGS_DW.moving_distance = APGS_P.moving_distance_InitialValue;

  /* Start for DataStoreMemory: '<S1>/multiShift' */
  APGS_DW.multiShift = APGS_P.multiShift_InitialValue;

  /* Start for DataStoreMemory: '<S1>/paraOfBackInCorrect' */
  APGS_DW.paraOfBackInCorrect = APGS_P.paraOfBackInCorrect_InitialValu;

  /* Start for DataStoreMemory: '<S1>/parking_edge_pos' */
  APGS_DW.parking_edge_pos = APGS_P.parking_edge_pos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/parking_m' */
  APGS_DW.parking_m = APGS_P.parking_m_InitialValue;

  /* Start for DataStoreMemory: '<S1>/parking_n' */
  APGS_DW.parking_n = APGS_P.parking_n_InitialValue;

  /* Start for DataStoreMemory: '<S1>/parking_space' */
  APGS_DW.parking_space = APGS_P.parking_space_InitialValue;

  /* Start for DataStoreMemory: '<S1>/second_edg_pos' */
  APGS_DW.second_edg_pos = APGS_P.second_edg_pos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/steerTurnCnt' */
  APGS_DW.steerTurnCnt = APGS_P.steerTurnCnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/steerTurnCnt_old' */
  APGS_DW.steerTurnCnt_old = APGS_P.steerTurnCnt_old_InitialValue;

  /* Start for DataStoreMemory: '<S1>/steer_cmd' */
  APGS_DW.steer_cmd = APGS_P.steer_cmd_InitialValue;

  /* Start for DataStoreMemory: '<S1>/turningStep' */
  APGS_DW.turningStep = APGS_P.turningStep_InitialValue;

  /* Start for DataStoreMemory: '<S1>/ultra_edge2_cnt' */
  APGS_DW.ultra_edge2_cnt = APGS_P.ultra_edge2_cnt_InitialValue;

  /* Start for DataStoreMemory: '<S1>/vld_status' */
  APGS_DW.vld_status = APGS_P.vld_status_InitialValue;

  /* Start for DataStoreMemory: '<S1>/x_pos' */
  APGS_DW.x_pos = APGS_P.x_pos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/y_pos' */
  APGS_DW.y_pos = APGS_P.y_pos_InitialValue;

  /* Start for DataStoreMemory: '<S1>/yaw_odm' */
  APGS_DW.yaw_odm = APGS_P.yaw_odm_InitialValue;

  APGS_DW.dis_For_OneEdge = 0;
  APGS_DW.dis_OverFlow_Cnt = 0;
  APGS_DW.IsOneEdgeFound = 0;

  /* InitializeConditions for S-Function (fcncallgen): '<S2>/TriggerSrc' incorporates:
   *  InitializeConditions for Chart: '<S1>/TaskScheduler'
   */
  APGS_DW.is_active_shcheduler = 0U;
  APGS_DW.temporalCounter_i1 = 0U;
  APGS_DW.is_active_c3_APGS = 0U;

  /* InitializeConditions for Chart: '<S1>/TaskScheduler' incorporates:
   *  InitializeConditions for SubSystem: '<S1>/mainTask_function'
   */
  /* InitializeConditions for Chart: '<S7>/APGS_FlowAction' */
  APGS_DW.is_active_c1_APGS = 0U;
  APGS_DW.is_c1_APGS = APGS_IN_NO_ACTIVE_CHILD;
}

/******************************************************
 * Function: APGS_terminate
 * Description: terminate function -> currently not used
 * Parameters: None
 * Return value: None
 *****************************************************/
void APGS_terminate(void)
{
  /* (no terminate code required) */
}

/*History 
*  ---------------------------------
*  a) Add mechanism which has ability to replan action when user unexpects to change the gear position .
*  b) Take the APGS_DW.edg_count to outside in edge0Det. To solve the bug of one edge detection.
*  02/18/105 modified by Stanley Li
*  ---------------------------------
*  12/15/104 ceated by Stanley Li
*
*/

