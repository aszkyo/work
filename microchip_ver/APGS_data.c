/*
 * File: APGS_data.c
 *
 * Code generated for Simulink model 'APGS'.
 *
 * Model version                  : 1.1064
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Sun Mar 01 14:00:36 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (Microchip->dsPIC)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "APGS.h"
#include "APGS_private.h"

/* Block parameters (auto storage) */
P_APGS_T APGS_P = {
  2.5,                                 /* Variable: carD
                                        * Referenced by: '<S9>/carD'
                                        */
  5.2,                                 /* Variable: carH
                                        * Referenced by: '<S9>/carH'
                                        */
  0.56,                                /* Variable: carMSA
                                        * Referenced by: '<S9>/carMSA'
                                        */
  6.0,                                 /* Variable: carRmin
                                        * Referenced by:
                                        *   '<S8>/carRmin'
                                        *   '<S9>/carRmin'
                                        */
  1.571,                               /* Variable: carTread
                                        * Referenced by:
                                        *   '<S9>/carTread'
                                        *   '<S23>/carTread'
                                        */
  -2.0,                                /* Variable: carV
                                        * Referenced by: '<S9>/carV'
                                        */
  1.809,                               /* Variable: carW
                                        * Referenced by:
                                        *   '<S8>/carW'
                                        *   '<S9>/carW'
                                        */
  0.2,                                 /* Variable: carb0
                                        * Referenced by: '<S9>/carb0'
                                        */
  0.2,                                 /* Variable: carb1
                                        * Referenced by: '<S9>/carb1'
                                        */
  2.604,                               /* Variable: carl
                                        * Referenced by: '<S9>/carl'
                                        */
  90.0,                                /* Variable: pathPoint
                                        * Referenced by: '<S9>/pathPoint'
                                        */
  0.0,                                 /* Expression: 0.0
                                        * Referenced by: '<S11>/Delay'
                                        */
  0.0,                                 /* Expression: 0.0
                                        * Referenced by: '<S11>/Delay1'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/IsMultiTurn'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/IsPassFirstTurn'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/IsSpaceOk'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/IsSteerZeroCmd'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Is_bal_parallel_ok'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/LHPResult'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/MsgForUI'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/angle'
                                        */
  -3.0,                                /* Expression: -3
                                        * Referenced by: '<S1>/angleOfLine'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/angleOfSecCar'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_IsOkToChg'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_backN'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_step'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_tmp_xpos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_tmp_yaw_odm'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/backIn_tmp_ypos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/back_bal_left_cnt'
                                        */
  50.0,                                /* Expression: 50
                                        * Referenced by: '<S1>/back_bal_left_ref'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/back_bal_left_sum'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/back_bal_right_cnt'
                                        */
  50.0,                                /* Expression: 50
                                        * Referenced by: '<S1>/back_bal_right_ref'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/back_bal_right_sum'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/bal_mode'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/car_Speed'
                                        */

  /*  Expression: zeros(3,30)
   * Referenced by: '<S1>/edg_info'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/final_steer'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/first_edg_pos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/moving_distance'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/multiShift'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/paraOfBackInCorrect'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/parking_edge_pos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/parking_m'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/parking_n'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/parking_space'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/second_edg_pos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/turningStep'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/vld_status'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/x_pos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/y_pos'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/yaw_odm'
                                        */
  0,                                   /* Computed Parameter: steer_cmd_InitialValue
                                        * Referenced by: '<S1>/steer_cmd'
                                        */
  1U,                                  /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S11>/Delay'
                                        */
  1U,                                  /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S11>/Delay1'
                                        */
  0U,                                  /* Computed Parameter: ActLevel_InitialValue
                                        * Referenced by: '<S1>/ActLevel'
                                        */
  1U,                                  /* Computed Parameter: edg_count_InitialValue
                                        * Referenced by: '<S1>/edg_count'
                                        */
  1U,                                  /* Computed Parameter: int_count_InitialValue
                                        * Referenced by: '<S1>/int_count'
                                        */
  1U,                                  /* Computed Parameter: ultra_edge2_cnt_InitialValue
                                        * Referenced by: '<S1>/ultra_edge2_cnt'
                                        */
  0U,                                  /* Computed Parameter: IsParallelOk_InitialValue
                                        * Referenced by: '<S1>/IsParallelOk'
                                        */
  0U,                                  /* Computed Parameter: edg1_cnt_InitialValue
                                        * Referenced by: '<S1>/edg1_cnt'
                                        */
  0U,                                  /* Computed Parameter: edg1_stage_InitialValue
                                        * Referenced by: '<S1>/edg1_stage'
                                        */
  0U,                                  /* Computed Parameter: edg2_cnt_InitialValue
                                        * Referenced by: '<S1>/edg2_cnt'
                                        */
  0U,                                  /* Computed Parameter: edgEndFlg_InitialValue
                                        * Referenced by: '<S1>/edgEndFlg'
                                        */
  0U,                                  /* Computed Parameter: edg_flg_InitialValue
                                        * Referenced by: '<S1>/edg_flg'
                                        */
  0U,                                  /* Computed Parameter: steerTurnCnt_InitialValue
                                        * Referenced by: '<S1>/steerTurnCnt'
                                        */
  0U                                   /* Computed Parameter: steerTurnCnt_old_InitialValue
                                        * Referenced by: '<S1>/steerTurnCnt_old'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
