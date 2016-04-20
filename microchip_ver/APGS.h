/*
 * File: APGS.h
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

#ifndef RTW_HEADER_APGS_h_
#define RTW_HEADER_APGS_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef APGS_COMMON_INCLUDES_
# define APGS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* APGS_COMMON_INCLUDES_ */

#include "APGS_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T mtp[256];                     /* '<S9>/get_Rs_Fcn' */
  real_T miiii_2;                      /* '<S9>/get_Rs_Fcn' */
  real_T mangle_cmd;                   /* '<S9>/get_Rs_Fcn' */
  uint16_T edgDetFcn;                  /* '<S7>/APGS_FlowAction' */
  uint16_T steerCtlParkingFcn;         /* '<S7>/APGS_FlowAction' */
  uint16_T PpathCalcFcn;               /* '<S7>/APGS_FlowAction' */
  uint16_T BpathCalcFcn;               /* '<S7>/APGS_FlowAction' */
  uint16_T PparkingFlowFcn;            /* '<S7>/APGS_FlowAction' */
  uint16_T BparkingFlowFcn;            /* '<S7>/APGS_FlowAction' */
} B_APGS_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T dis_For_OneEdge;
  real_T dis_OverFlow_Cnt;
  real_T IsOneEdgeFound;
  real_T Delay_DSTATE;                 /* '<S11>/Delay' */
  real_T Delay1_DSTATE;                /* '<S11>/Delay1' */
  real_T IsMultiTurn;                  /* '<S1>/IsMultiTurn' */
  real_T IsPassFirstTurn;              /* '<S1>/IsPassFirstTurn' */
  real_T IsSpaceOk;                    /* '<S1>/IsSpaceOk' */
  real_T IsSteerZeroCmd;               /* '<S1>/IsSteerZeroCmd' */
  real_T Is_bal_parallel_ok;           /* '<S1>/Is_bal_parallel_ok' */
  real_T LHPResult;                    /* '<S1>/LHPResult' */
  real_T MsgForUI;                     /* '<S1>/MsgForUI' */
  real_T angle;                        /* '<S1>/angle' */
  real_T angleOfLine;                  /* '<S1>/angleOfLine' */
  real_T angleOfSecCar;                /* '<S1>/angleOfSecCar' */
  real_T backIn_IsOkToChg;             /* '<S1>/backIn_IsOkToChg' */
  real_T backIn_backN;                 /* '<S1>/backIn_backN' */
  real_T backIn_step;                  /* '<S1>/backIn_step' */
  real_T backIn_tmp_xpos;              /* '<S1>/backIn_tmp_xpos' */
  real_T backIn_tmp_yaw_odm;           /* '<S1>/backIn_tmp_yaw_odm' */
  real_T backIn_tmp_ypos;              /* '<S1>/backIn_tmp_ypos' */
  real_T back_bal_left_cnt;            /* '<S1>/back_bal_left_cnt' */
  real_T back_bal_left_ref;            /* '<S1>/back_bal_left_ref' */
  real_T back_bal_left_sum;            /* '<S1>/back_bal_left_sum' */
  real_T back_bal_right_cnt;           /* '<S1>/back_bal_right_cnt' */
  real_T back_bal_right_ref;           /* '<S1>/back_bal_right_ref' */
  real_T back_bal_right_sum;           /* '<S1>/back_bal_right_sum' */
  real_T bal_mode;                     /* '<S1>/bal_mode' */
  real_T car_Speed;                    /* '<S1>/car_Speed' */
  real_T edg_info[90];                 /* '<S1>/edg_info' */
  real_T final_steer;                  /* '<S1>/final_steer' */
  real_T first_edg_pos;                /* '<S1>/first_edg_pos' */
  real_T moving_distance;              /* '<S1>/moving_distance' */
  real_T multiShift;                   /* '<S1>/multiShift' */
  real_T paraOfBackInCorrect;          /* '<S1>/paraOfBackInCorrect' */
  real_T parking_edge_pos;             /* '<S1>/parking_edge_pos' */
  real_T parking_m;                    /* '<S1>/parking_m' */
  real_T parking_n;                    /* '<S1>/parking_n' */
  real_T parking_space;                /* '<S1>/parking_space' */
  real_T second_edg_pos;               /* '<S1>/second_edg_pos' */
  real_T turningStep;                  /* '<S1>/turningStep' */
  real_T vld_status;                   /* '<S1>/vld_status' */
  real_T x_pos;                        /* '<S1>/x_pos' */
  real_T y_pos;                        /* '<S1>/y_pos' */
  real_T yaw_odm;                      /* '<S1>/yaw_odm' */
  real_T veh_wheel_std;                /* '<S11>/edgDetCalc' */
  int32_T steer_cmd;                   /* '<S1>/steer_cmd' */
  uint16_T ActLevel;                   /* '<S1>/ActLevel' */
  uint16_T edg_count;                  /* '<S1>/edg_count' */
  uint16_T int_count;                  /* '<S1>/int_count' */
  uint16_T ultra_edge2_cnt;            /* '<S1>/ultra_edge2_cnt' */
  uint16_T waitingMode;                /* '<S7>/APGS_FlowAction' */
  uint16_T edgDetMode;                 /* '<S7>/APGS_FlowAction' */
  uint16_T pathCalcMode;               /* '<S7>/APGS_FlowAction' */
  uint16_T enable;                     /* '<S7>/APGS_FlowAction' */
  uint16_T disable;                    /* '<S7>/APGS_FlowAction' */
  uint16_T APGS_FINISH;                /* '<S7>/APGS_FlowAction' */
  uint16_T parkingMode;                /* '<S7>/APGS_FlowAction' */
  uint16_T steerMode;                  /* '<S7>/APGS_FlowAction' */
  uint8_T IsParallelOk;                /* '<S1>/IsParallelOk' */
  uint8_T edg1_cnt;                    /* '<S1>/edg1_cnt' */
  uint8_T edg1_stage;                  /* '<S1>/edg1_stage' */
  uint8_T edg2_cnt;                    /* '<S1>/edg2_cnt' */
  uint8_T edgEndFlg;                   /* '<S1>/edgEndFlg' */
  uint8_T edg_flg;                     /* '<S1>/edg_flg' */
  uint8_T steerTurnCnt;                /* '<S1>/steerTurnCnt' */
  uint8_T steerTurnCnt_old;            /* '<S1>/steerTurnCnt_old' */
  uint8_T is_active_c3_APGS;           /* '<S1>/TaskScheduler' */
  uint8_T is_active_shcheduler;        /* '<S1>/TaskScheduler' */
  uint8_T temporalCounter_i1;          /* '<S1>/TaskScheduler' */
  uint8_T is_active_c1_APGS;           /* '<S7>/APGS_FlowAction' */
  uint8_T is_c1_APGS;                  /* '<S7>/APGS_FlowAction' */
} DW_APGS_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  uint16_T edgDetStartFlg;             /* '<Root>/edgDetStartFlg' */
  real_T wheel_R;                      /* '<Root>/wheel_R' */
  real_T wheel_L;                      /* '<Root>/wheel_L' */
  uint16_T gear_pos;                   /* '<Root>/gear_pos' */
  real_T front_right;                  /* '<Root>/front_right' */
  real_T ultraSonic_RCL;               /* '<Root>/ultraSonic_RCL' */
  real_T ultraSonic_RML;               /* '<Root>/ultraSonic_RML' */
  real_T ultraSonic_RMR;               /* '<Root>/ultraSonic_RMR' */
  real_T ultraSonic_RCR;               /* '<Root>/ultraSonic_RCR' */
  real_T ultraSonic_FCL;               /* '<Root>/ultraSonic_FCL' */
  real_T ultraSonic_FML;               /* '<Root>/ultraSonic_FML' */
  real_T ultraSonic_FMR;               /* '<Root>/ultraSonic_FMR' */
  real_T ultraSonic_FCR;               /* '<Root>/ultraSonic_FCR' */
  real_T parkMode;                     /* '<Root>/parkMode' */
  real_T SAngle;                       /* '<Root>/SAngle' */
  real_T back_right;                   /* '<Root>/back_right' */
  real_T back_left;                    /* '<Root>/back_left' */
  real_T front_left;                   /* '<Root>/front_left' */
  real_T update_Ultra;                 /* '<Root>/update_Ultra' */
  real_T dir_Light;                    /* '<Root>/dir_Light' */
} ExtU_APGS_T;

/* Parameters (auto storage) */
struct P_APGS_T_ {
  real_T carD;                         /* Variable: carD
                                        * Referenced by: '<S9>/carD'
                                        */
  real_T carH;                         /* Variable: carH
                                        * Referenced by: '<S9>/carH'
                                        */
  real_T carMSA;                       /* Variable: carMSA
                                        * Referenced by: '<S9>/carMSA'
                                        */
  real_T carRmin;                      /* Variable: carRmin
                                        * Referenced by:
                                        *   '<S8>/carRmin'
                                        *   '<S9>/carRmin'
                                        */
  real_T carTread;                     /* Variable: carTread
                                        * Referenced by:
                                        *   '<S9>/carTread'
                                        *   '<S23>/carTread'
                                        */
  real_T carV;                         /* Variable: carV
                                        * Referenced by: '<S9>/carV'
                                        */
  real_T carW;                         /* Variable: carW
                                        * Referenced by:
                                        *   '<S8>/carW'
                                        *   '<S9>/carW'
                                        */
  real_T carb0;                        /* Variable: carb0
                                        * Referenced by: '<S9>/carb0'
                                        */
  real_T carb1;                        /* Variable: carb1
                                        * Referenced by: '<S9>/carb1'
                                        */
  real_T carl;                         /* Variable: carl
                                        * Referenced by: '<S9>/carl'
                                        */
  real_T pathPoint;                    /* Variable: pathPoint
                                        * Referenced by: '<S9>/pathPoint'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0.0
                                        * Referenced by: '<S11>/Delay'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0.0
                                        * Referenced by: '<S11>/Delay1'
                                        */
  real_T IsMultiTurn_InitialValue;     /* Expression: 0
                                        * Referenced by: '<S1>/IsMultiTurn'
                                        */
  real_T IsPassFirstTurn_InitialValue; /* Expression: 0
                                        * Referenced by: '<S1>/IsPassFirstTurn'
                                        */
  real_T IsSpaceOk_InitialValue;       /* Expression: 0
                                        * Referenced by: '<S1>/IsSpaceOk'
                                        */
  real_T IsSteerZeroCmd_InitialValue;  /* Expression: 0
                                        * Referenced by: '<S1>/IsSteerZeroCmd'
                                        */
  real_T Is_bal_parallel_ok_InitialValue;/* Expression: 0
                                          * Referenced by: '<S1>/Is_bal_parallel_ok'
                                          */
  real_T LHPResult_InitialValue;       /* Expression: 0
                                        * Referenced by: '<S1>/LHPResult'
                                        */
  real_T MsgForUI_InitialValue;        /* Expression: 0
                                        * Referenced by: '<S1>/MsgForUI'
                                        */
  real_T angle_InitialValue;           /* Expression: 0
                                        * Referenced by: '<S1>/angle'
                                        */
  real_T angleOfLine_InitialValue;     /* Expression: -3
                                        * Referenced by: '<S1>/angleOfLine'
                                        */
  real_T angleOfSecCar_InitialValue;   /* Expression: 0
                                        * Referenced by: '<S1>/angleOfSecCar'
                                        */
  real_T backIn_IsOkToChg_InitialValue;/* Expression: 0
                                        * Referenced by: '<S1>/backIn_IsOkToChg'
                                        */
  real_T backIn_backN_InitialValue;    /* Expression: 0
                                        * Referenced by: '<S1>/backIn_backN'
                                        */
  real_T backIn_step_InitialValue;     /* Expression: 0
                                        * Referenced by: '<S1>/backIn_step'
                                        */
  real_T backIn_tmp_xpos_InitialValue; /* Expression: 0
                                        * Referenced by: '<S1>/backIn_tmp_xpos'
                                        */
  real_T backIn_tmp_yaw_odm_InitialValue;/* Expression: 0
                                          * Referenced by: '<S1>/backIn_tmp_yaw_odm'
                                          */
  real_T backIn_tmp_ypos_InitialValue; /* Expression: 0
                                        * Referenced by: '<S1>/backIn_tmp_ypos'
                                        */
  real_T back_bal_left_cnt_InitialValue;/* Expression: 0
                                         * Referenced by: '<S1>/back_bal_left_cnt'
                                         */
  real_T back_bal_left_ref_InitialValue;/* Expression: 50
                                         * Referenced by: '<S1>/back_bal_left_ref'
                                         */
  real_T back_bal_left_sum_InitialValue;/* Expression: 0
                                         * Referenced by: '<S1>/back_bal_left_sum'
                                         */
  real_T back_bal_right_cnt_InitialValue;/* Expression: 0
                                          * Referenced by: '<S1>/back_bal_right_cnt'
                                          */
  real_T back_bal_right_ref_InitialValue;/* Expression: 50
                                          * Referenced by: '<S1>/back_bal_right_ref'
                                          */
  real_T back_bal_right_sum_InitialValue;/* Expression: 0
                                          * Referenced by: '<S1>/back_bal_right_sum'
                                          */
  real_T bal_mode_InitialValue;        /* Expression: 0
                                        * Referenced by: '<S1>/bal_mode'
                                        */
  real_T car_Speed_InitialValue;       /* Expression: 0
                                        * Referenced by: '<S1>/car_Speed'
                                        */
  real_T edg_info_InitialValue[90];    /* Expression: zeros(3,30)
                                        * Referenced by: '<S1>/edg_info'
                                        */
  real_T final_steer_InitialValue;     /* Expression: 0
                                        * Referenced by: '<S1>/final_steer'
                                        */
  real_T first_edg_pos_InitialValue;   /* Expression: 0
                                        * Referenced by: '<S1>/first_edg_pos'
                                        */
  real_T moving_distance_InitialValue; /* Expression: 0
                                        * Referenced by: '<S1>/moving_distance'
                                        */
  real_T multiShift_InitialValue;      /* Expression: 0
                                        * Referenced by: '<S1>/multiShift'
                                        */
  real_T paraOfBackInCorrect_InitialValu;/* Expression: 0
                                          * Referenced by: '<S1>/paraOfBackInCorrect'
                                          */
  real_T parking_edge_pos_InitialValue;/* Expression: 0
                                        * Referenced by: '<S1>/parking_edge_pos'
                                        */
  real_T parking_m_InitialValue;       /* Expression: 0
                                        * Referenced by: '<S1>/parking_m'
                                        */
  real_T parking_n_InitialValue;       /* Expression: 0
                                        * Referenced by: '<S1>/parking_n'
                                        */
  real_T parking_space_InitialValue;   /* Expression: 0
                                        * Referenced by: '<S1>/parking_space'
                                        */
  real_T second_edg_pos_InitialValue;  /* Expression: 0
                                        * Referenced by: '<S1>/second_edg_pos'
                                        */
  real_T turningStep_InitialValue;     /* Expression: 0
                                        * Referenced by: '<S1>/turningStep'
                                        */
  real_T vld_status_InitialValue;      /* Expression: 0
                                        * Referenced by: '<S1>/vld_status'
                                        */
  real_T x_pos_InitialValue;           /* Expression: 0
                                        * Referenced by: '<S1>/x_pos'
                                        */
  real_T y_pos_InitialValue;           /* Expression: 0
                                        * Referenced by: '<S1>/y_pos'
                                        */
  real_T yaw_odm_InitialValue;         /* Expression: 0
                                        * Referenced by: '<S1>/yaw_odm'
                                        */
  int32_T steer_cmd_InitialValue;      /* Computed Parameter: steer_cmd_InitialValue
                                        * Referenced by: '<S1>/steer_cmd'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S11>/Delay'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S11>/Delay1'
                                        */
  uint16_T ActLevel_InitialValue;      /* Computed Parameter: ActLevel_InitialValue
                                        * Referenced by: '<S1>/ActLevel'
                                        */
  uint16_T edg_count_InitialValue;     /* Computed Parameter: edg_count_InitialValue
                                        * Referenced by: '<S1>/edg_count'
                                        */
  uint16_T int_count_InitialValue;     /* Computed Parameter: int_count_InitialValue
                                        * Referenced by: '<S1>/int_count'
                                        */
  uint16_T ultra_edge2_cnt_InitialValue;/* Computed Parameter: ultra_edge2_cnt_InitialValue
                                         * Referenced by: '<S1>/ultra_edge2_cnt'
                                         */
  uint8_T IsParallelOk_InitialValue;   /* Computed Parameter: IsParallelOk_InitialValue
                                        * Referenced by: '<S1>/IsParallelOk'
                                        */
  uint8_T edg1_cnt_InitialValue;       /* Computed Parameter: edg1_cnt_InitialValue
                                        * Referenced by: '<S1>/edg1_cnt'
                                        */
  uint8_T edg1_stage_InitialValue;     /* Computed Parameter: edg1_stage_InitialValue
                                        * Referenced by: '<S1>/edg1_stage'
                                        */
  uint8_T edg2_cnt_InitialValue;       /* Computed Parameter: edg2_cnt_InitialValue
                                        * Referenced by: '<S1>/edg2_cnt'
                                        */
  uint8_T edgEndFlg_InitialValue;      /* Computed Parameter: edgEndFlg_InitialValue
                                        * Referenced by: '<S1>/edgEndFlg'
                                        */
  uint8_T edg_flg_InitialValue;        /* Computed Parameter: edg_flg_InitialValue
                                        * Referenced by: '<S1>/edg_flg'
                                        */
  uint8_T steerTurnCnt_InitialValue;   /* Computed Parameter: steerTurnCnt_InitialValue
                                        * Referenced by: '<S1>/steerTurnCnt'
                                        */
  uint8_T steerTurnCnt_old_InitialValue;/* Computed Parameter: steerTurnCnt_old_InitialValue
                                         * Referenced by: '<S1>/steerTurnCnt_old'
                                         */
};

/* Group of EEPROM*/
typedef struct {
    real_T carH;
    real_T carMSA;
    real_T carRmin;
    real_T carTread;
    real_T carW;
    real_T carl;
    real_T carL;
    real_T whsp_factor;
    real_T Lq;
    real_T Lq_s;
    real_T Lg1;
    real_T Lg2;
    real_T Lg3;
    real_T nC;
    real_T k;
    real_T Lim_L_backIn;
    real_T Lim_YawAngle_backIn;
    real_T c0;
    real_T g0;
    real_T k1;
    real_T pos_of_ultra;
    unsigned long due_time_parking;
}tag_EE_APGS;

/* Real-time Model Data Structure */
struct tag_RTM_APGS_T {
  const char_T * volatile errorStatus;
};

/* EE parameteres */
extern tag_EE_APGS APGS_EE;

/* Block parameters (auto storage) */
extern P_APGS_T APGS_P;

/* Block signals (auto storage) */
extern B_APGS_T APGS_B;

/* Block states (auto storage) */
extern DW_APGS_T APGS_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_APGS_T APGS_U;

/* Model entry point functions */
extern void APGS_initialize(void);
extern void APGS_step(void);
extern void APGS_terminate(void);

/* Real-time Model object */
extern RT_MODEL_APGS_T *const APGS_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'APGS'
 * '<S1>'   : 'APGS/SchedulerAPGS'
 * '<S2>'   : 'APGS/SchedulerAPGS/ISR'
 * '<S3>'   : 'APGS/SchedulerAPGS/TaskScheduler'
 * '<S4>'   : 'APGS/SchedulerAPGS/mainTask_function'
 * '<S5>'   : 'APGS/SchedulerAPGS/odometryTask_function'
 * '<S6>'   : 'APGS/SchedulerAPGS/steerCtrlTask_function'
 * '<S7>'   : 'APGS/SchedulerAPGS/mainTask_function/AGPS_Action'
 * '<S8>'   : 'APGS/SchedulerAPGS/mainTask_function/CalParkingPath_BackIn'
 * '<S9>'   : 'APGS/SchedulerAPGS/mainTask_function/CalParkingPath_Parallel'
 * '<S10>'  : 'APGS/SchedulerAPGS/mainTask_function/edgDetCalc'
 * '<S11>'  : 'APGS/SchedulerAPGS/mainTask_function/edgDetCalc1'
 * '<S12>'  : 'APGS/SchedulerAPGS/mainTask_function/parkingStrategy'
 * '<S13>'  : 'APGS/SchedulerAPGS/mainTask_function/AGPS_Action/APGS_FlowAction'
 * '<S14>'  : 'APGS/SchedulerAPGS/mainTask_function/CalParkingPath_BackIn/get_park_n_Fcn'
 * '<S15>'  : 'APGS/SchedulerAPGS/mainTask_function/CalParkingPath_Parallel/get_Rs_Fcn'
 * '<S16>'  : 'APGS/SchedulerAPGS/mainTask_function/edgDetCalc/edgDetCalc'
 * '<S17>'  : 'APGS/SchedulerAPGS/mainTask_function/edgDetCalc1/edgDetCalc'
 * '<S18>'  : 'APGS/SchedulerAPGS/mainTask_function/edgDetCalc1/loopForParking'
 * '<S19>'  : 'APGS/SchedulerAPGS/mainTask_function/parkingStrategy/BparkingFow'
 * '<S20>'  : 'APGS/SchedulerAPGS/mainTask_function/parkingStrategy/PparkingFlow'
 * '<S21>'  : 'APGS/SchedulerAPGS/mainTask_function/parkingStrategy/BparkingFow/BparkingFlowFcn'
 * '<S22>'  : 'APGS/SchedulerAPGS/mainTask_function/parkingStrategy/PparkingFlow/ParkingFlowControl'
 * '<S23>'  : 'APGS/SchedulerAPGS/odometryTask_function/getWheelSpeed'
 * '<S24>'  : 'APGS/SchedulerAPGS/odometryTask_function/getWheelSpeed/odometry'
 * '<S25>'  : 'APGS/SchedulerAPGS/steerCtrlTask_function/get_steer_cmd'
 * '<S26>'  : 'APGS/SchedulerAPGS/steerCtrlTask_function/get_steer_cmd/get_two_turn_cmd_fcn'
 */
#endif                                 /* RTW_HEADER_APGS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
