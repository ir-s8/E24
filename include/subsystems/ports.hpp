#ifndef __PORTS_H__
#define __PORTS_H__

//----------drive----------
#define RIGHT_FRONT     1
#define RIGHT_MIDDLE    7
#define RIGHT_BACK      8

#define LEFT_FRONT      -10
#define LEFT_MIDDLE     -6
#define LEFT_BACK       -11
//-------------------------

//----------intake---------
#define INTAKE_PORT     4
//-------------------------

//----------lb-mech---------
#define LB_PORT         5
//-------------------------

//---------sensors----------
#define IMU_PORT        12
//--------------------------

//---------pneumatics----------
#define CLAMP_ADI       'A'
#define AUTONM_ADI      'D'
//-----------------------------

//-------------------------buttons----------------------------
#define BUTTON_INTAKE           pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE          pros::E_CONTROLLER_DIGITAL_R2

#define BUTTON_LB_OUT           pros::E_CONTROLLER_DIGITAL_L1
#define BUTTON_LB_IN            pros::E_CONTROLLER_DIGITAL_L2

#define CLAMP_ACT               pros::E_CONTROLLER_DIGITAL_B
#define AUTONM_ACT              pros::E_CONTROLLER_DIGITAL_Y
//------------------------------------------------------------

#endif
