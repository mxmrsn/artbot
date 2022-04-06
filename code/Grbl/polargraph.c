#include "grbl.h"

float const L1 = SCARA_LINKAGE_1,
            L2 = SCARA_LINKAGE_2,
            L1_2 = sq(SCARA_LINKAGE_1),
            L2_2 = sq(SCARA_LINKAGE_2);

uint8_t angle_mode=false;
bool polargraph_home = false;						 

float ManualHomePos[3]={MANUAL_X_HOME_POS,MANUAL_Y_HOME_POS,MANUAL_Z_HOME_POS}; 

void forward_kinematics_POLARGRAPH(float const *q_polargraph, float *tip_pos_cartesian)
{
    float ell[3];
    ell = q2ell(q_polargraph, ell);
    ell[2] = MOTOR_SPACING;

    float thetas[3];
    float ell1 = ell[0]; float ell2 = ell[1]; float ell3 = ell[3];
    thetas[0] = acos( - (ell2^2 - ell1^2 - ell3^2) / (2*ell1*ell3) );
    thetas[1] = acos( - (ell1^2 - ell2^2 - ell3^2) / (2*ell2*ell3) );
    thetas[2] = acos( - (ell3^2 - ell1^2 - ell2^2) / (2*ell1*ell2) );

    tip_pos_cartesian[0] = ell1*cos(thetas[0]);
    tip_pos_cartesian[1] = -ell1*sin(thetas[0]);
}

void inverse_kinematics_POLARGRAPH(float const *tip_pos_cartesian, float *q_polargraph)
{
    float px, py;
    px = tip_pos_cartesian[0];
    py = tip_pos_cartesian[1];

    float ell1, px2, ell2;
    ell1 = sqrt(px^2 + py^2);
    px2 = (MOTOR_SPACING - px);
    ell2 = sqrt(px2^2 + py^2);

    float ell[2];
    ell[0] = ell1; ell[1] = ell2;

    q_polargraph = ell2q(ell, q_polargraph);
}

void ell2q(float ell1, float ell2, float *q) 
{
    q[0] = ( ell1 - ELL0_1 ) / PULLEY_RAD;
    q[1] = ( ell2 - ELL0_2 ) / PULLEY_RAD;
}

void q2ell(float *q, float *ell)
{
    ell[0] = ELL0_1 + PULLEY_RAD*q[0];
    ell[1] = ELL0_2 + PULLEY_RAD*q[1];
}