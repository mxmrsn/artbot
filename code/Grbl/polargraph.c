#include "grbl.h"

void forward_kinematics_POLARGRAPH(float const *q_polargraph, float *tip_pos_cartesian)
{
    float ell[3];
    q2ell(q_polargraph, ell);

    float thetas[3];
    float ell1 = ell[0]; float ell2 = ell[1]; float ell3 = ell[3];
    thetas[0] = acos( - (pow(ell2,2) - pow(ell1,2) - pow(ell3,2)) / (2*ell1*ell3) );
    thetas[1] = acos( - (pow(ell1,2) - pow(ell2,2) - pow(ell3,2)) / (2*ell2*ell3) );
    thetas[2] = acos( - (pow(ell3,2) - pow(ell1,2) - pow(ell2,2)) / (2*ell1*ell2) );

    tip_pos_cartesian[0] = ell1*cos(thetas[0]) + TOOL_OFFSET_X;
    tip_pos_cartesian[1] = -ell1*sin(thetas[0]) + TOOL_OFFSET_Y; // y-axis oriented downwards
}

void inverse_kinematics_POLARGRAPH(float const *tip_pos_cartesian, float *q_polargraph)
{
    float px, py, px2;
    px = tip_pos_cartesian[0] - TOOL_OFFSET_X;
    py = tip_pos_cartesian[1] - TOOL_OFFSET_Y;
    px2 = (MOTOR_SPACING - px);

    float ell[2];
    ell[0] = sqrt(pow(px,2) + pow(py,2));
    ell[1] = sqrt(pow(px2,2) + pow(py,2));

    ell2q(ell, q_polargraph); // radians
}

void ell2q(float *ell, float *q)
{
    q[0] = ( ell[0] - ELL0_1 ) / PULLEY_RAD;
    q[1] = ( ell[1] - ELL0_2 ) / PULLEY_RAD;
}

void q2ell(float *q, float *ell)
{
    ell[0] = ELL0_1 + PULLEY_RAD*q[0];
    ell[1] = ELL0_2 + PULLEY_RAD*q[1];
    ell[2] = MOTOR_SPACING;
}
