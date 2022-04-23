#include "grbl.h"

uint8_t angle_mode_pol = false;
bool polargraph_home = false;
float home_pos[3]={HOME_X,HOME_Y,HOME_Z};

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

    // printFloat_CoordValue(tip_pos_cartesian[0]);
    // printFloat_CoordValue(tip_pos_cartesian[1]);
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

    if(!angle_mode_pol) // task space control (ik)
		{
        q_polargraph[Z_AXIS] = tip_pos_cartesian[Z_AXIS];
		}
		else // joint space control
		{
				q_polargraph[X_AXIS] = tip_pos_cartesian[X_AXIS]; // drive joints directly
				q_polargraph[Y_AXIS] = tip_pos_cartesian[Y_AXIS];
				q_polargraph[Z_AXIS] = tip_pos_cartesian[Z_AXIS];
		}
    // printFloat_CoordValue(q_polargraph[0]);
    // printFloat_CoordValue(q_polargraph[1]);
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

// TODO: finish this implementation; to convert motor steps to q
void msteps2q(int32_t *msteps, float *q_polargraph)
{
  for (uint8_t idx=0; idx<N_AXIS; idx++)
  {
      q_polargraph[idx] = msteps[idx]/settings.steps_per_mm[idx];
  }
}

void q2msteps(float *q_polargraph, int32_t *msteps)
{
  for (uint8_t idx=0; idx<N_AXIS; idx++)
  {
    msteps[idx] = q_polargraph[idx]*settings.steps_per_mm[idx];
  }
}

void polargraph_report_home_q(uint8_t idx)
{
		static float q_polargraph;
		q2msteps(sys.position, idx);
		switch(idx){
		case 0:
				if (polargraph_home) {
					home_pos[0] = q_polargraph;
					}
				printPgmString(PSTR("POLARGRAPH Theta 1:"));
                polargraph_home = false;
				break;
		case 1:
				// q_polargraph = q_polargraph + 240; // +135
				if (polargraph_home) {
					    home_pos[1] = q_polargraph;
				    }
				printPgmString(PSTR("POLARGRAPH Theta 2:"));
				break;
		}
		printFloat(q_polargraph,2);
        printPgmString(PSTR("\n"));

}
