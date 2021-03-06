#ifndef POLARGRAPH_h
#define POLARGRAPH_h

#include "grbl.h"

#define IN2MM 25.4f
#define MM2IN 1.0f/IN2MM

#define HOME_X 15.8125f*IN2MM // home tip position
#define HOME_Y -12.0f*IN2MM
#define HOME_Z 0.0f
#define ELL0_1 504.1981f   // associated belt length at home - these should be computed with ik(HOME_XY)
#define ELL0_2 1119.3f

#define TOOL_OFFSET_X 0.0f
#define TOOL_OFFSET_Y -4.125f*IN2MM; // mm

#define MOTOR_SPACING (54.75f*IN2MM) + (2.0f*44.0f)
#define PULLEY_RAD 20.0f

#define CANVAS_DIMS_W 36.0f*IN2MM
#define CANVAS_DIMS_H 72.0f*IN2MM
#define CANVAS_OFFSETS_X 11.0f*IN2MM
#define CANVAS_OFFSETS_Y 80.0f+PULLEY_RAD

void forward_kinematics_POLARGRAPH(float const *q_polargraph, float *tip_pos_cartesian);
void inverse_kinematics_POLARGRAPH(float const *cartesian, float *q_polargraph);
void ell2q(float *ell, float *q);
void q2ell(float *q, float *ell);

void msteps2q(int32_t *steps, float *q_polargraph);
void q2msteps(float *q_polargraph, int32_t *steps);

void polargraph_report_home_q(uint8_t idx);

#endif
