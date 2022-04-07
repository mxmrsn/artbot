#ifndef POLARGRAPH_h
#define POLARGRAPH_h

#include "grbl.h"

#ifdef POLARGRAPH
	#define IS_POLARGRAPH true
#endif

#define MM2IN 25.4f
#define IN2MM 1.0f/MM2IN

#define ELL0_1 800.0f
#define ELL0_2 800.0f
#define MOTOR_SPACING 1500.0f
#define PULLEY_RAD 20.0f

#define CANVAS DIMS_W 36.0f*MM2IN
#define CANVAS_DIMS_H 72.0f*MM2IN
#define CANVAS_OFFSETS_X 300.0f
#define CANVAS_OFFSETS_Y 100.0f
			   
void inverse_kinematics_POLARGRAPH(float const *cartesian, float *q_polargraph);
void forward_kinematics_POLARGRAPH(float const *q_polargraph, float *cartesian);
void ell2q(float ell1, float ell2, float *q);
void q2ell(float *q, float *ell);

#endif