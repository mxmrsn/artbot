#ifndef POLARGRAPH_h
#define POLARGRAPH_h

#include "grbl.h"

#ifdef POLARGRAPH
	#define IS_POLARGRAPH true
#endif

#define MM2IN 25.4
#define IN2MM 1.0/MM2IN

#define ELL0_1 800.0
#define ELL0_2 800.0
#define MOTOR_SPACING 1500.0
#define PULLEY_RAD 20.0

#define CANVAS DIMS_W 36.0*MM2IN
#define CANVAS_DIMS_H 72.0*MM2IN
#define CANVAS_OFFSETS_X 300.0
#define CANVAS_OFFSETS_Y 100.0
			   
void inverse_kinematics_POLARGRAPH(float const *cartesian, float *q_polargraph);
void forward_kinematics_POLARGRAPH(float const *q_polargraph, float *cartesian);
void ell2q(float ell1, float ell2, float *q);
void q2ell(float *q, float *ell);

#endif