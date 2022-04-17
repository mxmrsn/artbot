/
#ifndef SCARA_h
#define SCARA_h

#include "grbl.h"

// #ifdef SCARA
// 	#define IS_SCARA true
// #endif

// Length of inner and outer support arms. Measure arm lengths precisely.
#define SCARA_LINKAGE_1 160.0f	// 160mm
#define SCARA_LINKAGE_2 145.0f 	// 160mm

// SCARA tower offset (position of Tower relative to bed zero position)
// This needs to be reasonably accurate as it defines the printbed position in the SCARA space.
#define SCARA_OFFSET_X 0.0f 	// -245mm
#define SCARA_OFFSET_Y 0.0f	// 85mm

#define MANUAL_X_HOME_POS 0.0f	// Theta
#define MANUAL_Y_HOME_POS 0.0f 	// PSI
#define MANUAL_Z_HOME_POS 0.0f

#define RADIANS(d) ((d)*(float)M_PI/180.0f)
#define DEGREES(r) ((r)*180.0f/(float)M_PI)

#define sq(x) x*x

extern float ManualHomePos[3];
extern bool scara_home;
void inverse_kinematics_SCARA(float const *cartesian, float *f_scara);
void forward_kinematics_SCARA(float const *f_scara, float *cartesian);
void scara_report_positions(void) ;

#endif
