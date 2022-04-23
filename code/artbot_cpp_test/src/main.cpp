#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <functional>

void timeit(std::function<void()> func) {
    std::clock_t start = std::clock();

    func();

    double ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000.0);

    std::cout << "Finished in " << ms << "ms" << std::endl;
}

void forward_kinematics_POLARGRAPH(float *q_polargraph, float *tip_pos_cartesian)
{
    float ell[3];
    q2ell(q_polargraph, ell);

    float thetas[3];
    float ell1 = ell[0]; float ell2 = ell[1]; float ell3 = ell[2];
    thetas[0] = acosf( - (pow(ell2,2) - pow(ell1,2) - pow(ell3,2)) / (2*ell1*ell3) );
    thetas[1] = acosf( - (pow(ell1,2) - pow(ell2,2) - pow(ell3,2)) / (2*ell2*ell3) );
    thetas[2] = acosf( - (pow(ell3,2) - pow(ell1,2) - pow(ell2,2)) / (2*ell1*ell2) );

    tip_pos_cartesian[0] = ell1*cos(thetas[0]) + TOOL_OFFSET_X;
    tip_pos_cartesian[1] = -ell1*sin(thetas[0]) + TOOL_OFFSET_Y; // y-axis oriented downwards

    // printFloat_CoordValue(tip_pos_cartesian[0]);
    // printFloat_CoordValue(tip_pos_cartesian[1]);
}

void inverse_kinematics_POLARGRAPH(float *tip_pos_cartesian, float *q_polargraph)
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

int main () {

  timeit([] {
    for (int ii=0; ii<10000; ii++) {
      std::cout<<"ii = "<<ii<<std::endl;
    }
  });

  float q_polargraph[3] = {0.0,0.0,0.0};
  float p_tip[3] = {0.0,0.0,0.0};
  forward_kinematics_POLARGRAPH(q_polargraph, p_tip);
  Eigen::Matrix<double,3,1> q, p;
  q << q_polargraph[0], q_polargraph[1], q_polargraph[2];
  p << p_tip[0], p_tip[1], p_tip[2];

  std::cout << q.transpose() << std::endl;
  std::cout << p.transpose() << std::endl;

  std::cout<<"Main Loop Complete!"<<std::endl;

	// if(LOG_DATA_ON && log_file.is_open()) { log_file.close(); }
	return 0;
}
