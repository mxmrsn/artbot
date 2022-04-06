/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

float const L1 = SCARA_LINKAGE_1,
            L2 = SCARA_LINKAGE_2,
            L1_2 = sq(SCARA_LINKAGE_1),
            L2_2 = sq(SCARA_LINKAGE_2);

uint8_t angle_mode=false;
bool scara_home = false;						 

float ManualHomePos[3]={MANUAL_X_HOME_POS,MANUAL_Y_HOME_POS,MANUAL_Z_HOME_POS}; 

void forward_kinematics_SCARA(float const *f_scara, float *cartesian)
{
    float l1x, l1y, l2x, l2y;
    
    l1x = cos(RADIANS(f_scara[X_AXIS])) * L1;
    l1y = sin(RADIANS(f_scara[X_AXIS])) * L1;
    
    l2x = cos(RADIANS(f_scara[X_AXIS] + f_scara[Y_AXIS])) * L2;
    l2y = sin(RADIANS(f_scara[X_AXIS] + f_scara[Y_AXIS])) * L2;

    cartesian[X_AXIS] = l1x + l2x - SCARA_OFFSET_X;
    cartesian[Y_AXIS] = l1y + l2y - SCARA_OFFSET_Y; 
    cartesian[Z_AXIS] = (float)f_scara[Z_AXIS];
}

void inverse_kinematics_SCARA(float const *cartesian, float *f_scara)
{
    float SCARA_pos[2];

    static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi;

    SCARA_pos[X_AXIS] = cartesian[X_AXIS] + SCARA_OFFSET_X;
    SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] + SCARA_OFFSET_Y;
    
    
    SCARA_C2 = (sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - L1_2 - L2_2)/(2*L1*L2);
    SCARA_S2 = -sqrtf(1 - sq(SCARA_C2));

    SCARA_K1 = L1 + L2*SCARA_C2;
    SCARA_K2 = L2*SCARA_S2;

    SCARA_theta = atan2f(SCARA_pos[Y_AXIS], SCARA_pos[X_AXIS]) - atan2f(SCARA_K2, SCARA_K1);
    SCARA_psi   = atan2f(SCARA_S2, SCARA_C2);
		
		if(!angle_mode)
		{
				f_scara[X_AXIS] = DEGREES(SCARA_theta);
				f_scara[Y_AXIS] = DEGREES(SCARA_psi);
				f_scara[Z_AXIS] = cartesian[Z_AXIS];
		}
		else
		{
				f_scara[X_AXIS] = cartesian[X_AXIS]; 
				f_scara[Y_AXIS] = cartesian[Y_AXIS]; 
				f_scara[Z_AXIS] = cartesian[Z_AXIS];
		}
    
/*     printPgmString(PSTR("x:"));
    printFloat(SCARA_pos[X_AXIS],2);
		printPgmString(PSTR("   y:"));
    printFloat(SCARA_pos[Y_AXIS],2);
		printPgmString(PSTR("\n"));		
    
    printPgmString(PSTR("q1:"));
    printFloat(f_scara[X_AXIS],2);
		printPgmString(PSTR("   q2:"));
    printFloat(f_scara[Y_AXIS],2);
		printPgmString(PSTR("\n"));	 */
}

void scara_report_positions() 
{
		uint8_t idx;
		static float position_scara[3];
		
    for (idx=0; idx<N_AXIS; idx++)
    {
        position_scara[idx] = system_convert_axis_steps_to_mpos(sys.position, idx);
    }
		printPgmString(PSTR("SCARA Theta:"));
    printFloat(position_scara[X_AXIS],2);
		printPgmString(PSTR("   Psi+Theta:"));
    printFloat(position_scara[Y_AXIS],2);
		printPgmString(PSTR("\n"));		

}

void scara_report_home_pos(uint8_t idx) 
{	
		static float position_scara;
		position_scara = system_convert_axis_steps_to_mpos(sys.position, idx);		
		switch(idx){
		case 0: 
				if (scara_home) {
					ManualHomePos[0] = position_scara;
					//settings.scara_theta = - position_scara;
					}
				printPgmString(PSTR("SCARA Psi:"));
                scara_home = false;				
				break;
		case 1: 
				position_scara = position_scara + 240; // +135
				if (scara_home) {
					    ManualHomePos[1] = position_scara;
						//settings.scara_psi = position_scara;
				    }
				printPgmString(PSTR("SCARA Theta:")); 
				break;
		}
		printFloat(position_scara,2);
        printPgmString(PSTR("\n"));			

}										
