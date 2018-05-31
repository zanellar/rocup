//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotz.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2018 17:30:16
//

// Include Files
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "ikine6s_full.h"
#include "rotz.h"

// Function Definitions

//
// Arguments    : const double t_data[]
//                const int t_size[1]
//                double R[9]
// Return Type  : void
//
void rotz(const double t_data[], const int t_size[1], double R[9])
{
  int ct_size_idx_0;
  int loop_ub;
  double ct_data[1];
  double st_data[1];
  int i2;
  static const signed char iv0[3] = { 0, 0, 1 };

  // ROTZ Rotation about Z axis
  //
  //  R = ROTZ(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA  
  //  radians about the z-axis.
  //
  //  R = ROTZ(THETA, 'deg') as above but THETA is in degrees.
  //
  //  See also ROTX, ROTY, ANGVEC2R, ROT2, SO3.Rx.
  //  Copyright (C) 1993-2017, by Peter I. Corke
  //
  //  This file is part of The Robotics Toolbox for MATLAB (RTB).
  //
  //  RTB is free software: you can redistribute it and/or modify
  //  it under the terms of the GNU Lesser General Public License as published by 
  //  the Free Software Foundation, either version 3 of the License, or
  //  (at your option) any later version.
  //
  //  RTB is distributed in the hope that it will be useful,
  //  but WITHOUT ANY WARRANTY; without even the implied warranty of
  //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  //  GNU Lesser General Public License for more details.
  //
  //  You should have received a copy of the GNU Leser General Public License
  //  along with RTB.  If not, see <http://www.gnu.org/licenses/>.
  //
  //  http://www.petercorke.com
  ct_size_idx_0 = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    memcpy(&ct_data[0], &t_data[0], (unsigned int)(loop_ub * (int)sizeof(double)));
  }

  loop_ub = 1;
  while (loop_ub <= t_size[0]) {
    ct_data[0] = std::cos(ct_data[0]);
    loop_ub = 2;
  }

  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    memcpy(&st_data[0], &t_data[0], (unsigned int)(loop_ub * (int)sizeof(double)));
  }

  loop_ub = 1;
  while (loop_ub <= t_size[0]) {
    st_data[0] = std::sin(st_data[0]);
    loop_ub = 2;
  }

  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    memcpy(&R[0], &ct_data[0], (unsigned int)(loop_ub * (int)sizeof(double)));
  }

  loop_ub = t_size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    R[3 + i2] = -st_data[i2];
  }

  R[6] = 0.0;
  loop_ub = t_size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    R[i2 + ct_size_idx_0] = st_data[i2];
  }

  loop_ub = t_size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    R[3 + (i2 + ct_size_idx_0)] = ct_data[i2];
  }

  R[6 + t_size[0]] = 0.0;
  for (i2 = 0; i2 < 3; i2++) {
    R[(ct_size_idx_0 + t_size[0]) + 3 * i2] = iv0[i2];
  }
}

//
// File trailer for rotz.cpp
//
// [EOF]
//
