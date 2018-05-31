//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotx.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2018 17:30:16
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "ikine6s_full.h"
#include "rotx.h"

// Function Definitions

//
// Arguments    : double t
//                double R[9]
// Return Type  : void
//
void rotx(double t, double R[9])
{
  double ct;
  double st;
  int i3;
  static const signed char iv1[3] = { 1, 0, 0 };

  // ROTX Rotation about X axis
  //
  //  R = ROTX(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA  
  //  radians about the x-axis.
  //
  //  R = ROTX(THETA, 'deg') as above but THETA is in degrees.
  //
  //  See also ROTY, ROTZ, ANGVEC2R, ROT2, SO3.Rx.
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
  ct = std::cos(t);
  st = std::sin(t);
  for (i3 = 0; i3 < 3; i3++) {
    R[3 * i3] = iv1[i3];
  }

  R[1] = 0.0;
  R[4] = ct;
  R[7] = -st;
  R[2] = 0.0;
  R[5] = st;
  R[8] = ct;
}

//
// File trailer for rotx.cpp
//
// [EOF]
//
