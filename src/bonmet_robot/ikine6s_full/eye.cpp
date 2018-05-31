//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2018 12:07:21
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "ikine6s_full.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[9]
// Return Type  : void
//
void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

void eye4x4(double I[16])
{
  int k;
  memset(&I[0], 0, 16U * sizeof(double));
  for (k = 0; k < 4; k++) {
    I[k + 4 * k] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
