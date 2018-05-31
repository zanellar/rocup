//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ikine6s_full.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2018 18:25:20
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>
#include "ikine6s_full.h"
#include "norm.h"
#include "sqrt.h"
#include "rotx.h"
#include "rotz.h"
#include "eye.h"
#include "acos.h"

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Robot parameters loading
// Parametri_Robot2kg
// Arguments    : const double T[16]
//                double q_init[6]
//                double out[6]
// Return Type  : void
//
int ikine6s_full(const double T[16], double q_init[6], double out[6])
{
  int i0;
  static const double dv0[6] = { 0.0, 1.5707963267948966, 0.0, 0.0, 0.0,
    1.5707963267948966 };

  double pc[3];
  static const double dv1[3] = { 0.0, 0.0, 0.32 };

  double q1[2];
  double pc1[3];
  double pc2[3];
  int i;
  double l_pc;
  double C21;
  double d0;
  double q4_idx_1;
  double q2;
  double R03[9];
  double out_arm[6];
  int idx;
  int ii_size_idx_0;
  boolean_T exitg1;
  int arm_index_size_idx_0;
  int return_value = 0;
  signed char ii_data[1];
  signed char arm_index_data[1];
  int out_arm_size[1];
  double y[9];
  int i1;
  double out_arm_data[1];
  static const double dv2[6] = { 1.5707963267948966, 0.0, -1.5707963267948966,
    1.5707963267948966, 1.5707963267948966, 0.0 };

  double dv3[9];
  double q5[2];
  double b_R03[9];
  double q6[4];
  double out_wrist[12];
  signed char wrist_index_data[1];
  boolean_T empty_non_axis_sizes;
  int result;
  static const double dh_offset[6] = { 0.0, 1.5707963267948966, 0.0, 0.0, 0.0,
    1.5707963267948966 };

  //  Closed-form Inverse Kinematics of the Bonmet 2kg robot
  //  Dimensioni robot Bonmet giallo:
  // L7  = 0.0;
  // dh_offset = [pi, pi-q20, -pi/2+q20, 0, pi, 0];
  for (i0 = 0; i0 < 6; i0++) {
    q_init[i0] += dv0[i0];
  }

  //  Limits settings:
  // n = 6;
  //  wrist-tool z-axis distance form wrist center (CoW)
  //  Cartesian position of the tool tip
  //  Vector from base center (intersectoin point between axis 1 and 2) to CoW
  for (i0 = 0; i0 < 3; i0++) {
    pc[i0] = (T[12 + i0] - T[8 + i0] * 0.0785) - dv1[i0];
  }

  //  base angle - first solution
  q1[0] = 3.1415926535897931 + rt_atan2d_snf(pc[1], pc[0]);

  // q1(1) = atan2(pc(2),pc(1));
  //  base angle - second solution
  q1[1] = 3.1415926535897931 + rt_atan2d_snf(-pc[1], -pc[0]);

  // q1(2) = atan2(-pc(2),-pc(1));
  if (3.1415926535897931 + rt_atan2d_snf(pc[1], pc[0]) < -2.9670597283903604) {
    q1[0] = 3.1415926535897931 + rt_atan2d_snf(-pc[1], -pc[0]);
  }

  if (q1[0] < -2.9670597283903604) {
    q1[1] = q1[0];
  }

  if (q1[0] > 2.9670597283903604) {
    q1[0] = q1[1];
  }

  if (q1[1] > 2.9670597283903604) {
    q1[0] = q1[1];
  }

  //  Distance from base center to CoW
  // l_pc = norm(pc)
  pc1[0] = pc[0] - 0.05 * std::cos(q1[0]);
  pc1[1] = pc[1] - 0.05 * std::sin(q1[0]);
  pc1[2] = pc[2];
  pc2[0] = pc[0] - 0.05 * std::cos(q1[1]);
  pc2[1] = pc[1] - 0.05 * std::sin(q1[1]);
  pc2[2] = pc[2];
  if (norm(pc1) > norm(pc2)) {
    for (i = 0; i < 3; i++) {
      pc[i] = pc2[i];
    }
  } else {
    for (i = 0; i < 3; i++) {
      pc[i] = pc1[i];
    }
  }

  l_pc = norm(pc);

  //  for i = 1:2
  //      if q1(i)>=2*pi
  //          q1(i) = q1(i) - 2*pi;
  //      end
  //  end
  //
  C21 = (0.094300999999999982 - (l_pc * l_pc + 0.0729)) / (-2.0 * l_pc * 0.27);

  if (fabs(C21)>1) return_value = -6;

  // q22(2) = atan2(pc(3),-sqrt(pc(1)^2+pc(2)^2));
  // q2 = zeros(8,1);
  d0 = pc[0] * pc[0] + pc[1] * pc[1];
  b_sqrt(&d0);
  q4_idx_1 = 1.0 - C21 * C21;
  b_sqrt(&q4_idx_1);
  q2 = rt_atan2d_snf(pc[2], d0) + rt_atan2d_snf(q4_idx_1, C21);

  // q2(2) = pi - (q22(1) - q21);
  // q2(3) = pi - (q22(2) + q21);
  // q2(4) = pi - (q22(2) - q21);
  // q2(5) = -pi - (q22(1) + q21);
  // q2(6) = -pi - (q22(1) - q21);
  // q2(7) = -pi - (q22(2) + q21);
  // q2(8) = -pi - (q22(2) - q21);
  // for i = 1:8
  if (q2 > 3.1415926535897931) {
    q2 = rtInf;
  }

  if (q2 < -0.6981317007977319) {
    q2 = rtMinusInf;
  }

  // end
  C21 = (l_pc * l_pc - 0.167201) / -0.16582572659270936;

  // q3 = zeros(2,1);
  d0 = 1.0 - C21 * C21;
  b_sqrt(&d0);
  q4_idx_1 = 0.97367280287315017;
  b_acos(&q4_idx_1);
  C21 = (rt_atan2d_snf(d0, C21) - 1.5707963267948966) - q4_idx_1;

  // q3(2) = -q3(1)
  // for i = 1:2
  if (C21 > 1.5707963267948966) {
    C21 = rtInf;
  }

  if (C21 < -1.2217304763960306) {
    C21 = rtMinusInf;
  }

  // end
  eye(R03);
  out_arm[0] = q1[0];
  out_arm[2] = q2;
  out_arm[4] = C21;
  out_arm[1] = q1[1];
  out_arm[3] = q2;
  out_arm[5] = C21;

  //  out_arm(3,:) = [q1(2) q2(3) q3(1)];
  //  out_arm(4,:) = [q1(2) q2(4) q3(2)];
  //  out_arm(5,:) = [q1(1) q2(5) q3(1)];
  //  out_arm(6,:) = [q1(1) q2(6) q3(2)];
  //  out_arm(7,:) = [q1(2) q2(7) q3(1)];
  //  out_arm(8,:) = [q1(2) q2(8) q3(2)];
  // out_arm
  for (i = 0; i < 2; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      pc[i0] = out_arm[i + (i0 << 1)] - q_init[i0];
    }

    q1[i] = b_norm(pc);
  }

  if ((q1[0] > q1[1]) || (rtIsNaN(q1[0]) && (!rtIsNaN(q1[1])))) {
    C21 = q1[1];
  } else {
    C21 = q1[0];
  }

  idx = 0;
  ii_size_idx_0 = 1;
  i = 1;
  exitg1 = false;
  while ((!exitg1) && (i < 3)) {
    if (q1[i - 1] == C21) {
      idx = 1;
      ii_data[0] = (signed char)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (idx == 0) {
    ii_size_idx_0 = 0;
  }

  arm_index_size_idx_0 = ii_size_idx_0;
  for (i0 = 0; i0 < ii_size_idx_0; i0++) {
    arm_index_data[i0] = ii_data[i0];
  }

  for (i = 0; i < 3; i++) {
    out_arm_size[0] = ii_size_idx_0;
    for (i0 = 0; i0 < ii_size_idx_0; i0++) {
      out_arm_data[i0] = out_arm[(arm_index_data[i0] + (i << 1)) - 1];
    }

    rotz(out_arm_data, out_arm_size, y);
    rotx(dv2[i], dv3);
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        b_R03[i0 + 3 * i1] = 0.0;
        for (idx = 0; idx < 3; idx++) {
          b_R03[i0 + 3 * i1] += R03[i0 + 3 * idx] * y[idx + 3 * i1];
        }
      }

      for (i1 = 0; i1 < 3; i1++) {
        R03[i0 + 3 * i1] = 0.0;
        for (idx = 0; idx < 3; idx++) {
          R03[i0 + 3 * i1] += b_R03[i0 + 3 * idx] * dv3[idx + 3 * i1];
        }
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      y[i0 + 3 * i1] = 0.0;
      for (idx = 0; idx < 3; idx++) {
        y[i0 + 3 * i1] += R03[idx + 3 * i0] * T[idx + (i1 << 2)];
      }
    }
  }

  l_pc = -rt_atan2d_snf(y[7], y[6]);
  q4_idx_1 = -rt_atan2d_snf(-y[7], -y[6]);
  d0 = y[6] * y[6] + y[7] * y[7];
  b_sqrt(&d0);
  q5[0] = rt_atan2d_snf(d0, y[8]);
  q5[1] = rt_atan2d_snf(-std::sqrt(y[6] * y[6] + y[7] * y[7]), -y[8]);

  // q5(3) = -atan2(-sqrt(a46x^2+a46y^2),a46z)
  // q5(4) = atan2(-sqrt(a46x^2+a46y^2),-a46z)
  for (i = 0; i < 2; i++) {
    C21 = q5[i];
    if (q5[i] > 2.0943951023931953) {
      C21 = rtInf;
    }

    if (C21 < -2.0943951023931953) {
      C21 = rtMinusInf;
    }

    q5[i] = C21;
  }

  for (i = 0; i < 4; i++) {
    q6[i] = 0.0;
  }

  q6[0] = -rt_atan2d_snf(y[5], y[2]);
  q6[1] = -rt_atan2d_snf(y[5], y[2]);

  // q6(1) = atan2(-s46z,n46z)
  // q6(2) = atan2(s46z,-n46z)
  for (i = 0; i < 2; i++) {
    C21 = q6[i];
    if (q6[i] > 7.8539816339744828) {
      C21 = rtInf;
    }

    if (C21 < -4.71238898038469) {
      C21 = rtMinusInf;
    }

    q6[i] = C21;
  }

  memset(&out_wrist[0], 0, 12U * sizeof(double));
  if (std::abs(q5[0]) < 1.0E-6) {
    out_wrist[0] = q_init[3];
    out_wrist[4] = q5[0];
    out_wrist[8] = q_init[5];
    ii_size_idx_0 = 1;
    wrist_index_data[0] = 1;
  } else {
    out_wrist[0] = l_pc;
    out_wrist[4] = q5[0];
    out_wrist[8] = q6[0];
    out_wrist[1] = l_pc;
    out_wrist[5] = q5[0];
    out_wrist[9] = q6[1];
    out_wrist[2] = q4_idx_1;
    out_wrist[6] = q5[1];
    out_wrist[10] = q6[0];
    out_wrist[3] = q4_idx_1;
    out_wrist[7] = q5[1];
    out_wrist[11] = q6[1];

    //      out_wrist(5,:) = [q4(1) q5(1) q6(3)];
    //      out_wrist(6,:) = [q4(1) q5(1) q6(3)];
    //      out_wrist(7,:) = [q4(2) q5(2) q6(4)];
    //      out_wrist(8,:) = [q4(2) q5(2) q6(4)];
    //      out_wrist(9,:) = [q4(1) q5(3) q6(1)];
    //      out_wrist(10,:) = [q4(1) q5(3) q6(2)];
    //      out_wrist(11,:) = [q4(1) q5(4) q6(1)];
    //      out_wrist(12,:) = [q4(1) q5(4) q6(2)];
    //      out_wrist(13,:) = [q4(2) q5(3) q6(1)];
    //      out_wrist(14,:) = [q4(2) q5(3) q6(2)];
    //      out_wrist(15,:) = [q4(2) q5(4) q6(1)];
    //      out_wrist(16,:) = [q4(2) q5(4) q6(2)];
    for (i = 0; i < 4; i++) {
      for (i0 = 0; i0 < 3; i0++) {
        pc[i0] = out_wrist[i + (i0 << 2)] - q_init[3 + i0];
      }

      q6[i] = b_norm(pc);
    }

    if (!rtIsNaN(q6[0])) {
      idx = 1;
    } else {
      idx = 0;
      i = 2;
      exitg1 = false;
      while ((!exitg1) && (i < 5)) {
        if (!rtIsNaN(q6[i - 1])) {
          idx = i;
          exitg1 = true;
        } else {
          i++;
        }
      }
    }

    if (idx == 0) {
      C21 = q6[0];
    } else {
      C21 = q6[idx - 1];
      while (idx + 1 < 5) {
        if (C21 > q6[idx]) {
          C21 = q6[idx];
        }

        idx++;
      }
    }

    idx = 0;
    ii_size_idx_0 = 1;
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i < 5)) {
      if (q6[i - 1] == C21) {
        idx = 1;
        ii_data[0] = (signed char)i;
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (idx == 0) {
      ii_size_idx_0 = 0;
    }

    for (i0 = 0; i0 < ii_size_idx_0; i0++) {
      wrist_index_data[i0] = ii_data[i0];
    }
  }

  if (!(arm_index_size_idx_0 == 0)) {
    idx = 1;
  } else if (!(ii_size_idx_0 == 0)) {
    idx = 1;
  } else {
    idx = 0;
  }

  empty_non_axis_sizes = (idx == 0);
  if (empty_non_axis_sizes || (!(arm_index_size_idx_0 == 0))) {
    i = 3;
  } else {
    i = 0;
  }

  if (empty_non_axis_sizes || (!(ii_size_idx_0 == 0))) {
    result = 3;
  } else {
    result = 0;
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < arm_index_size_idx_0; i1++) {
      pc[i1 + arm_index_size_idx_0 * i0] = out_arm[(arm_index_data[i1] + (i0 <<
        1)) - 1];
    }

    for (i1 = 0; i1 < ii_size_idx_0; i1++) {
      pc1[i1 + ii_size_idx_0 * i0] = out_wrist[(wrist_index_data[i1] + (i0 << 2))
        - 1];
    }
  }

  for (i0 = 0; i0 < i; i0++) {
    for (i1 = 0; i1 < idx; i1++) {
      out_arm[i1 + idx * i0] = pc[i1 + idx * i0];
    }
  }

  for (i0 = 0; i0 < result; i0++) {
    for (i1 = 0; i1 < idx; i1++) {
      out_arm[i1 + idx * (i0 + i)] = pc1[i1 + idx * i0];
    }
  }

  for (i = 0; i < 6; i++) {
    C21 = out_arm[i] - dh_offset[i];
    if (C21 > 3.1415926535897931) {
      C21 -= 6.2831853071795862;
    }

    if (C21 < -3.1415926535897931) {
      C21 += 6.2831853071795862;
    }

    out[i] = C21;
  }

  for (i = 0; i < 6; i++){
    if (rtIsInf(out[i]) || rtIsNaN(out[i])) return_value = -6;
  } 

  if(return_value!=0)
    for (i = 0; i < 6; i++) out[i] = q_init[i];

  return return_value;

}

//
// File trailer for ikine6s_full.cpp
//
// [EOF]
//
