//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: get_Ja_dot.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 29-Jan-2021 10:48:41
//

// Include Files
#include <utils/get_Ja_dot.h> 
#include <cmath>

// Function Definitions
//
// Arguments    : const double q[7]
//                const double dq[7]
//                double Ja_dot[21]
// Return Type  : void
//

void get_Ja_dot(const double q[7], const double dq[7], double Ja_dot[21])
{
  double Ja_dot_tmp;
  double Ja_dot_tmp_tmp;
  double Ja_dot_tmp_tmp_tmp;
  double Ja_dot_tmp_tmp_tmp_tmp;
  double ab_Ja_dot_tmp_tmp;
  double b_Ja_dot_tmp;
  double b_Ja_dot_tmp_tmp;
  double b_Ja_dot_tmp_tmp_tmp;
  double b_Ja_dot_tmp_tmp_tmp_tmp;
  double bb_Ja_dot_tmp_tmp;
  double c_Ja_dot_tmp;
  double c_Ja_dot_tmp_tmp;
  double c_Ja_dot_tmp_tmp_tmp;
  double c_Ja_dot_tmp_tmp_tmp_tmp;
  double cb_Ja_dot_tmp_tmp;
  double d_Ja_dot_tmp;
  double d_Ja_dot_tmp_tmp;
  double d_Ja_dot_tmp_tmp_tmp;
  double db_Ja_dot_tmp_tmp;
  double e_Ja_dot_tmp;
  double e_Ja_dot_tmp_tmp;
  double e_Ja_dot_tmp_tmp_tmp;
  double eb_Ja_dot_tmp_tmp;
  double f_Ja_dot_tmp;
  double f_Ja_dot_tmp_tmp;
  double f_Ja_dot_tmp_tmp_tmp;
  double fb_Ja_dot_tmp_tmp;
  double g_Ja_dot_tmp;
  double g_Ja_dot_tmp_tmp;
  double g_Ja_dot_tmp_tmp_tmp;
  double h_Ja_dot_tmp;
  double h_Ja_dot_tmp_tmp;
  double h_Ja_dot_tmp_tmp_tmp;
  double i_Ja_dot_tmp;
  double i_Ja_dot_tmp_tmp;
  double i_Ja_dot_tmp_tmp_tmp;
  double j_Ja_dot_tmp;
  double j_Ja_dot_tmp_tmp;
  double j_Ja_dot_tmp_tmp_tmp;
  double k_Ja_dot_tmp;
  double k_Ja_dot_tmp_tmp;
  double k_Ja_dot_tmp_tmp_tmp;
  double l_Ja_dot_tmp;
  double l_Ja_dot_tmp_tmp;
  double m_Ja_dot_tmp;
  double m_Ja_dot_tmp_tmp;
  double n_Ja_dot_tmp;
  double n_Ja_dot_tmp_tmp;
  double o_Ja_dot_tmp;
  double o_Ja_dot_tmp_tmp;
  double p_Ja_dot_tmp;
  double p_Ja_dot_tmp_tmp;
  double q_Ja_dot_tmp_tmp;
  double r_Ja_dot_tmp_tmp;
  double s_Ja_dot_tmp_tmp;
  double t_Ja_dot_tmp_tmp;
  double u_Ja_dot_tmp_tmp;
  double v_Ja_dot_tmp_tmp;
  double w_Ja_dot_tmp_tmp;
  double x_Ja_dot_tmp_tmp;
  double y_Ja_dot_tmp_tmp;
  Ja_dot_tmp_tmp = std::cos(q[2]);
  b_Ja_dot_tmp_tmp = std::sin(q[0]);
  c_Ja_dot_tmp_tmp = std::sin(q[2]);
  d_Ja_dot_tmp_tmp = std::cos(q[0]);
  e_Ja_dot_tmp_tmp = std::cos(q[1]);
  f_Ja_dot_tmp_tmp = std::cos(q[3]);
  g_Ja_dot_tmp_tmp = std::sin(q[1]);
  h_Ja_dot_tmp_tmp = std::sin(q[3]);
  Ja_dot_tmp = std::cos(q[5]);
  i_Ja_dot_tmp_tmp = std::cos(q[4]);
  j_Ja_dot_tmp_tmp = std::sin(q[4]);
  b_Ja_dot_tmp = std::sin(q[5]);
  k_Ja_dot_tmp_tmp = d_Ja_dot_tmp_tmp * Ja_dot_tmp_tmp;
  Ja_dot_tmp_tmp_tmp = e_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp;
  l_Ja_dot_tmp_tmp = Ja_dot_tmp_tmp_tmp * c_Ja_dot_tmp_tmp;
  m_Ja_dot_tmp_tmp = d_Ja_dot_tmp_tmp * e_Ja_dot_tmp_tmp;
  n_Ja_dot_tmp_tmp = Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp;
  b_Ja_dot_tmp_tmp_tmp = m_Ja_dot_tmp_tmp * Ja_dot_tmp_tmp;
  c_Ja_dot_tmp_tmp_tmp = n_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  d_Ja_dot_tmp_tmp_tmp = b_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp;
  o_Ja_dot_tmp_tmp = (((d_Ja_dot_tmp_tmp_tmp * dq[0] - k_Ja_dot_tmp_tmp * dq[2])
                       + c_Ja_dot_tmp_tmp_tmp * dq[1]) + l_Ja_dot_tmp_tmp * dq[2])
    - b_Ja_dot_tmp_tmp_tmp * dq[0];
  e_Ja_dot_tmp_tmp_tmp = d_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp;
  Ja_dot_tmp_tmp_tmp_tmp = e_Ja_dot_tmp_tmp * Ja_dot_tmp_tmp;
  f_Ja_dot_tmp_tmp_tmp = Ja_dot_tmp_tmp_tmp_tmp * b_Ja_dot_tmp_tmp;
  p_Ja_dot_tmp_tmp = e_Ja_dot_tmp_tmp_tmp + f_Ja_dot_tmp_tmp_tmp;
  g_Ja_dot_tmp_tmp_tmp = f_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  q_Ja_dot_tmp_tmp = h_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp;
  c_Ja_dot_tmp = q_Ja_dot_tmp_tmp - g_Ja_dot_tmp_tmp_tmp;
  d_Ja_dot_tmp = k_Ja_dot_tmp_tmp - l_Ja_dot_tmp_tmp;
  b_Ja_dot_tmp_tmp_tmp_tmp = b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  h_Ja_dot_tmp_tmp_tmp = b_Ja_dot_tmp_tmp_tmp_tmp * h_Ja_dot_tmp_tmp;
  r_Ja_dot_tmp_tmp = f_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp;
  e_Ja_dot_tmp = r_Ja_dot_tmp_tmp + h_Ja_dot_tmp_tmp_tmp;
  i_Ja_dot_tmp_tmp_tmp = d_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  c_Ja_dot_tmp_tmp_tmp_tmp = e_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp;
  j_Ja_dot_tmp_tmp_tmp = c_Ja_dot_tmp_tmp_tmp_tmp * b_Ja_dot_tmp_tmp;
  r_Ja_dot_tmp_tmp = (((h_Ja_dot_tmp_tmp * o_Ja_dot_tmp_tmp - r_Ja_dot_tmp_tmp *
                        dq[3]) + i_Ja_dot_tmp_tmp_tmp * dq[0]) +
                      j_Ja_dot_tmp_tmp_tmp * dq[1]) - h_Ja_dot_tmp_tmp_tmp * dq
    [3];
  s_Ja_dot_tmp_tmp = std::cos(q[0]) * std::cos(q[2]) - std::cos(q[1]) * std::sin
    (q[0]) * std::sin(q[2]);
  t_Ja_dot_tmp_tmp = std::cos(q[3]) * (std::cos(q[0]) * std::sin(q[2]) + std::
    cos(q[1]) * std::cos(q[2]) * std::sin(q[0])) + std::sin(q[0]) * std::sin(q[1])
    * std::sin(q[3]);
  u_Ja_dot_tmp_tmp = m_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp;
  v_Ja_dot_tmp_tmp = d_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  w_Ja_dot_tmp_tmp = v_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp;
  x_Ja_dot_tmp_tmp = Ja_dot_tmp_tmp_tmp * h_Ja_dot_tmp_tmp;
  y_Ja_dot_tmp_tmp = b_Ja_dot_tmp_tmp_tmp_tmp * c_Ja_dot_tmp_tmp;
  ab_Ja_dot_tmp_tmp = (((n_Ja_dot_tmp_tmp * dq[0] + e_Ja_dot_tmp_tmp_tmp * dq[2])
                        + u_Ja_dot_tmp_tmp * dq[0]) + f_Ja_dot_tmp_tmp_tmp * dq
                       [2]) - y_Ja_dot_tmp_tmp * dq[1];
  q_Ja_dot_tmp_tmp = (((w_Ja_dot_tmp_tmp * dq[0] - q_Ja_dot_tmp_tmp * dq[3]) -
                       f_Ja_dot_tmp_tmp * o_Ja_dot_tmp_tmp) + x_Ja_dot_tmp_tmp *
                      dq[1]) + g_Ja_dot_tmp_tmp_tmp * dq[3];
  bb_Ja_dot_tmp_tmp = i_Ja_dot_tmp_tmp * s_Ja_dot_tmp_tmp;
  cb_Ja_dot_tmp_tmp = j_Ja_dot_tmp_tmp * t_Ja_dot_tmp_tmp;
  f_Ja_dot_tmp = ((i_Ja_dot_tmp_tmp * q_Ja_dot_tmp_tmp - j_Ja_dot_tmp_tmp *
                   ab_Ja_dot_tmp_tmp) + bb_Ja_dot_tmp_tmp * dq[4]) -
    cb_Ja_dot_tmp_tmp * dq[4];
  s_Ja_dot_tmp_tmp *= j_Ja_dot_tmp_tmp;
  t_Ja_dot_tmp_tmp *= i_Ja_dot_tmp_tmp;
  g_Ja_dot_tmp = s_Ja_dot_tmp_tmp + t_Ja_dot_tmp_tmp;
  Ja_dot[0] = (((((((((((((((((((((((107.0 * Ja_dot_tmp * r_Ja_dot_tmp_tmp /
    1000.0 - 11.0 * Ja_dot_tmp * (((i_Ja_dot_tmp_tmp * ((((d_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[0] - h_Ja_dot_tmp_tmp *
    p_Ja_dot_tmp_tmp * dq[3]) - f_Ja_dot_tmp_tmp * o_Ja_dot_tmp_tmp) + std::cos
    (q[1]) * std::sin(q[0]) * h_Ja_dot_tmp_tmp * dq[1]) + f_Ja_dot_tmp_tmp *
    b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[3]) - j_Ja_dot_tmp_tmp *
    ((((n_Ja_dot_tmp_tmp * dq[0] + e_Ja_dot_tmp_tmp_tmp * dq[2]) +
    m_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[0]) + f_Ja_dot_tmp_tmp_tmp * dq[2])
    - std::sin(q[0]) * std::sin(q[1]) * c_Ja_dot_tmp_tmp * dq[1])) +
    i_Ja_dot_tmp_tmp * d_Ja_dot_tmp * dq[4]) - j_Ja_dot_tmp_tmp * e_Ja_dot_tmp *
    dq[4]) / 125.0) - 107.0 * b_Ja_dot_tmp * f_Ja_dot_tmp / 1000.0) - 33.0 *
    f_Ja_dot_tmp_tmp * o_Ja_dot_tmp_tmp / 400.0) - 48.0 * h_Ja_dot_tmp_tmp *
    o_Ja_dot_tmp_tmp / 125.0) - 11.0 * b_Ja_dot_tmp * r_Ja_dot_tmp_tmp / 125.0)
    + 48.0 * f_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp * dq[3] / 125.0) - 33.0 *
    h_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp * dq[3] / 400.0) + 11.0 * std::cos(q[5])
    * c_Ja_dot_tmp * dq[5] / 125.0) + 107.0 * std::sin(q[5]) * c_Ja_dot_tmp *
    dq[5] / 1000.0) - 33.0 * d_Ja_dot_tmp_tmp * Ja_dot_tmp_tmp * dq[2] / 400.0)
    - 79.0 * d_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[0] / 250.0) - 79.0 *
    e_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp * dq[1] / 250.0) + 33.0 *
    b_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[0] / 400.0) - 107.0 * std::cos(q[5])
                        * (j_Ja_dot_tmp_tmp * d_Ja_dot_tmp + i_Ja_dot_tmp_tmp *
    e_Ja_dot_tmp) * dq[5] / 1000.0) + 11.0 * std::sin(q[5]) * g_Ja_dot_tmp * dq
                       [5] / 125.0) - 48.0 * d_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp
                      * g_Ja_dot_tmp_tmp * dq[0] / 125.0) - 48.0 *
                     e_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp *
                     dq[1] / 125.0) + 33.0 * std::cos(q[0]) * g_Ja_dot_tmp_tmp *
                    h_Ja_dot_tmp_tmp * dq[0] / 400.0) + 33.0 * Ja_dot_tmp_tmp *
                   b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[1] / 400.0) + 33.0 *
                  e_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[2]
                  / 400.0) + 33.0 * std::cos(q[1]) * std::sin(q[0]) *
                 h_Ja_dot_tmp_tmp * dq[1] / 400.0) + 33.0 * std::cos(q[3]) *
                b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[3] / 400.0) + 48.0 *
               b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3] /
               125.0) - 33.0 * std::cos(q[0]) * e_Ja_dot_tmp_tmp *
    Ja_dot_tmp_tmp * dq[0] / 400.0;
  Ja_dot_tmp_tmp_tmp = m_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp;
  db_Ja_dot_tmp_tmp = k_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  h_Ja_dot_tmp = Ja_dot_tmp_tmp_tmp - db_Ja_dot_tmp_tmp;
  k_Ja_dot_tmp_tmp_tmp = m_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp;
  b_Ja_dot_tmp_tmp_tmp_tmp = k_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  eb_Ja_dot_tmp_tmp = b_Ja_dot_tmp_tmp_tmp_tmp * h_Ja_dot_tmp_tmp;
  i_Ja_dot_tmp = k_Ja_dot_tmp_tmp_tmp + eb_Ja_dot_tmp_tmp;
  j_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]);
  v_Ja_dot_tmp_tmp *= c_Ja_dot_tmp_tmp;
  k_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::cos(q[2]);
  l_Ja_dot_tmp = 48.0 * std::cos(q[0]) * std::cos(q[3]) * std::sin(q[1]);
  m_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3]);
  n_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::cos(q[1]);
  fb_Ja_dot_tmp_tmp = 48.0 * std::cos(q[0]) * e_Ja_dot_tmp_tmp;
  o_Ja_dot_tmp = 33.0 * std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]);
  Ja_dot[3] = (((((((((((((((((((((((((107.0 * std::cos(q[5]) *
    ((((((j_Ja_dot_tmp_tmp_tmp * dq[0] + i_Ja_dot_tmp_tmp_tmp * dq[1]) +
    m_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3]) - b_Ja_dot_tmp_tmp_tmp *
    h_Ja_dot_tmp_tmp * dq[1]) - k_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * dq[3]) + c_Ja_dot_tmp_tmp_tmp * h_Ja_dot_tmp_tmp * dq[0])
    + v_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2]) / 1000.0 - 11.0 * std::sin(q
    [5]) * ((((((std::cos(q[1]) * std::cos(q[3]) * std::sin(q[0]) * dq[0] + std::
    cos(q[0]) * std::cos(q[3]) * std::sin(q[1]) * dq[1]) + std::cos(q[0]) * std::
    cos(q[1]) * std::sin(q[3]) * dq[3]) - std::cos(q[0]) * std::cos(q[1]) * std::
    cos(q[2]) * std::sin(q[3]) * dq[1]) - std::cos(q[0]) * std::cos(q[2]) * std::
    cos(q[3]) * std::sin(q[1]) * dq[3]) + std::cos(q[2]) * std::sin(q[0]) * std::
    sin(q[1]) * std::sin(q[3]) * dq[0]) + std::cos(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * std::sin(q[3]) * dq[2]) / 125.0) + 11.0 * std::cos(q[5]) *
    (((((i_Ja_dot_tmp_tmp * ((((((m_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp * dq[3] -
    w_Ja_dot_tmp_tmp * dq[1]) - x_Ja_dot_tmp_tmp * dq[0]) - b_Ja_dot_tmp_tmp_tmp
    * f_Ja_dot_tmp_tmp * dq[1]) + Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp *
    b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[0]) + i_Ja_dot_tmp_tmp_tmp *
    c_Ja_dot_tmp_tmp * dq[2]) + k_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp *
    h_Ja_dot_tmp_tmp * dq[3]) - j_Ja_dot_tmp_tmp * h_Ja_dot_tmp * dq[4]) +
    u_Ja_dot_tmp_tmp * j_Ja_dot_tmp_tmp * dq[1]) + b_Ja_dot_tmp_tmp_tmp_tmp *
    j_Ja_dot_tmp_tmp * dq[2]) + d_Ja_dot_tmp_tmp * i_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[4]) - y_Ja_dot_tmp_tmp *
    j_Ja_dot_tmp_tmp * dq[0]) / 125.0) + 107.0 * std::sin(q[5]) * (((((std::cos
    (q[4]) * ((((((std::cos(q[0]) * std::cos(q[1]) * std::cos(q[3]) * dq[3] -
    std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3]) * dq[1]) - std::cos(q[1]) *
    std::sin(q[0]) * std::sin(q[3]) * dq[0]) - std::cos(q[0]) * std::cos(q[1]) *
    std::cos(q[2]) * std::cos(q[3]) * dq[1]) + std::cos(q[2]) * std::cos(q[3]) *
    std::sin(q[0]) * std::sin(q[1]) * dq[0]) + std::cos(q[0]) * std::cos(q[3]) *
    std::sin(q[1]) * std::sin(q[2]) * dq[2]) + std::cos(q[0]) * std::cos(q[2]) *
    std::sin(q[1]) * std::sin(q[3]) * dq[3]) - std::sin(q[4]) * (std::cos(q[0]) *
    std::cos(q[1]) * std::sin(q[3]) - std::cos(q[0]) * std::cos(q[2]) * std::cos
    (q[3]) * std::sin(q[1])) * dq[4]) + std::cos(q[0]) * std::cos(q[1]) * std::
    sin(q[2]) * std::sin(q[4]) * dq[1]) + std::cos(q[0]) * std::cos(q[2]) * std::
    sin(q[1]) * std::sin(q[4]) * dq[2]) + std::cos(q[0]) * std::cos(q[4]) * std::
    sin(q[1]) * std::sin(q[2]) * dq[4]) - std::sin(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * std::sin(q[4]) * dq[0]) / 1000.0) + 107.0 * std::cos(q[5]) * dq
    [5] * (i_Ja_dot_tmp_tmp * h_Ja_dot_tmp + v_Ja_dot_tmp_tmp * j_Ja_dot_tmp_tmp)
    / 1000.0) - 11.0 * std::sin(q[5]) * dq[5] * (std::cos(q[4]) * (std::cos(q[0])
    * std::cos(q[1]) * std::sin(q[3]) - std::cos(q[0]) * std::cos(q[2]) * std::
    cos(q[3]) * std::sin(q[1])) + std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])
    * std::sin(q[4])) / 125.0) + 11.0 * std::cos(q[5]) * i_Ja_dot_tmp * dq[5] /
    125.0) + 107.0 * std::sin(q[5]) * i_Ja_dot_tmp * dq[5] / 1000.0) - 79.0 *
    std::cos(q[1]) * std::sin(q[0]) * dq[0] / 250.0) - 79.0 * std::cos(q[0]) *
    std::sin(q[1]) * dq[1] / 250.0) - 48.0 * std::cos(q[1]) * std::cos(q[3]) *
    std::sin(q[0]) * dq[0] / 125.0) - l_Ja_dot_tmp * dq[1] / 125.0) -
    fb_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3] / 125.0) + o_Ja_dot_tmp * dq[0]
    / 400.0) + 33.0 * std::cos(q[1]) * std::sin(q[0]) * std::sin(q[3]) * dq[0] /
    400.0) + 33.0 * std::cos(q[0]) * std::sin(q[1]) * c_Ja_dot_tmp_tmp * dq[2] /
    400.0) + m_Ja_dot_tmp * dq[1] / 400.0) - j_Ja_dot_tmp * dq[1] / 400.0) -
                      n_Ja_dot_tmp * f_Ja_dot_tmp_tmp * dq[3] / 400.0) +
                     j_Ja_dot_tmp * f_Ja_dot_tmp_tmp * dq[1] / 400.0) +
                    fb_Ja_dot_tmp_tmp * Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[1]
                    / 125.0) + 48.0 * std::cos(q[0]) * Ja_dot_tmp_tmp *
                   f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[3] / 125.0) - 33.0 *
                  std::cos(q[2]) * f_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp *
                  g_Ja_dot_tmp_tmp * dq[0] / 400.0) - 33.0 * std::cos(q[0]) *
                 f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[2] /
                 400.0) - k_Ja_dot_tmp * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp *
                dq[3] / 400.0) - 48.0 * Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp *
               g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[0] / 125.0) - 48.0 * std::
    cos(q[0]) * g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2] /
    125.0;
  h_Ja_dot_tmp = n_Ja_dot_tmp_tmp + u_Ja_dot_tmp_tmp;
  i_Ja_dot_tmp = (((d_Ja_dot_tmp_tmp_tmp * dq[2] - k_Ja_dot_tmp_tmp * dq[0]) +
                   l_Ja_dot_tmp_tmp * dq[0]) + v_Ja_dot_tmp_tmp * dq[1]) -
    b_Ja_dot_tmp_tmp_tmp * dq[2];
  p_Ja_dot_tmp = d_Ja_dot_tmp_tmp_tmp - b_Ja_dot_tmp_tmp_tmp;
  n_Ja_dot_tmp_tmp = (((e_Ja_dot_tmp_tmp_tmp * dq[0] + n_Ja_dot_tmp_tmp * dq[2])
                       + f_Ja_dot_tmp_tmp_tmp * dq[0]) +
                      b_Ja_dot_tmp_tmp_tmp_tmp * dq[1]) + u_Ja_dot_tmp_tmp * dq
    [2];
  d_Ja_dot_tmp_tmp = f_Ja_dot_tmp_tmp * i_Ja_dot_tmp_tmp;
  Ja_dot[6] = (((((((((((((((((11.0 * std::cos(q[5]) * ((((j_Ja_dot_tmp_tmp *
    n_Ja_dot_tmp_tmp + i_Ja_dot_tmp_tmp * p_Ja_dot_tmp * dq[4]) +
    d_Ja_dot_tmp_tmp * i_Ja_dot_tmp) + i_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp *
    h_Ja_dot_tmp * dq[3]) + f_Ja_dot_tmp_tmp * j_Ja_dot_tmp_tmp * h_Ja_dot_tmp *
    dq[4]) / 125.0 + 107.0 * std::sin(q[5]) * ((((std::sin(q[4]) * ((((std::cos
    (q[0]) * std::sin(q[2]) * dq[0] + std::cos(q[2]) * std::sin(q[0]) * dq[2]) +
    std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]) * dq[0]) + std::cos(q[0]) *
    std::cos(q[2]) * std::sin(q[1]) * dq[1]) + std::cos(q[0]) * std::cos(q[1]) *
    std::sin(q[2]) * dq[2]) + std::cos(q[4]) * (std::sin(q[0]) * std::sin(q[2])
    - std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) * dq[4]) + std::cos(q[3])
    * std::cos(q[4]) * ((((std::sin(q[0]) * std::sin(q[2]) * dq[2] - std::cos(q
    [0]) * std::cos(q[2]) * dq[0]) + std::cos(q[1]) * std::sin(q[0]) * std::sin
    (q[2]) * dq[0]) + std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]) * dq[1])
    - std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) * dq[2])) + std::cos(q[4])
    * std::sin(q[3]) * (std::cos(q[2]) * std::sin(q[0]) + std::cos(q[0]) * std::
    cos(q[1]) * std::sin(q[2])) * dq[3]) + std::cos(q[3]) * std::sin(q[4]) *
    (std::cos(q[2]) * std::sin(q[0]) + std::cos(q[0]) * std::cos(q[1]) * std::
     sin(q[2])) * dq[4]) / 1000.0) - 33.0 * std::cos(q[3]) * i_Ja_dot_tmp /
    400.0) - 48.0 * std::sin(q[3]) * i_Ja_dot_tmp / 125.0) - 11.0 *
    h_Ja_dot_tmp_tmp * b_Ja_dot_tmp * i_Ja_dot_tmp / 125.0) + 48.0 * std::cos(q
    [3]) * h_Ja_dot_tmp * dq[3] / 125.0) - 33.0 * std::sin(q[3]) * h_Ja_dot_tmp *
    dq[3] / 400.0) - k_Ja_dot_tmp * dq[0] / 400.0) + 107.0 * std::cos(q[5]) *
                        (j_Ja_dot_tmp_tmp * p_Ja_dot_tmp - d_Ja_dot_tmp_tmp *
    h_Ja_dot_tmp) * dq[5] / 1000.0) + 33.0 * std::sin(q[0]) * std::sin(q[2]) *
                       dq[2] / 400.0) - 11.0 * std::sin(q[5]) * (std::sin(q[4]) *
    (std::sin(q[0]) * std::sin(q[2]) - std::cos(q[0]) * std::cos(q[1]) * std::
     cos(q[2])) - std::cos(q[3]) * std::cos(q[4]) * (std::cos(q[2]) * std::sin
    (q[0]) + std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2]))) * dq[5] / 125.0)
                     + 107.0 * std::cos(q[5]) * h_Ja_dot_tmp_tmp * i_Ja_dot_tmp /
                     1000.0) + 33.0 * std::cos(q[1]) * std::sin(q[0]) * std::sin
                    (q[2]) * dq[0] / 400.0) + 33.0 * std::cos(q[0]) * std::sin
                   (q[1]) * std::sin(q[2]) * dq[1] / 400.0) - 107.0 *
                  f_Ja_dot_tmp_tmp * Ja_dot_tmp * h_Ja_dot_tmp * dq[3] / 1000.0)
                 + 11.0 * f_Ja_dot_tmp_tmp * b_Ja_dot_tmp * h_Ja_dot_tmp * dq[3]
                 / 125.0) + 11.0 * std::cos(q[5]) * h_Ja_dot_tmp_tmp *
                h_Ja_dot_tmp * dq[5] / 125.0) + 107.0 * h_Ja_dot_tmp_tmp *
               b_Ja_dot_tmp * h_Ja_dot_tmp * dq[5] / 1000.0) - j_Ja_dot_tmp *
    dq[2] / 400.0;
  k_Ja_dot_tmp_tmp = f_Ja_dot_tmp_tmp * (std::sin(q[0]) * std::sin(q[2]) - std::
    cos(q[0]) * std::cos(q[1]) * std::cos(q[2]));
  j_Ja_dot_tmp = k_Ja_dot_tmp_tmp - w_Ja_dot_tmp_tmp;
  m_Ja_dot_tmp_tmp = h_Ja_dot_tmp_tmp * (std::sin(q[0]) * std::sin(q[2]) - std::
    cos(q[0]) * std::cos(q[1]) * std::cos(q[2]));
  b_Ja_dot_tmp_tmp_tmp = m_Ja_dot_tmp_tmp + i_Ja_dot_tmp_tmp_tmp;
  m_Ja_dot_tmp_tmp = (((m_Ja_dot_tmp_tmp * dq[3] - f_Ja_dot_tmp_tmp * ((((std::
    cos(q[0]) * std::sin(q[2]) * dq[0] + std::cos(q[2]) * std::sin(q[0]) * dq[2])
    + std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]) * dq[0]) + std::cos(q[0])
    * std::cos(q[2]) * std::sin(q[1]) * dq[1]) + std::cos(q[0]) * std::cos(q[1])
    * std::sin(q[2]) * dq[2])) + Ja_dot_tmp_tmp_tmp * dq[1]) +
                      i_Ja_dot_tmp_tmp_tmp * dq[3]) - h_Ja_dot_tmp_tmp_tmp * dq
    [0];
  d_Ja_dot_tmp_tmp_tmp = (((h_Ja_dot_tmp_tmp * ((((std::cos(q[0]) * std::sin(q[2])
    * dq[0] + std::cos(q[2]) * std::sin(q[0]) * dq[2]) + std::cos(q[1]) * std::
    cos(q[2]) * std::sin(q[0]) * dq[0]) + std::cos(q[0]) * std::cos(q[2]) * std::
    sin(q[1]) * dq[1]) + std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2]) * dq[2])
    + k_Ja_dot_tmp_tmp * dq[3]) - g_Ja_dot_tmp_tmp_tmp * dq[0]) -
    w_Ja_dot_tmp_tmp * dq[3]) + k_Ja_dot_tmp_tmp_tmp * dq[1];
  v_Ja_dot_tmp_tmp = 33.0 * std::cos(q[3]) * std::sin(q[0]) * std::sin(q[1]);
  Ja_dot[9] = ((((((((((((((((((48.0 * std::cos(q[3]) * n_Ja_dot_tmp_tmp / 125.0
    - 33.0 * std::sin(q[3]) * n_Ja_dot_tmp_tmp / 400.0) + 107.0 * std::cos(q[5])
    * ((((h_Ja_dot_tmp_tmp * p_Ja_dot_tmp * dq[3] - f_Ja_dot_tmp_tmp *
    n_Ja_dot_tmp_tmp) + Ja_dot_tmp_tmp_tmp * dq[1]) + i_Ja_dot_tmp_tmp_tmp * dq
        [3]) - h_Ja_dot_tmp_tmp_tmp * dq[0]) / 1000.0) - 11.0 * std::sin(q[5]) *
    m_Ja_dot_tmp_tmp / 125.0) - 33.0 * std::cos(q[3]) * p_Ja_dot_tmp * dq[3] /
    400.0) - 48.0 * std::sin(q[3]) * p_Ja_dot_tmp * dq[3] / 125.0) + 11.0 *
    i_Ja_dot_tmp_tmp * Ja_dot_tmp * ((((h_Ja_dot_tmp_tmp * n_Ja_dot_tmp_tmp +
    f_Ja_dot_tmp_tmp * p_Ja_dot_tmp * dq[3]) - g_Ja_dot_tmp_tmp_tmp * dq[0]) -
    w_Ja_dot_tmp_tmp * dq[3]) + k_Ja_dot_tmp_tmp_tmp * dq[1]) / 125.0) + 107.0 *
    i_Ja_dot_tmp_tmp * b_Ja_dot_tmp * d_Ja_dot_tmp_tmp_tmp / 1000.0) + 11.0 *
    std::cos(q[5]) * j_Ja_dot_tmp * dq[5] / 125.0) + 107.0 * std::sin(q[5]) *
                        j_Ja_dot_tmp * dq[5] / 1000.0) - 48.0 * std::cos(q[0]) *
                       std::cos(q[1]) * std::sin(q[3]) * dq[1] / 125.0) -
                      l_Ja_dot_tmp * dq[3] / 125.0) + v_Ja_dot_tmp_tmp * dq[0] /
                     400.0) + m_Ja_dot_tmp * dq[3] / 400.0) + 48.0 * std::sin(q
    [0]) * std::sin(q[1]) * std::sin(q[3]) * dq[0] / 125.0) + 107.0 * std::cos
                  (q[4]) * Ja_dot_tmp * b_Ja_dot_tmp_tmp_tmp * dq[5] / 1000.0) -
                 11.0 * std::cos(q[5]) * j_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp_tmp
                 * dq[4] / 125.0) - 11.0 * std::cos(q[4]) * b_Ja_dot_tmp *
                b_Ja_dot_tmp_tmp_tmp * dq[5] / 125.0) - 107.0 * j_Ja_dot_tmp_tmp
               * b_Ja_dot_tmp * b_Ja_dot_tmp_tmp_tmp * dq[4] / 1000.0) - 33.0 *
    std::cos(q[0]) * std::cos(q[1]) * std::cos(q[3]) * dq[1] / 400.0;
  Ja_dot[12] = ((11.0 * std::cos(q[5]) * (((i_Ja_dot_tmp_tmp * i_Ja_dot_tmp -
    j_Ja_dot_tmp_tmp * m_Ja_dot_tmp_tmp) + j_Ja_dot_tmp_tmp * h_Ja_dot_tmp * dq
    [4]) + i_Ja_dot_tmp_tmp * j_Ja_dot_tmp * dq[4]) / 125.0 + 107.0 * std::sin
                 (q[5]) * (((std::cos(q[4]) * ((((std::sin(q[0]) * std::sin(q[2])
    * dq[2] - std::cos(q[0]) * std::cos(q[2]) * dq[0]) + std::cos(q[1]) * std::
    sin(q[0]) * std::sin(q[2]) * dq[0]) + std::cos(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * dq[1]) - std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) * dq[2])
    - std::sin(q[4]) * ((((std::sin(q[3]) * (std::sin(q[0]) * std::sin(q[2]) -
    std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) * dq[3] - std::cos(q[3]) *
    ((((std::cos(q[0]) * std::sin(q[2]) * dq[0] + std::cos(q[2]) * std::sin(q[0])
        * dq[2]) + std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]) * dq[0]) +
      std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]) * dq[1]) + std::cos(q[0])
     * std::cos(q[1]) * std::sin(q[2]) * dq[2])) + std::cos(q[0]) * std::cos(q[1])
    * std::sin(q[3]) * dq[1]) + std::cos(q[0]) * std::cos(q[3]) * std::sin(q[1])
    * dq[3]) - std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * dq[0])) + std::
    sin(q[4]) * (std::cos(q[2]) * std::sin(q[0]) + std::cos(q[0]) * std::cos(q[1])
                 * std::sin(q[2])) * dq[4]) + std::cos(q[4]) * (std::cos(q[3]) *
    (std::sin(q[0]) * std::sin(q[2]) - std::cos(q[0]) * std::cos(q[1]) * std::
     cos(q[2])) - std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3])) * dq[4]) /
                 1000.0) - 107.0 * std::cos(q[5]) * (i_Ja_dot_tmp_tmp *
    h_Ja_dot_tmp - j_Ja_dot_tmp_tmp * j_Ja_dot_tmp) * dq[5] / 1000.0) + 11.0 *
    std::sin(q[5]) * (std::cos(q[4]) * (std::cos(q[2]) * std::sin(q[0]) + std::
    cos(q[0]) * std::cos(q[1]) * std::sin(q[2])) - std::sin(q[4]) * (std::cos(q
    [3]) * (std::sin(q[0]) * std::sin(q[2]) - std::cos(q[0]) * std::cos(q[1]) *
            std::cos(q[2])) - std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3])))
    * dq[5] / 125.0;
  h_Ja_dot_tmp = std::sin(q[4]) * (std::cos(q[2]) * std::sin(q[0]) + std::cos(q
    [0]) * std::cos(q[1]) * std::sin(q[2])) + std::cos(q[4]) * (std::cos(q[3]) *
    (std::sin(q[0]) * std::sin(q[2]) - std::cos(q[0]) * std::cos(q[1]) * std::
     cos(q[2])) - std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3]));
  j_Ja_dot_tmp = ((j_Ja_dot_tmp_tmp * ((((std::sin(q[0]) * std::sin(q[2]) * dq[2]
    - std::cos(q[0]) * std::cos(q[2]) * dq[0]) + std::cos(q[1]) * std::sin(q[0])
    * std::sin(q[2]) * dq[0]) + std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])
    * dq[1]) - std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) * dq[2]) +
                   i_Ja_dot_tmp_tmp * m_Ja_dot_tmp_tmp) - std::cos(q[4]) * (std::
    cos(q[2]) * std::sin(q[0]) + std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2]))
                  * dq[4]) + std::sin(q[4]) * (std::cos(q[3]) * (std::sin(q[0]) *
    std::sin(q[2]) - std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) - std::
    cos(q[0]) * std::sin(q[1]) * std::sin(q[3])) * dq[4];
  Ja_dot[15] = ((((((107.0 * std::cos(q[5]) * (((j_Ja_dot_tmp_tmp * i_Ja_dot_tmp
    + i_Ja_dot_tmp_tmp * m_Ja_dot_tmp_tmp) - std::cos(q[4]) * (std::cos(q[2]) *
    std::sin(q[0]) + std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2])) * dq[4])
    + std::sin(q[4]) * (std::cos(q[3]) * (std::sin(q[0]) * std::sin(q[2]) - std::
    cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) - std::cos(q[0]) * std::sin(q[1])
                        * std::sin(q[3])) * dq[4]) / 1000.0 - 11.0 * std::sin(q
    [5]) * j_Ja_dot_tmp / 125.0) + 11.0 * std::cos(q[5]) * d_Ja_dot_tmp_tmp_tmp /
                    125.0) + 107.0 * std::sin(q[5]) * d_Ja_dot_tmp_tmp_tmp /
                   1000.0) + 107.0 * std::cos(q[5]) * b_Ja_dot_tmp_tmp_tmp * dq
                  [5] / 1000.0) - 11.0 * std::sin(q[5]) * b_Ja_dot_tmp_tmp_tmp *
                 dq[5] / 125.0) + 11.0 * std::cos(q[5]) * h_Ja_dot_tmp * dq[5] /
                125.0) + 107.0 * std::sin(q[5]) * h_Ja_dot_tmp * dq[5] / 1000.0;
  Ja_dot[18] = 0.0;
  i_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]);
  l_Ja_dot_tmp = 33.0 * std::cos(q[2]) * std::sin(q[0]);
  m_Ja_dot_tmp = 33.0 * std::cos(q[0]) * std::cos(q[3]) * std::sin(q[1]);
  Ja_dot[1] = (((((((((((((((((((((((11.0 * std::cos(q[5]) * j_Ja_dot_tmp /
    125.0 + 107.0 * std::sin(q[5]) * j_Ja_dot_tmp / 1000.0) + 33.0 * std::cos(q
    [3]) * n_Ja_dot_tmp_tmp / 400.0) + 48.0 * std::sin(q[3]) * n_Ja_dot_tmp_tmp /
    125.0) - 107.0 * std::cos(q[5]) * d_Ja_dot_tmp_tmp_tmp / 1000.0) + 11.0 *
    std::sin(q[5]) * d_Ja_dot_tmp_tmp_tmp / 125.0) + 48.0 * std::cos(q[3]) *
    p_Ja_dot_tmp * dq[3] / 125.0) - 33.0 * std::sin(q[3]) * p_Ja_dot_tmp * dq[3]
    / 400.0) + 11.0 * std::cos(q[5]) * b_Ja_dot_tmp_tmp_tmp * dq[5] / 125.0) +
    107.0 * std::sin(q[5]) * b_Ja_dot_tmp_tmp_tmp * dq[5] / 1000.0) + 79.0 * std::
    cos(q[0]) * e_Ja_dot_tmp_tmp * dq[1] / 250.0) - 33.0 * std::cos(q[0]) *
    c_Ja_dot_tmp_tmp * dq[0] / 400.0) - l_Ja_dot_tmp * dq[2] / 400.0) - 79.0 *
    b_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[0] / 250.0) - 107.0 * std::cos(q[5])
                        * h_Ja_dot_tmp * dq[5] / 1000.0) + 11.0 * std::sin(q[5])
                       * h_Ja_dot_tmp * dq[5] / 125.0) - 33.0 * std::cos(q[1]) *
                      Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp * dq[0] / 400.0) -
                     i_Ja_dot_tmp * dq[1] / 400.0) - n_Ja_dot_tmp *
                    c_Ja_dot_tmp_tmp * dq[2] / 400.0) - n_Ja_dot_tmp *
                   h_Ja_dot_tmp_tmp * dq[1] / 400.0) - m_Ja_dot_tmp * dq[3] /
                  400.0) - 48.0 * std::cos(q[3]) * b_Ja_dot_tmp_tmp *
                 g_Ja_dot_tmp_tmp * dq[0] / 125.0) - 48.0 * std::cos(q[0]) * std::
                sin(q[1]) * h_Ja_dot_tmp_tmp * dq[3] / 125.0) + 33.0 * std::sin
               (q[0]) * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[0] / 400.0) +
    fb_Ja_dot_tmp_tmp * f_Ja_dot_tmp_tmp * dq[1] / 125.0;
  h_Ja_dot_tmp = std::cos(q[2]) * std::cos(q[3]) * std::sin(q[0]) * std::sin(q[1]);
  j_Ja_dot_tmp = x_Ja_dot_tmp_tmp - h_Ja_dot_tmp;
  n_Ja_dot_tmp = std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]);
  p_Ja_dot_tmp = j_Ja_dot_tmp_tmp_tmp + n_Ja_dot_tmp;
  n_Ja_dot_tmp_tmp = 33.0 * std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]);
  b_Ja_dot_tmp_tmp_tmp = 48.0 * std::cos(q[3]) * std::sin(q[0]) * std::sin(q[1]);
  d_Ja_dot_tmp_tmp_tmp = 33.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]);
  b_Ja_dot_tmp_tmp_tmp_tmp = 33.0 * std::cos(q[1]) * std::cos(q[2]);
  Ja_dot[4] = (((((((((((((((((((((((((11.0 * std::cos(q[5]) *
    (((((i_Ja_dot_tmp_tmp * ((((((Ja_dot_tmp_tmp_tmp * dq[0] +
    j_Ja_dot_tmp_tmp_tmp * dq[3]) - h_Ja_dot_tmp_tmp_tmp * dq[1]) -
    db_Ja_dot_tmp_tmp * dq[0]) - Ja_dot_tmp_tmp_tmp_tmp * f_Ja_dot_tmp_tmp *
    b_Ja_dot_tmp_tmp * dq[1]) + g_Ja_dot_tmp_tmp_tmp * c_Ja_dot_tmp_tmp * dq[2])
    + n_Ja_dot_tmp * dq[3]) - j_Ja_dot_tmp_tmp * j_Ja_dot_tmp * dq[4]) + std::
    cos(q[0]) * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * dq[0]) +
    l_Ja_dot_tmp_tmp * j_Ja_dot_tmp_tmp * dq[1]) + c_Ja_dot_tmp_tmp_tmp *
    j_Ja_dot_tmp_tmp * dq[2]) + i_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[4]) / 125.0 + 107.0 * std::sin(q[5])
    * (((((std::cos(q[4]) * ((((((std::cos(q[0]) * std::cos(q[1]) * std::sin(q[3])
    * dq[0] + std::cos(q[1]) * std::cos(q[3]) * std::sin(q[0]) * dq[3]) - std::
    sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * dq[1]) - std::cos(q[0]) * std::
    cos(q[2]) * std::cos(q[3]) * std::sin(q[1]) * dq[0]) - std::cos(q[1]) * std::
    cos(q[2]) * std::cos(q[3]) * std::sin(q[0]) * dq[1]) + std::cos(q[3]) * std::
    sin(q[0]) * std::sin(q[1]) * std::sin(q[2]) * dq[2]) + std::cos(q[2]) * std::
    sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * dq[3]) - std::sin(q[4]) * (std::
    cos(q[1]) * std::sin(q[0]) * std::sin(q[3]) - std::cos(q[2]) * std::cos(q[3])
    * std::sin(q[0]) * std::sin(q[1])) * dq[4]) + std::cos(q[0]) * std::sin(q[1])
    * std::sin(q[2]) * std::sin(q[4]) * dq[0]) + std::cos(q[1]) * std::sin(q[0])
    * std::sin(q[2]) * std::sin(q[4]) * dq[1]) + std::cos(q[2]) * std::sin(q[0])
    * std::sin(q[1]) * std::sin(q[4]) * dq[2]) + std::cos(q[4]) * std::sin(q[0])
    * std::sin(q[1]) * std::sin(q[2]) * dq[4]) / 1000.0) - 107.0 * std::cos(q[5])
    * ((((((k_Ja_dot_tmp_tmp_tmp * dq[0] - x_Ja_dot_tmp_tmp * dq[3]) -
    g_Ja_dot_tmp_tmp_tmp * dq[1]) + eb_Ja_dot_tmp_tmp * dq[0]) +
    f_Ja_dot_tmp_tmp_tmp * h_Ja_dot_tmp_tmp * dq[1]) + h_Ja_dot_tmp * dq[3]) -
    y_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2]) / 1000.0) + 11.0 * std::sin(q[5])
    * ((((((std::cos(q[0]) * std::cos(q[1]) * std::cos(q[3]) * dq[0] - std::cos
    (q[1]) * std::sin(q[0]) * std::sin(q[3]) * dq[3]) - std::cos(q[3]) * std::
    sin(q[0]) * std::sin(q[1]) * dq[1]) + std::cos(q[0]) * std::cos(q[2]) * std::
    sin(q[1]) * std::sin(q[3]) * dq[0]) + std::cos(q[1]) * std::cos(q[2]) * std::
    sin(q[0]) * std::sin(q[3]) * dq[1]) + std::cos(q[2]) * std::cos(q[3]) * std::
    sin(q[0]) * std::sin(q[1]) * dq[3]) - std::sin(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * std::sin(q[3]) * dq[2]) / 125.0) + 107.0 * std::cos(q[5]) *
    (i_Ja_dot_tmp_tmp * j_Ja_dot_tmp + std::sin(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * std::sin(q[4])) * dq[5] / 1000.0) - 11.0 * std::sin(q[5]) * (std::
    cos(q[4]) * (std::cos(q[1]) * std::sin(q[0]) * std::sin(q[3]) - std::cos(q[2])
    * std::cos(q[3]) * std::sin(q[0]) * std::sin(q[1])) + std::sin(q[0]) * std::
    sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) * dq[5] / 125.0) + 79.0 * std::
    cos(q[0]) * std::cos(q[1]) * dq[0] / 250.0) + 11.0 * std::cos(q[5]) *
    p_Ja_dot_tmp * dq[5] / 125.0) - 79.0 * std::sin(q[0]) * std::sin(q[1]) * dq
    [1] / 250.0) + 107.0 * std::sin(q[5]) * p_Ja_dot_tmp * dq[5] / 1000.0) -
    i_Ja_dot_tmp * dq[0] / 400.0) - 33.0 * std::cos(q[0]) * std::cos(q[1]) * std::
    sin(q[3]) * dq[0] / 400.0) - n_Ja_dot_tmp_tmp * dq[1] / 400.0) - 33.0 * std::
    cos(q[1]) * f_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp * dq[3] / 400.0) -
    b_Ja_dot_tmp_tmp_tmp * dq[1] / 125.0) - 48.0 * std::cos(q[1]) *
    b_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3] / 125.0) + 33.0 * std::sin(q[0])
                        * std::sin(q[1]) * c_Ja_dot_tmp_tmp * dq[2] / 400.0) +
                       d_Ja_dot_tmp_tmp_tmp * dq[1] / 400.0) + 48.0 * std::cos
                      (q[0]) * std::cos(q[1]) * std::cos(q[3]) * dq[0] / 125.0)
                     + k_Ja_dot_tmp * f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[0]
                     / 400.0) + b_Ja_dot_tmp_tmp_tmp_tmp * f_Ja_dot_tmp_tmp *
                    b_Ja_dot_tmp_tmp * dq[1] / 400.0) + 48.0 * std::cos(q[0]) *
                   std::cos(q[2]) * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[0] /
                   125.0) + 48.0 * std::cos(q[1]) * Ja_dot_tmp_tmp *
                  b_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[1] / 125.0) + 48.0 *
                 std::cos(q[2]) * f_Ja_dot_tmp_tmp * b_Ja_dot_tmp_tmp *
                 g_Ja_dot_tmp_tmp * dq[3] / 125.0) - v_Ja_dot_tmp_tmp *
                c_Ja_dot_tmp_tmp * dq[2] / 400.0) - o_Ja_dot_tmp *
               h_Ja_dot_tmp_tmp * dq[3] / 400.0) - 48.0 * std::sin(q[0]) * std::
    sin(q[1]) * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2] / 125.0;
  h_Ja_dot_tmp = 107.0 * std::cos(q[3]) * std::cos(q[5]);
  Ja_dot[7] = (((((((((((((((((33.0 * std::cos(q[3]) * ab_Ja_dot_tmp_tmp / 400.0
    - 107.0 * std::sin(q[5]) * ((((i_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp * dq[4] -
    j_Ja_dot_tmp_tmp * o_Ja_dot_tmp_tmp) + d_Ja_dot_tmp_tmp * ab_Ja_dot_tmp_tmp)
    + std::cos(q[4]) * std::sin(q[3]) * d_Ja_dot_tmp * dq[3]) + std::cos(q[3]) *
    std::sin(q[4]) * d_Ja_dot_tmp * dq[4]) / 1000.0) - 11.0 * std::cos(q[5]) *
    ((((std::cos(q[4]) * (std::cos(q[0]) * std::sin(q[2]) + std::cos(q[1]) * std::
    cos(q[2]) * std::sin(q[0])) * dq[4] - std::sin(q[4]) * ((((std::sin(q[0]) *
    std::sin(q[2]) * dq[0] - std::cos(q[0]) * std::cos(q[2]) * dq[2]) + std::cos
    (q[2]) * std::sin(q[0]) * std::sin(q[1]) * dq[1]) + std::cos(q[1]) * std::
    sin(q[0]) * std::sin(q[2]) * dq[2]) - std::cos(q[0]) * std::cos(q[1]) * std::
    cos(q[2]) * dq[0])) + std::cos(q[3]) * std::cos(q[4]) * ((((std::cos(q[2]) *
    std::sin(q[0]) * dq[0] + std::cos(q[0]) * std::sin(q[2]) * dq[2]) + std::cos
    (q[0]) * std::cos(q[1]) * std::sin(q[2]) * dq[0]) + std::cos(q[1]) * std::
    cos(q[2]) * std::sin(q[0]) * dq[2]) - std::sin(q[0]) * std::sin(q[1]) * std::
    sin(q[2]) * dq[1])) + std::cos(q[4]) * std::sin(q[3]) * (std::cos(q[0]) *
    std::cos(q[2]) - std::cos(q[1]) * std::sin(q[0]) * std::sin(q[2])) * dq[3])
     + std::cos(q[3]) * std::sin(q[4]) * (std::cos(q[0]) * std::cos(q[2]) - std::
    cos(q[1]) * std::sin(q[0]) * std::sin(q[2])) * dq[4]) / 125.0) + 48.0 * std::
    sin(q[3]) * ab_Ja_dot_tmp_tmp / 125.0) - 107.0 * std::cos(q[5]) * std::sin
    (q[3]) * ab_Ja_dot_tmp_tmp / 1000.0) - 48.0 * std::cos(q[3]) * d_Ja_dot_tmp *
    dq[3] / 125.0) + 11.0 * std::sin(q[3]) * std::sin(q[5]) * ab_Ja_dot_tmp_tmp /
    125.0) + 33.0 * std::sin(q[3]) * d_Ja_dot_tmp * dq[3] / 400.0) -
                        l_Ja_dot_tmp * dq[0] / 400.0) - 33.0 * std::cos(q[0]) *
                       std::sin(q[2]) * dq[2] / 400.0) - 107.0 * std::cos(q[5]) *
                      (j_Ja_dot_tmp_tmp * p_Ja_dot_tmp_tmp - d_Ja_dot_tmp_tmp *
                       d_Ja_dot_tmp) * dq[5] / 1000.0) + 11.0 * std::sin(q[5]) *
                     (std::sin(q[4]) * (std::cos(q[0]) * std::sin(q[2]) + std::
    cos(q[1]) * std::cos(q[2]) * std::sin(q[0])) - std::cos(q[3]) * std::cos(q[4])
                      * (std::cos(q[0]) * std::cos(q[2]) - std::cos(q[1]) * std::
    sin(q[0]) * std::sin(q[2]))) * dq[5] / 125.0) - 33.0 * std::cos(q[0]) * std::
                    cos(q[1]) * std::sin(q[2]) * dq[0] / 400.0) -
                   n_Ja_dot_tmp_tmp * dq[2] / 400.0) + 33.0 * std::sin(q[0]) *
                  std::sin(q[1]) * std::sin(q[2]) * dq[1] / 400.0) +
                 h_Ja_dot_tmp * d_Ja_dot_tmp * dq[3] / 1000.0) - 11.0 * std::cos
                (q[3]) * std::sin(q[5]) * d_Ja_dot_tmp * dq[3] / 125.0) - 11.0 *
               std::cos(q[5]) * std::sin(q[3]) * d_Ja_dot_tmp * dq[5] / 125.0) -
    107.0 * std::sin(q[3]) * std::sin(q[5]) * d_Ja_dot_tmp * dq[5] / 1000.0;
  d_Ja_dot_tmp = 107.0 * std::cos(q[4]) * std::sin(q[5]);
  i_Ja_dot_tmp = 11.0 * std::cos(q[4]) * std::cos(q[5]);
  j_Ja_dot_tmp = 107.0 * std::cos(q[4]) * std::cos(q[5]);
  k_Ja_dot_tmp = 11.0 * std::cos(q[5]) * std::sin(q[4]);
  l_Ja_dot_tmp = 11.0 * std::cos(q[4]) * std::sin(q[5]);
  n_Ja_dot_tmp = 107.0 * std::sin(q[4]) * std::sin(q[5]);
  Ja_dot[10] = ((((((((((((((((((107.0 * std::cos(q[5]) * q_Ja_dot_tmp_tmp /
    1000.0 - 11.0 * std::sin(q[5]) * q_Ja_dot_tmp_tmp / 125.0) + 48.0 * std::cos
    (q[3]) * o_Ja_dot_tmp_tmp / 125.0) - 33.0 * std::sin(q[3]) *
    o_Ja_dot_tmp_tmp / 400.0) + 33.0 * std::cos(q[3]) * p_Ja_dot_tmp_tmp * dq[3]
    / 400.0) + 48.0 * std::sin(q[3]) * p_Ja_dot_tmp_tmp * dq[3] / 125.0) +
    i_Ja_dot_tmp * r_Ja_dot_tmp_tmp / 125.0) + d_Ja_dot_tmp * r_Ja_dot_tmp_tmp /
    1000.0) - 11.0 * std::cos(q[5]) * e_Ja_dot_tmp * dq[5] / 125.0) - 107.0 *
    std::sin(q[5]) * e_Ja_dot_tmp * dq[5] / 1000.0) - m_Ja_dot_tmp * dq[0] /
                        400.0) - 33.0 * std::cos(q[1]) * std::cos(q[3]) * std::
                       sin(q[0]) * dq[1] / 400.0) - 48.0 * std::cos(q[0]) * std::
                      sin(q[1]) * std::sin(q[3]) * dq[0] / 125.0) - 48.0 * std::
                     cos(q[1]) * std::sin(q[0]) * std::sin(q[3]) * dq[1] / 125.0)
                    - b_Ja_dot_tmp_tmp_tmp * dq[3] / 125.0) +
                   d_Ja_dot_tmp_tmp_tmp * dq[3] / 400.0) - j_Ja_dot_tmp *
                  c_Ja_dot_tmp * dq[5] / 1000.0) + k_Ja_dot_tmp * c_Ja_dot_tmp *
                 dq[4] / 125.0) + l_Ja_dot_tmp * c_Ja_dot_tmp * dq[5] / 125.0) +
    n_Ja_dot_tmp * c_Ja_dot_tmp * dq[4] / 1000.0;
  e_Ja_dot_tmp = bb_Ja_dot_tmp_tmp - cb_Ja_dot_tmp_tmp;
  Ja_dot[13] = ((107.0 * std::cos(q[5]) * e_Ja_dot_tmp * dq[5] / 1000.0 - 107.0 *
                 std::sin(q[5]) * (((j_Ja_dot_tmp_tmp * q_Ja_dot_tmp_tmp +
    i_Ja_dot_tmp_tmp * ab_Ja_dot_tmp_tmp) + s_Ja_dot_tmp_tmp * dq[4]) +
    t_Ja_dot_tmp_tmp * dq[4]) / 1000.0) - 11.0 * std::cos(q[5]) * (((std::sin(q
    [4]) * ((((std::cos(q[0]) * std::sin(q[1]) * std::sin(q[3]) * dq[0] - std::
               sin(q[3]) * (std::cos(q[0]) * std::sin(q[2]) + std::cos(q[1]) *
    std::cos(q[2]) * std::sin(q[0])) * dq[3]) - std::cos(q[3]) * ((((std::sin(q
    [0]) * std::sin(q[2]) * dq[0] - std::cos(q[0]) * std::cos(q[2]) * dq[2]) +
    std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]) * dq[1]) + std::cos(q[1]) *
    std::sin(q[0]) * std::sin(q[2]) * dq[2]) - std::cos(q[0]) * std::cos(q[1]) *
    std::cos(q[2]) * dq[0])) + std::cos(q[1]) * std::sin(q[0]) * std::sin(q[3]) *
             dq[1]) + std::cos(q[3]) * std::sin(q[0]) * std::sin(q[1]) * dq[3])
    + std::cos(q[4]) * ((((std::cos(q[2]) * std::sin(q[0]) * dq[0] + std::cos(q
    [0]) * std::sin(q[2]) * dq[2]) + std::cos(q[0]) * std::cos(q[1]) * std::sin
    (q[2]) * dq[0]) + std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]) * dq[2])
                        - std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]) * dq
                        [1])) + std::sin(q[4]) * (std::cos(q[0]) * std::cos(q[2])
    - std::cos(q[1]) * std::sin(q[0]) * std::sin(q[2])) * dq[4]) + std::cos(q[4])
    * (std::cos(q[3]) * (std::cos(q[0]) * std::sin(q[2]) + std::cos(q[1]) * std::
    cos(q[2]) * std::sin(q[0])) + std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]))
    * dq[4]) / 125.0) - 11.0 * std::sin(q[5]) * e_Ja_dot_tmp * dq[5] / 125.0;
  Ja_dot[16] = ((((((107.0 * std::sin(q[5]) * r_Ja_dot_tmp_tmp / 1000.0 + 107.0 *
                     std::cos(q[5]) * f_Ja_dot_tmp / 1000.0) - 11.0 * std::sin
                    (q[5]) * f_Ja_dot_tmp / 125.0) + 11.0 * std::cos(q[5]) *
                   r_Ja_dot_tmp_tmp / 125.0) - 107.0 * std::cos(q[5]) *
                  c_Ja_dot_tmp * dq[5] / 1000.0) + 11.0 * std::sin(q[5]) *
                 c_Ja_dot_tmp * dq[5] / 125.0) - 11.0 * std::cos(q[5]) *
                g_Ja_dot_tmp * dq[5] / 125.0) - 107.0 * std::sin(q[5]) *
    g_Ja_dot_tmp * dq[5] / 1000.0;
  Ja_dot[19] = 0.0;
  Ja_dot[2] = 0.0;
  c_Ja_dot_tmp = std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]);
  Ja_dot_tmp_tmp_tmp = f_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp;
  b_Ja_dot_tmp_tmp_tmp = Ja_dot_tmp_tmp_tmp_tmp * h_Ja_dot_tmp_tmp;
  e_Ja_dot_tmp = Ja_dot_tmp_tmp_tmp - b_Ja_dot_tmp_tmp_tmp;
  c_Ja_dot_tmp_tmp_tmp = g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp;
  f_Ja_dot_tmp = c_Ja_dot_tmp_tmp_tmp + c_Ja_dot_tmp;
  g_Ja_dot_tmp = std::cos(q[2]) * std::cos(q[3]);
  m_Ja_dot_tmp = 48.0 * std::cos(q[1]) * std::cos(q[3]);
  Ja_dot[5] = (((((((((((((((((((107.0 * std::cos(q[5]) *
    ((((c_Ja_dot_tmp_tmp_tmp_tmp * dq[1] - c_Ja_dot_tmp_tmp_tmp * dq[3]) +
    Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[1]) +
    e_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2]) -
    c_Ja_dot_tmp * dq[3]) / 1000.0 - 79.0 * std::cos(q[1]) * dq[1] / 250.0) -
    11.0 * std::sin(q[5]) * ((((std::cos(q[1]) * std::cos(q[3]) * dq[1] - std::
    sin(q[1]) * std::sin(q[3]) * dq[3]) + std::cos(q[2]) * std::sin(q[1]) * std::
    sin(q[3]) * dq[1]) + std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]) * dq[2])
    - std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) * dq[3]) / 125.0) + 11.0 *
    std::cos(q[5]) * ((((i_Ja_dot_tmp_tmp * ((((g_Ja_dot_tmp * g_Ja_dot_tmp_tmp *
    dq[1] - Ja_dot_tmp_tmp_tmp * dq[3]) - e_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp *
    dq[1]) + c_Ja_dot_tmp_tmp_tmp_tmp * c_Ja_dot_tmp_tmp * dq[2]) +
    Ja_dot_tmp_tmp_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3]) + j_Ja_dot_tmp_tmp *
    f_Ja_dot_tmp * dq[4]) + Ja_dot_tmp_tmp_tmp_tmp * j_Ja_dot_tmp_tmp * dq[2]) +
                       e_Ja_dot_tmp_tmp * i_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp *
                       dq[4]) - g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp *
                      j_Ja_dot_tmp_tmp * dq[1]) / 125.0) + 107.0 * std::sin(q[5])
    * ((((std::cos(q[4]) * ((((std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]) *
    dq[1] - std::cos(q[3]) * std::sin(q[1]) * dq[3]) - std::cos(q[1]) * std::sin
    (q[3]) * dq[1]) + std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2]) * dq[2])
    + std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3]) * dq[3]) + std::sin(q[4])
    * (std::sin(q[1]) * std::sin(q[3]) + std::cos(q[1]) * std::cos(q[2]) * std::
    cos(q[3])) * dq[4]) + std::cos(q[1]) * std::cos(q[2]) * std::sin(q[4]) * dq
         [2]) + std::cos(q[1]) * std::cos(q[4]) * std::sin(q[2]) * dq[4]) - std::
       sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * dq[1]) / 1000.0) - 11.0 *
    std::cos(q[5]) * e_Ja_dot_tmp * dq[5] / 125.0) - 107.0 * std::sin(q[5]) *
    e_Ja_dot_tmp * dq[5] / 1000.0) - 107.0 * std::cos(q[5]) * (i_Ja_dot_tmp_tmp *
    f_Ja_dot_tmp - std::cos(q[1]) * std::sin(q[2]) * j_Ja_dot_tmp_tmp) * dq[5] /
    1000.0) + 11.0 * std::sin(q[5]) * (std::cos(q[4]) * (std::sin(q[1]) * std::
    sin(q[3]) + std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3])) - std::cos(q[1])
    * std::sin(q[2]) * std::sin(q[4])) * dq[5] / 125.0) - m_Ja_dot_tmp * dq[1] /
    125.0) + 33.0 * std::cos(q[2]) * g_Ja_dot_tmp_tmp * dq[1] / 400.0) + 33.0 *
                       std::cos(q[1]) * c_Ja_dot_tmp_tmp * dq[2] / 400.0) + 33.0
                      * std::cos(q[1]) * h_Ja_dot_tmp_tmp * dq[1] / 400.0) +
                     33.0 * std::cos(q[3]) * g_Ja_dot_tmp_tmp * dq[3] / 400.0) +
                    48.0 * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3] / 125.0)
                   - 33.0 * std::cos(q[2]) * std::cos(q[3]) * g_Ja_dot_tmp_tmp *
                   dq[1] / 400.0) - 33.0 * std::cos(q[1]) * std::cos(q[3]) *
                  c_Ja_dot_tmp_tmp * dq[2] / 400.0) - b_Ja_dot_tmp_tmp_tmp_tmp *
                 h_Ja_dot_tmp_tmp * dq[3] / 400.0) - 48.0 * std::cos(q[2]) *
                g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[1] / 125.0) - 48.0 *
               std::cos(q[1]) * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2] /
               125.0) + 48.0 * std::cos(q[1]) * std::cos(q[2]) *
    f_Ja_dot_tmp_tmp * dq[3] / 125.0;
  e_Ja_dot_tmp = 33.0 * std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]);
  f_Ja_dot_tmp = 48.0 * std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3]);
  o_Ja_dot_tmp = std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]);
  Ja_dot[8] = ((((((((((((((((((11.0 * std::cos(q[5]) * ((((((std::cos(q[1]) *
    std::cos(q[2]) * std::sin(q[4]) * dq[1] + Ja_dot_tmp_tmp * i_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * dq[4]) - o_Ja_dot_tmp * dq[2]) + c_Ja_dot_tmp_tmp_tmp_tmp
    * i_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[1]) + g_Ja_dot_tmp *
    i_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp * dq[2]) - i_Ja_dot_tmp_tmp *
    g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[3]) -
    Ja_dot_tmp_tmp_tmp * c_Ja_dot_tmp_tmp * j_Ja_dot_tmp_tmp * dq[4]) / 125.0 +
    107.0 * std::sin(q[5]) * ((((((std::cos(q[1]) * std::cos(q[2]) * std::sin(q
    [4]) * dq[1] + std::cos(q[2]) * std::cos(q[4]) * std::sin(q[1]) * dq[4]) -
    std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * dq[2]) + std::cos(q[1]) *
    std::cos(q[3]) * std::cos(q[4]) * std::sin(q[2]) * dq[1]) + std::cos(q[2]) *
    std::cos(q[3]) * std::cos(q[4]) * std::sin(q[1]) * dq[2]) - std::cos(q[4]) *
    std::sin(q[1]) * std::sin(q[2]) * std::sin(q[3]) * dq[3]) - std::cos(q[3]) *
    std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * dq[4]) / 1000.0) + 33.0 *
    std::cos(q[1]) * std::sin(q[2]) * dq[1] / 400.0) + 33.0 * std::cos(q[2]) *
    std::sin(q[1]) * dq[2] / 400.0) + 107.0 * std::cos(q[5]) * (std::cos(q[2]) *
    std::sin(q[1]) * j_Ja_dot_tmp_tmp + d_Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp *
    c_Ja_dot_tmp_tmp) * dq[5] / 1000.0) - 11.0 * std::sin(q[5]) * (std::cos(q[2])
    * std::sin(q[1]) * std::sin(q[4]) + std::cos(q[3]) * std::cos(q[4]) * std::
    sin(q[1]) * std::sin(q[2])) * dq[5] / 125.0) - 33.0 * std::cos(q[1]) * std::
    cos(q[3]) * std::sin(q[2]) * dq[1] / 400.0) - e_Ja_dot_tmp * dq[2] / 400.0)
    - 48.0 * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]) * dq[1] / 125.0) -
                        f_Ja_dot_tmp * dq[2] / 125.0) - 48.0 * std::cos(q[3]) *
                       g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * dq[3] / 125.0) +
                      33.0 * g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp *
                      h_Ja_dot_tmp_tmp * dq[3] / 400.0) + 107.0 *
                     e_Ja_dot_tmp_tmp * Ja_dot_tmp * c_Ja_dot_tmp_tmp *
                     h_Ja_dot_tmp_tmp * dq[1] / 1000.0) + 107.0 * Ja_dot_tmp_tmp
                    * Ja_dot_tmp * g_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[2] /
                    1000.0) + h_Ja_dot_tmp * g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp
                   * dq[3] / 1000.0) - 11.0 * e_Ja_dot_tmp_tmp *
                  c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * b_Ja_dot_tmp * dq[1] /
                  125.0) - 11.0 * Ja_dot_tmp_tmp * g_Ja_dot_tmp_tmp *
                 h_Ja_dot_tmp_tmp * b_Ja_dot_tmp * dq[2] / 125.0) - 11.0 * std::
                cos(q[3]) * g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * b_Ja_dot_tmp *
                dq[3] / 125.0) - 11.0 * std::cos(q[5]) * g_Ja_dot_tmp_tmp *
               c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * dq[5] / 125.0) - 107.0 *
    g_Ja_dot_tmp_tmp * c_Ja_dot_tmp_tmp * h_Ja_dot_tmp_tmp * b_Ja_dot_tmp * dq[5]
    / 1000.0;
  Ja_dot_tmp_tmp = std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3]);
  Ja_dot_tmp = (((c_Ja_dot_tmp_tmp_tmp_tmp * dq[3] - c_Ja_dot_tmp_tmp_tmp * dq[1])
                 + std::cos(q[3]) * std::sin(q[1]) * std::sin(q[2]) * dq[2]) +
                Ja_dot_tmp_tmp * dq[3]) - c_Ja_dot_tmp * dq[1];
  b_Ja_dot_tmp_tmp = std::cos(q[1]) * std::sin(q[3]);
  c_Ja_dot_tmp_tmp = std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]);
  b_Ja_dot_tmp = b_Ja_dot_tmp_tmp - c_Ja_dot_tmp_tmp;
  c_Ja_dot_tmp = c_Ja_dot_tmp_tmp_tmp_tmp + Ja_dot_tmp_tmp;
  g_Ja_dot_tmp = (((Ja_dot_tmp_tmp_tmp * dq[1] + b_Ja_dot_tmp_tmp * dq[3]) -
                   b_Ja_dot_tmp_tmp_tmp * dq[1]) - c_Ja_dot_tmp_tmp * dq[3]) +
    std::sin(q[1]) * std::sin(q[2]) * h_Ja_dot_tmp_tmp * dq[2];
  Ja_dot[11] = ((((((((((((((((((107.0 * std::cos(q[5]) * Ja_dot_tmp / 1000.0 -
    11.0 * std::sin(q[5]) * Ja_dot_tmp / 125.0) - d_Ja_dot_tmp *
    ((((Ja_dot_tmp_tmp_tmp * dq[1] + b_Ja_dot_tmp_tmp * dq[3]) -
    b_Ja_dot_tmp_tmp_tmp * dq[1]) - c_Ja_dot_tmp_tmp * dq[3]) + std::sin(q[1]) *
     std::sin(q[2]) * h_Ja_dot_tmp_tmp * dq[2]) / 1000.0) - 11.0 * std::cos(q[5])
    * b_Ja_dot_tmp * dq[5] / 125.0) - 107.0 * std::sin(q[5]) * b_Ja_dot_tmp *
    dq[5] / 1000.0) - m_Ja_dot_tmp * dq[3] / 125.0) + 33.0 * std::cos(q[3]) *
    std::sin(q[1]) * dq[1] / 400.0) + 33.0 * std::cos(q[1]) * std::sin(q[3]) *
    dq[3] / 400.0) + 48.0 * std::sin(q[1]) * std::sin(q[3]) * dq[1] / 125.0) -
    i_Ja_dot_tmp * g_Ja_dot_tmp / 125.0) - 33.0 * std::cos(q[1]) * std::cos(q[2])
                        * std::sin(q[3]) * dq[1] / 400.0) - e_Ja_dot_tmp * dq[3]
                       / 400.0) - 48.0 * std::cos(q[3]) * std::sin(q[1]) * std::
                      sin(q[2]) * dq[2] / 125.0) - f_Ja_dot_tmp * dq[3] / 125.0)
                    + 33.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[3]) *
                    dq[2] / 400.0) + j_Ja_dot_tmp * c_Ja_dot_tmp * dq[5] /
                   1000.0) - k_Ja_dot_tmp * c_Ja_dot_tmp * dq[4] / 125.0) -
                 l_Ja_dot_tmp * c_Ja_dot_tmp * dq[5] / 125.0) - n_Ja_dot_tmp *
                c_Ja_dot_tmp * dq[4] / 1000.0) + 48.0 * std::cos(q[1]) * std::
    cos(q[2]) * std::cos(q[3]) * dq[1] / 125.0;
  d_Ja_dot_tmp = std::cos(q[4]) * std::sin(q[1]) * std::sin(q[2]);
  Ja_dot[14] = ((11.0 * std::sin(q[5]) * (j_Ja_dot_tmp_tmp * b_Ja_dot_tmp -
    d_Ja_dot_tmp) * dq[5] / 125.0 - 11.0 * std::cos(q[5]) * ((((j_Ja_dot_tmp_tmp
    * Ja_dot_tmp + i_Ja_dot_tmp_tmp * b_Ja_dot_tmp * dq[4]) - std::cos(q[1]) *
    std::cos(q[4]) * std::sin(q[2]) * dq[1]) - std::cos(q[2]) * std::cos(q[4]) *
    std::sin(q[1]) * dq[2]) + o_Ja_dot_tmp * dq[4]) / 125.0) - 107.0 * std::cos
                (q[5]) * (std::sin(q[4]) * (std::cos(q[1]) * std::sin(q[3]) -
    std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1])) - std::cos(q[4]) * std::
    sin(q[1]) * std::sin(q[2])) * dq[5] / 1000.0) - 107.0 * std::sin(q[5]) *
    ((((std::sin(q[4]) * ((((std::cos(q[1]) * std::cos(q[3]) * dq[3] - std::sin
            (q[1]) * std::sin(q[3]) * dq[1]) + std::cos(q[3]) * std::sin(q[1]) *
           std::sin(q[2]) * dq[2]) + std::cos(q[2]) * std::sin(q[1]) * std::sin
          (q[3]) * dq[3]) - std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
         dq[1]) + std::cos(q[4]) * (std::cos(q[1]) * std::sin(q[3]) - std::cos
         (q[2]) * std::cos(q[3]) * std::sin(q[1])) * dq[4]) - std::cos(q[1]) *
       std::cos(q[4]) * std::sin(q[2]) * dq[1]) - std::cos(q[2]) * std::cos(q[4])
      * std::sin(q[1]) * dq[2]) + std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])
     * dq[4]) / 1000.0;
  b_Ja_dot_tmp = std::cos(q[4]) * (std::cos(q[1]) * std::sin(q[3]) - std::cos(q
    [2]) * std::cos(q[3]) * std::sin(q[1])) + o_Ja_dot_tmp;
  Ja_dot[17] = ((((((107.0 * std::cos(q[5]) * ((((i_Ja_dot_tmp_tmp * Ja_dot_tmp
    - std::sin(q[4]) * (std::cos(q[1]) * std::sin(q[3]) - std::cos(q[2]) * std::
                        cos(q[3]) * std::sin(q[1])) * dq[4]) + std::cos(q[1]) *
    std::sin(q[2]) * std::sin(q[4]) * dq[1]) + std::cos(q[2]) * std::sin(q[1]) *
    std::sin(q[4]) * dq[2]) + d_Ja_dot_tmp * dq[4]) / 1000.0 - 11.0 * std::sin
                     (q[5]) * ((((std::cos(q[4]) * ((((std::cos(q[1]) * std::cos
    (q[3]) * dq[3] - std::sin(q[1]) * std::sin(q[3]) * dq[1]) + std::cos(q[3]) *
    std::sin(q[1]) * std::sin(q[2]) * dq[2]) + std::cos(q[2]) * std::sin(q[1]) *
    std::sin(q[3]) * dq[3]) - std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
    dq[1]) - std::sin(q[4]) * (std::cos(q[1]) * std::sin(q[3]) - std::cos(q[2]) *
    std::cos(q[3]) * std::sin(q[1])) * dq[4]) + std::cos(q[1]) * std::sin(q[2]) *
    std::sin(q[4]) * dq[1]) + std::cos(q[2]) * std::sin(q[1]) * std::sin(q[4]) *
    dq[2]) + std::cos(q[4]) * std::sin(q[1]) * std::sin(q[2]) * dq[4]) / 125.0)
                    - 11.0 * std::cos(q[5]) * g_Ja_dot_tmp / 125.0) - 107.0 *
                   std::sin(q[5]) * g_Ja_dot_tmp / 1000.0) + 107.0 * std::cos(q
    [5]) * c_Ja_dot_tmp * dq[5] / 1000.0) - 11.0 * std::sin(q[5]) * c_Ja_dot_tmp
                 * dq[5] / 125.0) - 11.0 * std::cos(q[5]) * b_Ja_dot_tmp * dq[5]
                / 125.0) - 107.0 * std::sin(q[5]) * b_Ja_dot_tmp * dq[5] /
    1000.0;
  Ja_dot[20] = 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
void get_Ja_dot_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void get_Ja_dot_terminate()
{
  // (no terminate code required)
}

//
// File trailer for get_Ja_dot.cpp
//
// [EOF]
//
