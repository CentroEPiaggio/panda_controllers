#ifndef KINEMATICS_UTILS_H
#define KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include "thunder_frankino.h"

struct JointStateTarget
{
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
};

class KinematicsUtils
{
public:
    static JointStateTarget computeFullTarget(
        thunder_frankino &robot,
        const Eigen::Vector3d &p_target,
        const Eigen::Vector3d &v_target,
        const Eigen::VectorXd &q_start,
        const Eigen::VectorXd &dq_start,
        const Eigen::VectorXd &ddq_start)
    {
        int nj = 7; // Franka ha 7 DOF
        JointStateTarget target;
        
        // --- 1. CALCOLO POSIZIONE (q target) ---
        // Usiamo un metodo iterativo Newton-Raphson puro per trovare la q finale
        Eigen::VectorXd q_res = q_start; 
        double eps = 1e-6;      // Tolleranza errore cartesiano (1 micron)
        int max_iter = 150;     // Più iterazioni per garantire precisione nel lancio
        double damping = 0.01;  // Damping per stabilità vicino alle singolarità

        for (int i = 0; i < max_iter; i++)
        {
            robot.set_q(q_res);
            // Ottieni posizione attuale dell'EE
            Eigen::Vector3d p_curr = robot.get_T_0_ee().block<3, 1>(0, 3);
            Eigen::Vector3d error = p_target - p_curr;

            if (error.norm() < eps) break;

            // Jacobiano lineare (3x7)
            Eigen::Matrix<double, 3, 7> J_lin = robot.get_J_ee().block<3, 7>(0, 0);
            
            // Pseudo-inversa con Damped Least Squares (DLS)
            // dq = J^T * (J*J^T + lambda^2*I)^-1 * error
            Eigen::Matrix3d JJT = J_lin * J_lin.transpose();
            Eigen::MatrixXd J_pinv = J_lin.transpose() * (JJT + damping * Eigen::Matrix3d::Identity()).inverse();

            q_res += J_pinv * error;
        }
        target.q = q_res;

        // --- 2. CALCOLO VELOCITÀ (dq target) ---
        // Per il lancio: v_ee = J * dq  => dq = J# * v_target
        // Usiamo lo Jacobiano calcolato nella configurazione finale target.q
        robot.set_q(target.q);
        Eigen::Matrix<double, 3, 7> J_final = robot.get_J_ee().block<3, 7>(0, 0);
        
        Eigen::Matrix3d JJT_f = J_final * J_final.transpose();
        Eigen::MatrixXd J_final_pinv = J_final.transpose() * (JJT_f + damping * Eigen::Matrix3d::Identity()).inverse();

        target.dq = J_final_pinv * v_target;

        // --- 3. ACCELERAZIONE ---
        target.ddq = Eigen::VectorXd::Zero(nj);

        return target;
    }
};

#endif
