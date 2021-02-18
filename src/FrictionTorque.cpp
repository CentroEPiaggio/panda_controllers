/*
----- Code based on:
% author: Claudio Gaz, Marco Cognetti, Alexander Oliva
% date: September 3, 2019
% 
% -------------------------------------------------
% Panda Dynamic Model v. 1.0
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
*/

#include <utils/FrictionTorque.h>
#include <math.h>

void get_friction_torque(const double dq[7], double tau_F[7]){

    double A[7] = {0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489};
    double k[7] = {0, 0, 0, 0, 0, 0, 0};
    double qdotsign[7] = {0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047, 0.0035526};
    double alpha[7] = {5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336};
    
    for(int i=0; i<7; i++){
        tau_F[i] = -A[i]/(1+exp(-alpha[i]*qdotsign[i])) + A[i]/(1+exp(-alpha[i]*(dq[i]+qdotsign[i])));
    }
}
