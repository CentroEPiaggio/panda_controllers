#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#ifndef     NJ
#define     NJ 7	    // number of joints
#endif

#ifndef     PARAM
#define     PARAM 10	// number of parameters for each link
#endif

namespace regrob{
    
    inline Eigen::Matrix3d hat(const Eigen::Vector3d v){
        Eigen::Matrix3d vhat;
                
        // chech
        if(v.size() != 3 ){
            std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
        }
        
        vhat(0,0) = 0;
        vhat(0,1) = -v[2];
        vhat(0,2) = v[1];
        vhat(1,0) = v[2];
        vhat(1,1) = 0;
        vhat(1,2) = -v[0];
        vhat(2,0) = -v[1];
        vhat(2,1) = v[0];
        vhat(2,2) = 0;

        return vhat;
    }

    inline Eigen::Matrix3d rpyRot(const std::vector<double> rpy){

        Eigen::Matrix3d rotTr;
        
        double cy = cos(rpy[2]);
        double sy = sin(rpy[2]);
        double cp = cos(rpy[1]);
        double sp = sin(rpy[1]);
        double cr = cos(rpy[0]);
        double sr = sin(rpy[0]);

        //template R yaw-pitch-roll
        rotTr(0,0)=cy*cp;
        rotTr(0,1)=cy*sp*sr-sy*cr;
        rotTr(0,2)=cy*sp*cr-sy*sr;
        rotTr(1,0)=sy*cp;
        rotTr(1,1)=sy*sp*sr+cy*cr;
        rotTr(1,2)=sy*sp*cr-cy*sr;
        rotTr(2,0)=-sp;
        rotTr(2,1)=cp*sr;
        rotTr(2,2)=cp*cr;

        return rotTr;
    }
    
    inline Eigen::Matrix3d createI(const std::vector<double> parI){

        if(parI.size() != 6){
            std::cout<<"\nin function createI: invalid dimension of parI\n";
            return Eigen::Matrix3d::Identity();
        }

        Eigen::Matrix3d I;
        I(0, 0) = parI[0];
        I(0, 1) = parI[1];
        I(0, 2) = parI[2];
        I(1, 0) = parI[1];
        I(1, 1) = parI[3];
        I(1, 2) = parI[4];
        I(2, 0) = parI[2];
        I(2, 1) = parI[4];
        I(2, 2) = parI[5];

        return I;
    }
    
    inline void dyn2reg(const int n, const int np, Eigen::Matrix<double, NJ*PARAM, 1> param_dyn,  Eigen::Matrix<double, NJ*PARAM, 1> &param_reg){ 

        double mi;
        std::vector<double> parI(6);
        Eigen::Vector3d OiGi;
        Eigen::Matrix3d IGi;
        Eigen::Matrix3d IOi;
        Eigen::Matrix3d OiGi_hat;

        for(int i=0;i<n;i++){
            mi = param_dyn(i*np,0);
            OiGi << param_dyn(i*np+1,0), param_dyn(i*np+2,0), param_dyn(i*np+3,0);
            parI = {param_dyn(i*np+4,0), param_dyn(i*np+5,0), param_dyn(i*np+6,0), param_dyn(i*np+7,0), param_dyn(i*np+8,0), param_dyn(i*np+9,0)};
            IGi = createI(parI);
            OiGi_hat = hat(OiGi);

            IOi = IGi + mi*OiGi_hat.transpose()*OiGi_hat;

            param_reg(i*np,0) = mi;
            param_reg(i*np+1,0) = mi*OiGi(0);
            param_reg(i*np+2,0) = mi*OiGi(1);
            param_reg(i*np+3,0) = mi*OiGi(2);
            param_reg(i*np+4,0) = IGi(0,0);
            param_reg(i*np+5,0) = IGi(0,1);
            param_reg(i*np+6,0) = IGi(0,2);
            param_reg(i*np+7,0) = IGi(1,1);
            param_reg(i*np+8,0) = IGi(1,2);
            param_reg(i*np+9,0) = IGi(2,2);
        }
    }

    inline void reg2dyn(const int n, const int np, Eigen::Matrix<double, NJ*PARAM, 1> param_reg,  Eigen::Matrix<double, NJ*PARAM, 1> &param_dyn){ 

        double mi;
        std::vector<double> parI(6);
        Eigen::Vector3d OiGi;
        Eigen::Matrix3d IGi;
        Eigen::Matrix3d IOi;
        Eigen::Matrix3d OiGi_hat;

        for(int i=0;i<n;i++){
            mi = param_reg(i*np,0);
            OiGi << param_reg(i*np+1,0), param_reg(i*np+2,0), param_reg(i*np+3,0);
            OiGi = OiGi/mi;
            parI = {param_reg(i*np+4,0), param_reg(i*np+5,0), param_reg(i*np+6,0), param_reg(i*np+7,0), param_reg(i*np+8,0), param_reg(i*np+9,0)};
            IOi = createI(parI);
            OiGi_hat = hat(OiGi);

            IGi = IOi - mi*OiGi_hat.transpose()*OiGi_hat;

            param_dyn(i*np,0) = mi;
            param_dyn(i*np+1,0) = OiGi(0);
            param_dyn(i*np+2,0) = OiGi(1);
            param_dyn(i*np+3,0) = OiGi(2);
            param_dyn(i*np+4,0) = IOi(0,0);
            param_dyn(i*np+5,0) = IOi(0,1);
            param_dyn(i*np+6,0) = IOi(0,2);
            param_dyn(i*np+7,0) = IOi(1,1);
            param_dyn(i*np+8,0) = IOi(1,2);
            param_dyn(i*np+9,0) = IOi(2,2);
            std::cout<<"\nmi "<<i<<" = "<<mi<<"\n";
            std::cout<<"\nOiGi "<<i<<" = \n"<<OiGi<<"\n";
            std::cout<<"\n ============================== \n";

        }
    }


}