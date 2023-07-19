#include "utils/frameLib.h"

namespace regrob{
    
    frame::frame(){
        ypr.resize(3);
        position.resize(3);
        gravity.resize(3);        
    }
    frame::frame(const vec3d& ypr_,const vec3d& pos_,const vec3d& g_){
        ypr.resize(3);
        position.resize(3);
        gravity.resize(3);        
        if(ypr_.size()==3 && pos_.size()==3 && g_.size()==3){
        ypr = ypr_;
        position = pos_;
        gravity = g_;              
        }else{
            throw std::runtime_error("in costructor frame: invalid dimension of arguments");
        }
    };
    frame::frame(const vec3d& ypr_,const vec3d& pos_){
        ypr.resize(3);
        position.resize(3);
        gravity.resize(3);        
        if(ypr_.size()==3 && pos_.size()==3){
        ypr = ypr_;
        position = pos_;
        gravity = {0,0,-9.81};              
        }else{
            throw std::runtime_error("in costructor frame: invalid dimension of arguments");
        }
    };
    
    vec3d frame::get_ypr(){return ypr;}
    casadi::SX frame::get_rotation(){
        casadi::SX R = casadi::SX::eye(3);
        double cy = cos(ypr[0]);
        double sy = sin(ypr[0]);
        double cp = cos(ypr[1]);
        double sp = sin(ypr[1]);
        double cr = cos(ypr[2]);
        double sr = sin(ypr[2]);

        //template R yaw-pitch-roll
        R(0,0)=cy*cp;
        R(0,1)=cy*sp*sr-sy*cr;
        R(0,2)=cy*sp*cr-sy*sr;
        R(1,0)=sy*cp;
        R(1,1)=sy*sp*sr+cy*cr;
        R(1,2)=sy*sp*cr-cy*sr;
        R(2,0)=-sp;
        R(2,1)=cp*sr;
        R(2,2)=cp*cr;

        return R;
    };
    casadi::SX frame::get_translation(){
        casadi::SX pos(3,1);
        pos(0,0) = position[0];
        pos(1,0) = position[1];
        pos(2,0) = position[2];
        return pos;
        //return {{position[0]},{position[1]},{position[2]}};
    }
    casadi::SX frame::get_transform(){
        casadi::SX rotTr = casadi::SX::eye(4);
        
        double cy = cos(ypr[0]);
        double sy = sin(ypr[0]);
        double cp = cos(ypr[1]);
        double sp = sin(ypr[1]);
        double cr = cos(ypr[2]);
        double sr = sin(ypr[2]);
        double dx = position[0];
        double dy = position[1];
        double dz = position[2];

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
        rotTr(0,3) = dx;
        rotTr(1,3) = dy;
        rotTr(2,3) = dz;

        return rotTr;
    }
    casadi::SX frame::get_gravity(){
        casadi::SX SXg(3,1);
        SXg(0,0) = gravity[0];
        SXg(1,0) = gravity[1];
        SXg(2,0) = gravity[2];
        return SXg;
        //return {{gravity[0]},{gravity[1]},{gravity[2]}};
    }
    
    void frame::set_gravity(const vec3d& g_){gravity = g_;}
    void frame::set_position(const vec3d& pos_){position = pos_;}
    void frame::set_ypr(const vec3d& ypr_){ypr = ypr_;}
    
    casadi::SX frame::hat(const vec3d& v){
        if(v.size() != 3 ){
            std::cout<<"in function hat of class frame invalid dimension of input"<<std::endl;
        }
        casadi::SX vhat(3,3);
        vhat(0,0) = 0;
        vhat(0,1) = v[3];
        vhat(0,2) = v[2];
        vhat(1,0) = -v[2];
        vhat(1,1) = 0;
        vhat(1,2) = v[0];
        vhat(2,0) = -v[1];
        vhat(2,1) = -v[0];
        vhat(2,2) = 0;

        return vhat;
    }
}