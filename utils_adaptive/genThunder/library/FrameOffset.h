#ifndef FRAMEOFFSET_H
#define FRAMEOFFSET_H

#include <iostream>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>

/* FrameOffset CLASS */

typedef std::vector<double> vec3d;

namespace regrob{

    /* Homogenous transformation of offset frame */
    class FrameOffset {

        private:

            /* [yaw, pitch, roll] */
            vec3d ypr;
            /* [transl_x,transl_y,transl_z] */
            vec3d translation;
            /* [grav_x,grav_y,grav_z] */
            vec3d gravity;
            /* Operator hat */
            casadi::SX hat(const vec3d&);

        public:

            /* Empty constructor */
            FrameOffset();
            /* Set [yaw,pitch,roll], [transl_x,transl_y,transl_z], [grav_x,grav_y,grav_z]
            All transformation is referred in world FrameOffset */
            FrameOffset(const vec3d& ypr,const vec3d& transl,const vec3d& gravity);
            /* Set [yaw,pitch,roll], [transl_x,transl_y,transl_z]
            All transformation is referred in world FrameOffset */
            FrameOffset(const vec3d& ,const vec3d&);
            
            FrameOffset(const Eigen::Matrix4d& transformationMatrix);
            /* Destructor */
            ~FrameOffset(){};
            
            /* Get translation vector of FrameOffset */
            void set_translation(const vec3d&);
            /* Set yaw, pitch, roll vector of FrameOffset */
            void set_ypr(const vec3d&);
            /* Set gravity of FrameOffset */
            void set_gravity(const vec3d&);

            void set_homogenousT(const Eigen::Matrix4d& matrixT);
            
            /* Get yaw, pitch, roll vector of FrameOffset */
            vec3d get_ypr();
            /* Get rotation matrix of FrameOffset */
            casadi::SX get_rotation();
            /* Get translation vector of FrameOffset */
            casadi::SX get_translation();
            /* Get homogeneous transformation matrix of FrameOffset */
            casadi::SX get_transform();
            /* Get gravity of FrameOffset */
            casadi::SX get_gravity();

    };
}
#endif