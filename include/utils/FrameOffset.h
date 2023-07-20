#ifndef FrameOffsetLIB_H
#define FrameOffsetLIB_H

#include <iostream>
#include <casadi/casadi.hpp>
#include <cmath>
#include <stdexcept>

/* FrameOffset CLASS */


typedef std::vector<double> vec3d;

namespace regrob{

    /* Set or get a homogenous transformation variables of offset frame*/
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
            FrameOffset(const vec3d& ,const vec3d&,const vec3d&);
            
            /* Set [yaw,pitch,roll], [transl_x,transl_y,transl_z]
            All transformation is referred in world FrameOffset, default gravity is [0,0,-9.81] */
            FrameOffset(const vec3d& ,const vec3d&);

            /* Destructor */
            ~FrameOffset(){std::cout<<"FrameOffset destruct\n";};

            /* Get translation vector of FrameOffset */
            void set_translation(const vec3d&);

            /* Set yaw, pitch, roll vector of FrameOffset */
            void set_ypr(const vec3d&);

            /* Set gravity of FrameOffset */
            void set_gravity(const vec3d&);

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