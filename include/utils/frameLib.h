#ifndef FRAMELIB_H
#define FRAMELIB_H

#include <iostream>
#include <casadi/casadi.hpp>
#include <cmath>
#include <stdexcept>

typedef std::vector<double> vec3d;

namespace regrob{
    /* frame is used to set offset between frame of link 0 of robot and world-frame */
    class frame {
        private:
            vec3d ypr;
            vec3d position;
            vec3d gravity;
            casadi::SX hat(const vec3d&);
        public:
            frame();
            frame(const vec3d& ,const vec3d&,const vec3d&);
            frame(const vec3d& ,const vec3d&);
            ~frame(){std::cout<<"frame eliminato\n";};

            void set_position(const vec3d&);
            void set_ypr(const vec3d&);
            void set_gravity(const vec3d&);

            vec3d get_ypr();
            casadi::SX get_rotation();
            casadi::SX get_translation();
            casadi::SX get_transform();
            casadi::SX get_gravity();
    };
}
#endif