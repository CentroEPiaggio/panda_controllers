#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

#include "utils/myLibReg.h"

namespace regrob{ 

    #define TOLL 1e-15

    // Function to generate matrix template of transformation from Denavit-Hartenberg parameterization
    casadi::MX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::MX q, char jtsType) {

        // Check
        if (rowDHTable.cols() != 4) {
            throw std::runtime_error("DHTemplate: Error size of DH table");
        }

        // Variables
        double a;           // DH table format [a alpha d theta]
        double alpha;
        casadi::MX d;
        casadi::MX theta;
        casadi::MX ct;      // cos(theta)
        casadi::MX st;      // sin(theta)
        double ca;          // cos(alpha)
        double sa;          // sin(alpha)
        casadi::MX Ti(4,4); // output

        // Prismatic or Rotoidal joint
        if (jtsType == 'P') {
            d = rowDHTable(2) + q;
            theta = rowDHTable(3);
        }
        else if (jtsType == 'R') {
            d = rowDHTable(2);
            theta = rowDHTable(3) + q;
        }
        else {
            throw std::runtime_error("DHTemplate: Error joint type");
        }

        a = rowDHTable(0);
        alpha = rowDHTable(1);
        ct = cos(theta);
        st = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);

        // Template DH
        Ti(0,0) = ct;
        Ti(0,1) = -ca*st;
        Ti(0,2) = sa*st;
        Ti(0,3) = a*ct;
        Ti(1,0) = st;
        Ti(1,1) = ca*ct;
        Ti(1,2) = -sa*ct;
        Ti(1,3) = a*st;
        Ti(2,1) = sa;
        Ti(2,2) = ca;
        Ti(2,3) = d;
        Ti(3,3) = 1;

        return Ti;
    }

    // Function for Forward Kinematic from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHFwKin(const Eigen::MatrixXd& DHTable, const casadi::MX& q, const std::string& jtsType) {
        
        // Number of joints
        int nj = q.size1();
        //std::cout<<"nj: "<<nj<<std::endl;
        //std::cout<<"sixe jtypes: "<<(int)jtsType.size()<<std::endl;

        // Outputs
        casadi::MXVector Ti(nj);
        casadi::MXVector T0i(nj);
        
        // Check of dimensions
        if ((int)q.size1() != (int)jtsType.size()) {
            throw std::runtime_error("DHFwkin: Error size of joints string");
        }
        if (q.size1() != DHTable.rows()) {
            throw std::runtime_error("DHFwkin: Error size of DH table");
        }    
        
        // Creazione di T0i
        Ti[0] = DHTemplate(DHTable.row(0), q(0), jtsType[0]);
        T0i[0] = Ti[0];

        for (int i = 1; i < nj; ++i) {
            Ti[i] = DHTemplate(DHTable.row(i), q(i), jtsType[i]);
            T0i[i] = T0i[i - 1] * Ti[i];
        }

        return std::make_tuple(T0i, Ti);
    }

    // Function hat
    casadi::MX hat(const casadi::MX& v) {
        
        casadi::MX skew(3, 3);
        skew(0, 0) = 0;
        skew(0, 1) = -v(2);
        skew(0, 2) = v(1);
        skew(1, 0) = v(2);
        skew(1, 1) = 0;
        skew(1, 2) = -v(0);
        skew(2, 0) = -v(1);
        skew(2, 1) = v(0);
        skew(2, 2) = 0;

        return skew;
    }

    // Function for Jacobian of link i-th from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHJac(const casadi::MXVector& T0i_vec, const std::string& jtsType, frame& base_frame) {
        
        // Number of joints
        int nj = T0i_vec.size();

        // Check
        if (nj != (int)jtsType.size()) {
            throw std::runtime_error("DHJac: Error joints' string size");
        }

        // Variables 
        casadi::MX Ji_pos(3, nj);       // matrix of velocity jacobian
        casadi::MX Ji_or(3, nj);        // matrix of omega jacobian
        casadi::MXVector Ji_v(nj);       // vector of matrix
        casadi::MXVector Ji_w(nj);       // vector of matrix
        casadi::Slice r_tra_idx(0, 3);  // select translation vector of T()
        casadi::Slice c_tra_idx(3);
        casadi::Slice r_rot_idx(0, 3);  // select k versor of T()
        casadi::Slice c_rot_idx(2);
        casadi::Slice allRows;          // Select all rows

        for (int i = 0; i < nj; i++) {

            // Init variables of first column jacobian
            casadi::MX k0(3,1);             // versor of joint i
            casadi::MX O_0i(3,1);           // distance of joint i from joint 0
            casadi::MX T_0i(4,4);           // matrix tranformation of joint 1 from joint 0

            // First column of jacobian
            //k0 = casadi::MX::vertcat({0, 0, 1});
            k0(2,0) = casadi::MX::ones(1,1);
            T_0i = T0i_vec[i];
            O_0i = T_0i(r_tra_idx, c_tra_idx);

            if (jtsType[0] == 'P') {
                Ji_pos(allRows,0) = k0;
            } else if (jtsType[0] == 'R') {
                Ji_pos(allRows,0) = mtimes(hat(k0),O_0i);           // controlla che faccia quello che deve fare
                Ji_or(allRows,0) = k0;
            } else {
                throw std::runtime_error("DHJac: Error joint type");
            }

            // Rest of columns of jacobian
            for (int j = 1; j <= i; j++) {
                
                // Init variables of column j-th of jacobian each cycle
                casadi::MX kj_1(3,1);             // versor of joint j-1
                casadi::MX O_j_1i(3,1);           // distance of joint i from joint j-1
                casadi::MX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1
        
                T_0j_1 = T0i_vec[j-1];
                kj_1 = T_0j_1(r_rot_idx, c_rot_idx);
                O_j_1i = O_0i - T_0j_1(r_tra_idx, c_tra_idx);

                if (jtsType[j] == 'P') {
                    Ji_pos(allRows, j) = kj_1;
                } else if (jtsType[j] == 'R') {
                    Ji_pos(allRows, j) = mtimes(hat(kj_1),O_j_1i);
                    Ji_or(allRows, j) = kj_1;
                } else {
                    throw std::runtime_error("DHJac: Error joint type");
                }
            }

            Ji_pos = mtimes(base_frame.rotation(),Ji_pos);
            Ji_or = mtimes(base_frame.rotation(),Ji_or);
            Ji_v[i] = Ji_pos;
            Ji_w[i] = Ji_or;
        }

        return std::make_tuple(Ji_v, Ji_w);
    }

    // Function to create dq_selection matrix
    casadi::MX dq_select(const casadi::MX& dq) {
        
        int n = dq.size1();
        casadi::Slice allRows;          // Select all rows
        
        casadi::MX mat_dq = casadi::MX::zeros(n, n * n);
        for (int i = 0; i < n; i++) {
            casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
            mat_dq(allRows, sel_col) = casadi::MX::eye(n)*dq(i);
        }
        return mat_dq;
    }
    // Function to create Q
    casadi::MXVector createQ() {

        casadi::MXVector Q(3);

        for(int i=0;i<3;i++){
            Q[i] = casadi::MX::zeros(3,3);
        }
        Q[0](1,2) = -1;
        Q[0](2,1) = 1;

        Q[1](0,2) = 1;
        Q[1](2,0) = -1;

        Q[2](0,1) = -1;
        Q[2](1,0) = 1;

        return Q;
    }

    // Function to create E
    casadi::MXVector createE() {
        
        casadi::MXVector E(6);

        for(int i=0;i<6;i++){
            E[i] = casadi::MX::zeros(3,3);
        }
        E[0](0,0) = 1;

        E[1](0,1) = 1;
        E[1](1,0) = 1;

        E[2](0,2) = 1;
        E[2](2,0) = 1;

        E[3](1,1) = 1;

        E[4](1,2) = 1;
        E[4](2,1) = 1;

        E[5](2,2) = 1;

        return E;
    }
    /*
    // Function to create Q
    casadi::MXVector createQ() {

        casadi::MXVector Q(3);
        Q[0] = casadi::MX(casadi::DM({
            {0, 0, 0},
            {0, 0, -1},
            {0, 1, 0}
        }));

        Q[1] = casadi::MX(casadi::DM({
            {0, 0, 1},
            {0, 0, 0},
            {-1, 0, 0}
        }));

        Q[2] = casadi::MX(casadi::DM({
            {0, -1, 0},
            {1, 0, 0},
            {0, 0, 0}
        }));

        return Q;
    }

    // Function to create E
    casadi::MXVector createE() {
        
        casadi::MXVector E(6);
        
        E[0] = casadi::MX(casadi::DM({
            {1, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
        }));

        E[1] = casadi::MX(casadi::DM({
            {0, 1, 0},
            {1, 0, 0},
            {0, 0, 0}
        }));

        E[2] = casadi::MX(casadi::DM({
            {0, 0, 1},
            {0, 0, 0},
            {1, 0, 0}
        }));

        E[3] = casadi::MX(casadi::DM({
            {0, 0, 0},
            {0, 1, 0},
            {0, 0, 0}
        }));

        E[4] = casadi::MX(casadi::DM({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0}
        }));

        E[5] = casadi::MX(casadi::DM({
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 1}
        }));

        return E;
    }
    */

    // Function to obtain elements to construct C matrix with Christoffel symbols
    casadi::MXVector stdCmatrix(const casadi::MX& B, const casadi::MX& q, const casadi::MX& dq, casadi::MX& dq_sel) {
        
        int n = q.size1();

        //std::cout << "B " << B.size1() << " " << B.size2() << std::endl;
        
        casadi::MX jac_B = jacobian(B,q);
        //std::cout << "jac_B " << jac_B.size1() << " " <<  jac_B.size2() << std::endl;
        
        casadi::MX C123 = reshape(mtimes(jac_B,dq),n,n);
        //std::cout << "C123 " << C123.size1() << " " <<  C123.size2() << std::endl;

        casadi::MX C132 = mtimes(dq_sel,jac_B);
        casadi::MX C231 = transpose(C132);

        casadi::MXVector C(3);
        C[0] = C123;
        C[1] = C132;
        C[2] = C231;

        return C;
    }

    casadi::Function DHRegressor(const Eigen::MatrixXd& DH_table,const std::string& jType, frame& base_frame){
        // create a symbolic vector
        const int nj = DH_table.rows();
        casadi::MX q = casadi::MX::sym("q", nj);
        casadi::MX dq = casadi::MX::sym("dq", nj);
        casadi::MX dqr = casadi::MX::sym("dqr", nj);
        casadi::MX ddqr = casadi::MX::sym("ddqr", nj);
        
        casadi::MXVector Ti(nj);
        casadi::MXVector T0i(nj);
        std::tuple<casadi::MXVector, casadi::MXVector> T_tuple;

        casadi::MXVector Jvi(nj);
        casadi::MXVector Jwi(nj);
        std::tuple<casadi::MXVector, casadi::MXVector> J_tuple;

        casadi::MX T1(4,4);
        T1 = DHTemplate(DH_table.row(0), q(0), jType[0]);
        //std::cout << "T1 " << T1.size() << std::endl;

        T_tuple = DHFwKin(DH_table, q, jType);
        T0i = std::get<0>(T_tuple);
        Ti  = std::get<1>(T_tuple);
        //std::cout << "T0i3 " << T0i[3].size() << std::endl;
        //std::cout << "T0i " << T0i.size() << std::endl;

        J_tuple = DHJac(T0i, jType, base_frame);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);
        //std::cout << "J0i3 " << Jvi[3].size() << std::endl;
        //std::cout << "J0i6 " << Jvi[6].columns() << std::endl;
        //std::cout << "J0i " << Jvi.size() << std::endl;

        casadi::MX g = base_frame.get_gravity();
        //std::cout<<"in DHregressor, g:\n"<<g<<std::endl;

        casadi::MX Yr(nj,10*nj);
        
        casadi::MX dq_sel = dq_select(dq); 
        casadi::MXVector Q = createQ();
        casadi::MXVector E = createE();
        casadi::Slice allRows;          // Select all rows
        casadi::Slice selR(0,3);

        for (int i=0; i<nj; i++) {
            
            casadi::MX R0i = mtimes(base_frame.rotation(),T0i[i](selR,selR));
            //std::cout<<"in DHregressor, base rotation : \n"<<base_frame.rotation()<<std::endl;


            // ------------------------- Y0r_i -------------------------- //
            casadi::MX B0_i = mtimes(Jvi[i].T(),Jvi[i]);
            casadi::MXVector C = stdCmatrix(B0_i,q,dq,dq_sel);

            casadi::MX dX0r_i = mtimes(B0_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
            casadi::MX W0r_i = mtimes(C[2],dqr)/2;
            casadi::MX Z0r_i = -1*mtimes(Jvi[i].T(),g);
            
            casadi::MX Y0r_i = dX0r_i - W0r_i + Z0r_i;
            
            // ------------------------- Y1r_i -------------------------- //
            casadi::MX dX1r_i(nj,3);
            casadi::MX W1r_i(nj,3);

            for (int l=0; l<3; l++) {
                casadi::MX Ql = Q[l];
                //std::cout << "Ql " << Ql << std::endl;
                casadi::MX B1l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jvi[i])))) - 
                                    mtimes(Jvi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jwi[i]))));

                casadi::MXVector C = stdCmatrix(B1l_i,q,dq,dq_sel);

                dX1r_i(allRows,l) = mtimes(B1l_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W1r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }
            casadi::MX Z1r_i= -(jacobian(mtimes(R0i.T(),g),q)).T();
            
            casadi::MX Y1r_i = dX1r_i - W1r_i + Z1r_i;

            // ------------------------- Y2r_i -------------------------- //
            casadi::MX dX2r_i(nj,6);
            casadi::MX W2r_i(nj,6);
            
            for (int l=0; l<6; l++) {
                casadi::MX El = E[l];
                casadi::MX B2l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(El,mtimes(R0i.T(),Jwi[i]))));
                casadi::MXVector C = stdCmatrix(B2l_i,q,dq,dq_sel);
                dX2r_i(allRows,l) = mtimes(B2l_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W2r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }

            casadi::MX Y2r_i = dX2r_i - W2r_i;

            // ------------------------- Yr_i -------------------------- //
            casadi::MX Yr_i = horzcat(Y0r_i,Y1r_i,Y2r_i);

            casadi::Slice selCols(i*10,(i+1)*10);          // Select current columns of matrix regressor
            Yr(allRows,selCols) = Yr_i;
        }
        casadi::Function regressor_fun("regr_fun", {q,dq,dqr,ddqr}, {densify(Yr)});
        return regressor_fun;
    }

    /* ========== CLASSE FRAME DEFINIZIONE ========== */
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
    casadi::MX frame::rotation(){
        casadi::MX R = casadi::MX::eye(3);
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
    casadi::MX frame::transform(){
        casadi::MX rotTr = casadi::MX::eye(4);
        
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
        rotTr(1,1)=sy*sp*sr-cy*cr;
        rotTr(1,2)=sy*sp*cr-cy*sr;
        rotTr(2,0)=-sp;
        rotTr(2,1)=cp*sr;
        rotTr(2,2)=cp*cr;
        rotTr(0,3) = dx;
        rotTr(1,3) = dy;
        rotTr(2,3) = dz;

        return rotTr;
    }
    void frame::set_gravity(const vec3d& g_){gravity = g_;}
    void frame::set_position(const vec3d& pos_){position = pos_;}
    void frame::set_ypr(const vec3d& ypr_){ypr = ypr_;}
    casadi::MX frame::hat(const vec3d& v){
        if(v.size() != 3 ){
            std::cout<<"in function hat of class frame invalid dimension of input"<<std::endl;
        }
        casadi::MX vhat(3,3);
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
    casadi::MX frame::get_gravity(){
        casadi::MX MXg(3,1);
        MXg(0,0) = gravity[0];
        MXg(1,0) = gravity[1];
        MXg(2,0) = gravity[2];
        return MXg;
        //return {{gravity[0]},{gravity[1]},{gravity[2]}};
    }
    casadi::MX frame::get_position(){
        casadi::MX pos(3,1);
        pos(0,0) = position[0];
        pos(1,0) = position[1];
        pos(2,0) = position[2];
        return pos;
        //return {{position[0]},{position[1]},{position[2]}};
    }
    vec3d frame::get_ypr(){return ypr;}
}
