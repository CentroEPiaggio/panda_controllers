#include "utils/RegBasic.h"

namespace regrob{

    RegBasic::RegBasic(){};
    RegBasic::RegBasic(const int numJoints_):numJoints(numJoints_){
        q = casadi::SX::sym("q", numJoints,1);
        dq = casadi::SX::sym("dq", numJoints,1);
        dqr = casadi::SX::sym("dqr", numJoints,1);
        ddqr = casadi::SX::sym("ddqr", numJoints,1);
        E = createE();
        Q = createQ();
        dq_sel = dq_select();
    }
    void RegBasic::basic_init(const int numJoints_){
        numJoints = numJoints_;
        q = casadi::SX::sym("q", numJoints,1);
        dq = casadi::SX::sym("dq", numJoints,1);
        dqr = casadi::SX::sym("dqr", numJoints,1);
        ddqr = casadi::SX::sym("ddqr", numJoints,1);
        E = createE();
        Q = createQ();
        dq_sel = dq_select(); 
    }
    // Function to generate matrix template of transformation from Denavit-Hartenberg table row
    casadi::SX RegBasic::DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, char jtsType) {
        
        double a;           // DH table format [a alpha d theta]
        double alpha;
        casadi::SX d;
        casadi::SX theta;
        casadi::SX ct;      // cos(theta)
        casadi::SX st;      // sin(theta)
        double ca;          // cos(alpha)
        double sa;          // sin(alpha)
        double c3;
        double s3;
        
        casadi::SX Ti(4,4); // output
        
        if (rowDHTable.cols() != 4) {
            throw std::runtime_error("DHTemplate: Error size of DH table");
        }

        if (jtsType == 'P') {
            d = rowDHTable(2) + qi;
            theta = rowDHTable(3);
            //ct = cos(theta);
            //st = sin(theta);
        }
        else if (jtsType == 'R') {
            d = rowDHTable(2);
            theta = rowDHTable(3) + qi;

            //c3 = mapToZero(cos(rowDHTable(3)));
            //s3 = mapToZero(sin(rowDHTable(3)));
            //ct = c3*cos(qi)-s3*sin(qi);
            //st = s3*cos(qi)+c3*sin(qi);
        }
        else {
            throw std::runtime_error("DHTemplate: Error joint type");
        }

        a = rowDHTable(0);
        alpha = rowDHTable(1);
        
        ca = cos(alpha);
        sa = sin(alpha);
        ct = cos(theta);
        st = sin(theta);
        
        //ca = mapToZero(ca);
        //sa = mapToZero(sa);
        
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
    std::tuple<casadi::SXVector,casadi::SXVector> RegBasic::DHFwKin(const Eigen::MatrixXd& DHTable, const std::string& jtsType) {
        
        /* Variables */
        int nj = q.size1();         // Number of joints

        casadi::SXVector Ti(nj);    // Output
        casadi::SXVector T0i(nj);   // Output

        // Check of dimensions
        if ((int)q.size1() != (int)jtsType.size()) {
            throw std::runtime_error("DHFwkin: Error size of joints string");
        }
        if (q.size1() != DHTable.rows()) {
            throw std::runtime_error("DHFwkin: Error size of DH table");
        }    
        
        // Creazione di T0i
        Ti[0] = RegBasic::DHTemplate(DHTable.row(0), q(0), jtsType[0]);
        T0i[0] = Ti[0];

        for (int i = 1; i < nj; i++) {
            Ti[i] = RegBasic::DHTemplate(DHTable.row(i), q(i), jtsType[i]);
            T0i[i] = mtimes(T0i[i - 1],Ti[i]);
        }

        return std::make_tuple(T0i, Ti);
    }

    // Function hat
    casadi::SX RegBasic::hat(const casadi::SX& v) {
        
        casadi::SX skew(3, 3);
        
        skew(0, 1) = -v(2);
        skew(0, 2) = v(1);
        skew(1, 0) = v(2);
        skew(1, 2) = -v(0);
        skew(2, 0) = -v(1);
        skew(2, 1) = v(0);
        
        return skew;
    }

    // Function for Jacobian of link i-th from Denavit-Hartenberg parameterization
    std::tuple<casadi::SXVector,casadi::SXVector> RegBasic::DHJac(const casadi::SXVector& T0i_vec, const std::string& jtsType, frame& base_frame) {
        
        // Number of joints
        int nj = T0i_vec.size();

        // Variables 
        casadi::SX Ji_pos(3, nj);       // matrix of velocity jacobian
        casadi::SX Ji_or(3, nj);        // matrix of omega jacobian
        casadi::SXVector Ji_v(nj);      // vector of matrix Ji_v
        casadi::SXVector Ji_w(nj);      // vector of matrix Ji_w
        casadi::Slice r_tra_idx(0, 3);  // select translation vector of T()
        casadi::Slice r_rot_idx(0, 3);  // select k versor of T()
        casadi::Slice allRows;          // Select all rows

        // Check
        if (nj != (int)jtsType.size()) {
            throw std::runtime_error("DHJac: Error joints' string size");
        }

        for (int i = 0; i < nj; i++) {

            // Init variables of first column jacobian
            casadi::SX k0(3,1);             // versor of joint i
            casadi::SX O_0i(3,1);           // distance of joint i from joint 0
            casadi::SX T_0i(4,4);           // matrix tranformation of joint 1 from joint 0

            // First column of jacobian
            k0(2,0) = 1;
            T_0i = T0i_vec[i];
            O_0i = T_0i(r_tra_idx, 3);

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
                casadi::SX kj_1(3,1);             // versor of joint j-1
                casadi::SX O_j_1i(3,1);           // distance of joint i from joint j-1
                casadi::SX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1
        
                T_0j_1 = T0i_vec[j-1];
                kj_1 = T_0j_1(r_rot_idx, 2);
                O_j_1i = O_0i - T_0j_1(r_tra_idx, 3);

                if (jtsType[j] == 'P') {
                    Ji_pos(allRows, j) = kj_1;
                } else if (jtsType[j] == 'R') {
                    Ji_pos(allRows, j) = mtimes(hat(kj_1),O_j_1i);
                    Ji_or(allRows, j) = kj_1;
                } else {
                    throw std::runtime_error("DHJac: Error joint type");
                }
            }

            Ji_pos = mtimes(base_frame.get_rotation(),Ji_pos);
            Ji_or = mtimes(base_frame.get_rotation(),Ji_or);
            Ji_v[i] = Ji_pos;
            Ji_w[i] = Ji_or;
        }

        return std::make_tuple(Ji_v, Ji_w);
    }

    // Function to create dq_selection matrix
    casadi::SX RegBasic::dq_select() {
        
        //int n = dq.size1();
        int n = numJoints;
        std::cout<<"in dq_select(): numJoints = "<<numJoints<<std::endl;
        casadi::Slice allRows;          // Select all rows
        
        casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
        for (int i = 0; i < n; i++) {
            casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
            mat_dq(allRows, sel_col) = casadi::SX::eye(n)*dq(i);
        }
        
        return mat_dq;
    }
    
    // Function to create Q
    casadi::SXVector RegBasic::createQ() {

        casadi::SXVector Q_(3);

        for(int i=0;i<3;i++){
            Q_[i] = casadi::SX::zeros(3,3);
        }
        
        Q_[0](1,2) = -1;
        Q_[0](2,1) = 1;

        Q_[1](0,2) = 1;
        Q_[1](2,0) = -1;

        Q_[2](0,1) = -1;
        Q_[2](1,0) = 1;

        return Q_;
    }

    // Function to create E
    casadi::SXVector RegBasic::createE() {
        
        casadi::SXVector E_(6);

        for(int i=0;i<6;i++){
            E_[i] = casadi::SX::zeros(3,3);
        }
        E_[0](0,0) = 1;

        E_[1](0,1) = 1;
        E_[1](1,0) = 1;

        E_[2](0,2) = 1;
        E_[2](2,0) = 1;

        E_[3](1,1) = 1;

        E_[4](1,2) = 1;
        E_[4](2,1) = 1;

        E_[5](2,2) = 1;

        return E_;
    }
    
    // Function to obtain elements to construct C matrix with Christoffel symbols
    casadi::SXVector RegBasic::stdCmatrix(const casadi::SX& B) {
        
        int n = q.size1();

        casadi::SX jac_B = jacobian(B,q);
        
        casadi::SX C123 = reshape(mtimes(jac_B,dq),n,n);
        casadi::SX C132 = mtimes(dq_sel,jac_B);
        casadi::SX C231 = C123.T();

        casadi::SXVector C(3);
        C[0] = C123;
        C[1] = C132;
        C[2] = C231;

        return C;
    }
/* 
    casadi::Function RegBasic::DHRegressor(const Eigen::MatrixXd& DH_table,const std::string& jType, frame& base_frame){
        // create a symbolic vectors
        const int nj = DH_table.rows();
        
        casadi::SXVector Ti(nj);
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jvi(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        T_tuple = RegBasic::DHFwKin(DH_table, jType);
        T0i = std::get<0>(T_tuple);
        //Ti  = std::get<1>(T_tuple);

        J_tuple = RegBasic::DHJac(T0i, jType, base_frame);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);

        casadi::SX g = base_frame.get_gravity();

        casadi::SX Yr(nj,10*nj);        // regressor matrix
        
        //casadi::SX dq_sel = dq_select(dq);
        //casadi::SXVector Q = createQ();
        //casadi::SXVector E = createE();
        casadi::Slice allRows;          // Select all rows
        casadi::Slice allCols(0,nj);          
        casadi::Slice selR(0,3);

        for (int i=0; i<nj; i++) {
            
            casadi::SX R0i = casadi::SX::mtimes({base_frame.get_rotation(),T0i[i](selR,selR),base_frame.get_rotation().T()});

            // ------------------------- Y0r_i -------------------------- //
            casadi::SX B0_i = mtimes(Jvi[i].T(),Jvi[i]);
            casadi::SXVector C = RegBasic::stdCmatrix(B0_i);

            casadi::SX dX0r_i = mtimes(B0_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
            casadi::SX W0r_i = mtimes(C[2],dqr)/2;
            casadi::SX Z0r_i = -mtimes(Jvi[i].T(),g);
            
            casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
            
            // ------------------------- Y1r_i -------------------------- //
            casadi::SX dX1r_i(nj,3);
            casadi::SX W1r_i(nj,3);

            casadi::SXVector B1l_i(3);
            casadi::SX matJwR = mtimes(R0i.T(),Jwi[i]);
            casadi::SX matJvR = mtimes(R0i.T(),Jvi[i]);

            casadi::SX tmpMat0 = mtimes(matJwR(2,allCols).T(),matJvR(1,allCols)) - mtimes(matJwR(1,allCols).T(),matJvR(2,allCols));
            casadi::SX tmpMat1 = mtimes(matJwR(0,allCols).T(),matJvR(2,allCols)) - mtimes(matJwR(2,allCols).T(),matJvR(0,allCols));
            casadi::SX tmpMat2 = mtimes(matJwR(1,allCols).T(),matJvR(0,allCols)) - mtimes(matJwR(0,allCols).T(),matJvR(1,allCols));
                      
            B1l_i[0] = tmpMat0 + tmpMat0.T();
            B1l_i[1] = tmpMat1 + tmpMat1.T();
            B1l_i[2] = tmpMat2 + tmpMat2.T();
        
            for (int l=0; l<3; l++) {
                //casadi::SX Ql = Q[l];
                //casadi::SX B1l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jvi[i])))) - 
                //                    mtimes(Jvi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jwi[i]))));
                //casadi::SX B1l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,Ql,R0i.T(),Jvi[i]}) - 
                //                   casadi::SX::mtimes({Jvi[i].T(),R0i,Ql,R0i.T(),Jwi[i]});

                casadi::SXVector C = RegBasic::stdCmatrix(B1l_i[l]);

                dX1r_i(allRows,l) = mtimes(B1l_i[l],ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W1r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }
            casadi::SX Z1r_i= -(jacobian(mtimes(R0i.T(),g),q)).T();
            
            casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

            // ------------------------- Y2r_i -------------------------- //
            casadi::SX dX2r_i(nj,6);
            casadi::SX W2r_i(nj,6);
            
            for (int l=0; l<6; l++) {
                casadi::SX El = E[l];
                //casadi::SX B2l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(El,mtimes(R0i.T(),Jwi[i]))));
                casadi::SX B2l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,El,R0i.T(),Jwi[i]});
                casadi::SXVector C = RegBasic::stdCmatrix(B2l_i);
                dX2r_i(allRows,l) = mtimes(B2l_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W2r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }

            casadi::SX Y2r_i = dX2r_i - W2r_i;

            // ------------------------- Yr_i -------------------------- //
            casadi::SX Yr_i = horzcat(Y0r_i,Y1r_i,Y2r_i);

            casadi::Slice selCols(i*10,(i+1)*10);          // Select current columns of matrix regressor
            Yr(allRows,selCols) = Yr_i;
        }

        //return Yr;
        casadi::Function regressor_fun("regr_fun", {q,dq,dqr,ddqr}, {densify(Yr)});
        return regressor_fun;
    }
 */
    casadi::SX RegBasic::SXregressor(const Eigen::MatrixXd& DH_table,const std::string& jType, frame& base_frame){
        // create a symbolic vectors
        const int nj = DH_table.rows();
        
        casadi::SXVector Ti(nj);
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jvi(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        T_tuple = RegBasic::DHFwKin(DH_table, jType);
        T0i = std::get<0>(T_tuple);
        //Ti  = std::get<1>(T_tuple);

        J_tuple = RegBasic::DHJac(T0i, jType, base_frame);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);

        casadi::SX g = base_frame.get_gravity();

        casadi::SX Yr(nj,10*nj);        // regressor matrix
        
        //casadi::SX dq_sel = dq_select(dq);
        //casadi::SXVector Q = createQ();
        //casadi::SXVector E = createE();
        casadi::Slice allRows;          // Select all rows
        casadi::Slice allCols(0,nj);          
        casadi::Slice selR(0,3);

        for (int i=0; i<nj; i++) {
            
            casadi::SX R0i = casadi::SX::mtimes({base_frame.get_rotation(),T0i[i](selR,selR),base_frame.get_rotation().T()});

            // ------------------------- Y0r_i -------------------------- //
            casadi::SX B0_i = mtimes(Jvi[i].T(),Jvi[i]);
            casadi::SXVector C = RegBasic::stdCmatrix(B0_i);

            casadi::SX dX0r_i = mtimes(B0_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
            casadi::SX W0r_i = mtimes(C[2],dqr)/2;
            casadi::SX Z0r_i = -mtimes(Jvi[i].T(),g);
            
            casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
            
            // ------------------------- Y1r_i -------------------------- //
            casadi::SX dX1r_i(nj,3);
            casadi::SX W1r_i(nj,3);

            //casadi::SXVector B1l_i(3);
            casadi::SX matJwR = mtimes(R0i.T(),Jwi[i]);
            casadi::SX matJvR = mtimes(R0i.T(),Jvi[i]);

            //casadi::SX tmpMat0 = mtimes(matJwR(2,allCols).T(),matJvR(1,allCols)) - mtimes(matJwR(1,allCols).T(),matJvR(2,allCols));
            //casadi::SX tmpMat1 = mtimes(matJwR(0,allCols).T(),matJvR(2,allCols)) - mtimes(matJwR(2,allCols).T(),matJvR(0,allCols));
            //casadi::SX tmpMat2 = mtimes(matJwR(1,allCols).T(),matJvR(0,allCols)) - mtimes(matJwR(0,allCols).T(),matJvR(1,allCols));
                      
            //B1l_i[0] = tmpMat0 + tmpMat0.T();
            //B1l_i[1] = tmpMat1 + tmpMat1.T();
            //B1l_i[2] = tmpMat2 + tmpMat2.T();
        
            for (int l=0; l<3; l++) {
                casadi::SX Ql = Q[l];
                //casadi::SX B1l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jvi[i])))) - 
                //                    mtimes(Jvi[i].T(),mtimes(R0i,mtimes(Ql,mtimes(R0i.T(),Jwi[i]))));
                casadi::SX B1l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,Ql,R0i.T(),Jvi[i]}) - 
                                   casadi::SX::mtimes({Jvi[i].T(),R0i,Ql,R0i.T(),Jwi[i]});

                casadi::SXVector C = RegBasic::stdCmatrix(B1l_i);

                dX1r_i(allRows,l) = mtimes(B1l_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W1r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }
            casadi::SX Z1r_i= -(jacobian(mtimes(R0i.T(),g),q)).T();
            
            casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

            // ------------------------- Y2r_i -------------------------- //
            casadi::SX dX2r_i(nj,6);
            casadi::SX W2r_i(nj,6);
            
            for (int l=0; l<6; l++) {
                casadi::SX El = E[l];
                //casadi::SX B2l_i = mtimes(Jwi[i].T(),mtimes(R0i,mtimes(El,mtimes(R0i.T(),Jwi[i]))));
                casadi::SX B2l_i = casadi::SX::mtimes({Jwi[i].T(),R0i,El,R0i.T(),Jwi[i]});
                casadi::SXVector C = RegBasic::stdCmatrix(B2l_i);
                dX2r_i(allRows,l) = mtimes(B2l_i,ddqr) + mtimes((C[0]+C[1]),dqr)/2;
                W2r_i(allRows,l) = mtimes(C[2],dqr)/2;
            }

            casadi::SX Y2r_i = dX2r_i - W2r_i;

            // ------------------------- Yr_i -------------------------- //
            casadi::SX Yr_i = horzcat(Y0r_i,Y1r_i,Y2r_i);

            casadi::Slice selCols(i*10,(i+1)*10);          // Select current columns of matrix regressor
            Yr(allRows,selCols) = Yr_i;
        }

        return Yr;
        //casadi::Function regressor_fun("regr_fun", {q,dq,dqr,ddqr}, {densify(Yr)});
        //return regressor_fun;
    }

    casadi::Function RegBasic::DHReg_fun(const casadi::SX& SX_Yr){

        casadi::Function regressor_fun("regr_fun", {q,dq,dqr,ddqr}, {densify(SX_Yr)});
        return regressor_fun;
    }
    
    // Function for Jacobian of link n-th from Denavit-Hartenberg parameterization
    casadi::Function RegBasic::DHJac_fun(const Eigen::MatrixXd& DH_table, const std::string& jtsType, frame& base_frame,const double mu_) {
        // create a symbolic vectors
        const int nj = DH_table.rows();
        
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

        casadi::SXVector Jvi(nj);
        casadi::SXVector Jwi(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

        casadi::SX Ji_pos(3,nj);
        casadi::SX Ji_or(3,nj);
        casadi::SX Ji(6,nj);
        casadi::SX invJn_dumped(nj,6);
        casadi::SX pinvJn(nj,6);
        casadi::SX dot_pinvJn(nj,6);

        T_tuple = RegBasic::DHFwKin(DH_table, jtsType);
        T0i = std::get<0>(T_tuple);

        J_tuple = RegBasic::DHJac(T0i, jtsType, base_frame);
        Jvi = std::get<0>(J_tuple);
        Jwi = std::get<1>(J_tuple);

        Ji_pos = Jvi[nj-1];
        Ji_or = Jwi[nj-1];
        Ji = casadi::SX::vertcat({Ji_pos,Ji_or});

        /* ============================================== */
        /* ==   PSEUDOINVERSION AND TIME DERIVATION    == */
        /* ============================================== */

        //pinvJn = casadi::SX::pinv(Ji); // PSEUDOINVERSION WITH CASADI 
        
        invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Ji,Ji.T()}) + casadi::SX::eye(6)*mu_);
        //std::cout<<"\nin DHJacobian: invJn_dumped size\n"<<invJn_dumped.size()<<std::endl;
        //std::cout<<"\nin DHJacobian: invJn_dumped\n"<<invJn_dumped<<std::endl;

        //std::cout<<"\nin DHJacobian: check pinv 0\n";

        pinvJn = casadi::SX::mtimes({Ji.T(),invJn_dumped});
        //std::cout<<"\nin DHJacobian: pinvJn size\n"<<pinvJn.size()<<std::endl;
        //std::cout<<"\nin DHJacobian: pinvJn\n"<<pinvJn<<std::endl;

        dot_pinvJn = casadi::SX::jtimes(pinvJn,q,dq);        
        //std::cout<<"\nin DHJacobian: dot_pinvJn size\n"<<dot_pinvJn.size()<<std::endl;
        //std::cout<<"\nin DHJacobian: dot_pinvJn\n"<<dot_pinvJn<<std::endl;

        //std::cout<<"\nin DHJacobian: check 2\n";

        casadi::Function Jac_fun("Jac_fun",{q,dq},{densify(Ji),densify(pinvJn),densify(dot_pinvJn)});
        //std::cout<<"Funzione del Jacobiano: "<<Jac_fun<<std::endl;
        return Jac_fun;
    }
    casadi::Function RegBasic::DHKin_fun(const Eigen::MatrixXd& DH_table, const std::string& jtsType, frame& base_frame) {
        // create a symbolic vectors
        const int nj = DH_table.rows();
        
        casadi::SXVector T0i(nj);
        std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;
        casadi::SX T0EE = casadi::SX::eye(4);

        T_tuple = RegBasic::DHFwKin(DH_table, jtsType);
        T0i = std::get<0>(T_tuple);

        
        T0EE = mtimes(base_frame.get_transform(),T0i[nj-1]);

        casadi::Function Kin_fun("Kin_fun",{q},{densify(T0EE)});
        //std::cout<<"Funzione del Jacobiano: "<<Jac_fun<<std::endl;
        return Kin_fun;
    }

}