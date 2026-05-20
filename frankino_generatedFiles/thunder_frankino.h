#ifndef THUNDER_frankino_H
#define THUNDER_frankino_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* Class thunder_frankino 
Useful functions for adaptive control with Slotine-Li regressor
Usable only for generated code from casadi library! */
class thunder_frankino{

	private:
		// standard number of parameters
		const int STD_PAR_LINK = 10;
		/* Joints' variables */
		Eigen::VectorXd q, dq, ddq, d3q, d4q, dqr, ddqr, par_DYN, par_REG, par_Dl;
		Eigen::VectorXd x, dx, ddx, ddxr, par_K, par_D, par_Dm, par_Mm;
		Eigen::VectorXd w;
		Eigen::VectorXd par_DHtable, par_gravity, par_world2L0, par_Ln2EE;
		std::vector<int> DHtable_symb, gravity_symb, world2L0_symb, Ln2EE_symb;

		void update_inertial_DYN();
		void update_inertial_REG();

		struct LinkProp {
			double mass;
			std::vector<double> parI = std::vector<double>(6); // Inertia in the order xx, xy, xz, yy, yz, zz
			std::vector<double> xyz = std::vector<double>(3); // Origin xyz as std::vector
			std::vector<double> rpy = std::vector<double>(3);
			std::vector<double> Dl = std::vector<double>(1);
			std::string name;
		};

		struct urdf2dh_T{
			std::vector<double> xyz;
			std::vector<double> rpy;
		};

		void fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_);
		// void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body);
		// void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody);

		Eigen::Matrix3d rpyRot(const std::vector<double> rpy);
		Eigen::Matrix3d createI(const std::vector<double> parI);
		Eigen::Matrix3d hat(const Eigen::Vector3d v);
		
	public:
		const std::string frankinoName = "frankino";
		const int n_joints = 7;
		const bool ELASTIC = 0;
		const int numElasticJoints = 0;
		const int K_order = 0;
		const int D_order = 0;
		const int Dl_order = 0;
		const int Dm_order = 0;
		const int numParDYN = STD_PAR_LINK*n_joints;
		const int numParREG = STD_PAR_LINK*n_joints;
		// const int numParELA = /*#-numParELA-#*/;
		const int isElasticJoint[7] = {0, 0, 0, 0, 0, 0, 0};

		/* Empty constructor, initialization inside */
		thunder_frankino();
		/* Constructor to init variables*/
		// thunder_frankino(const int);
		/* Destructor*/
		~thunder_frankino() = default;
		
		/* Resize variables */
		void resizeVariables();
		/* Init variables from number of joints*/
		// void init(const int);
		/* Set q, dq, dqr, ddqr, to compute Regressor and update state*/
		void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_);
		void set_q(const Eigen::VectorXd& q_);
		void set_dq(const Eigen::VectorXd& dq_);
		void set_ddq(const Eigen::VectorXd& ddq_);
		void set_d3q(const Eigen::VectorXd& d3q_);
		void set_d4q(const Eigen::VectorXd& d4q_);
		void set_dqr(const Eigen::VectorXd& dqr_);
		void set_ddqr(const Eigen::VectorXd& ddqr_);
		void set_x(const Eigen::VectorXd& x_);
		void set_dx(const Eigen::VectorXd& dx_);
		void set_ddx(const Eigen::VectorXd& ddx_);
		void set_ddxr(const Eigen::VectorXd& ddxr_);
		void set_w(const Eigen::VectorXd& w_);
		void set_par_REG(const Eigen::VectorXd& par_, bool update_DYN = true);
		void set_par_DYN(const Eigen::VectorXd& par_, bool update_REG = true);
		void set_par_K(const Eigen::VectorXd& par_);
		void set_par_D(const Eigen::VectorXd& par_);
		void set_par_Dm(const Eigen::VectorXd& par_);
		void set_par_Mm(const Eigen::VectorXd& par_);
		void set_par_Dl(const Eigen::VectorXd& par_);
		void set_par_DHtable(const Eigen::MatrixXd& par_);
		void set_par_gravity(const Eigen::VectorXd& par_);
		void set_par_world2L0(const Eigen::VectorXd& par_);
		void set_par_Ln2EE(const Eigen::VectorXd& par_);
		// void set_par_ELA(const Eigen::VectorXd& par_);
		Eigen::VectorXd get_par_REG();
		Eigen::VectorXd get_par_DYN();
		Eigen::VectorXd get_par_K();
		Eigen::VectorXd get_par_D();
		Eigen::VectorXd get_par_Dm();
		Eigen::VectorXd get_par_Mm();
		Eigen::VectorXd get_par_Dl();
		Eigen::MatrixXd get_par_DHtable();
		Eigen::VectorXd get_par_gravity();
		Eigen::VectorXd get_par_world2L0();
		Eigen::VectorXd get_par_Ln2EE();
		// Eigen::VectorXd get_par_ELA();

		Eigen::VectorXd load_par_REG(std::string, bool update_DYN = true);
		void save_par_REG(std::string);
		void load_conf(std::string, bool update_REG = true);
		void save_par_DYN(std::string);
		int save_par(std::string par_file);
		// void load_par_DYN(std::string);
		// void save_par_DYN(std::string);
		// void load_par_elastic(std::string);
		// void load_par_ELA(std::string);
		// void save_par_ELA(std::string);

		int get_numJoints();
		// int get_numParLink();
		int get_numParDYN();
		int get_numParREG();
		// int get_numParELA();

		// ----- generated functions ----- //
		
		// - Manipulator Coriolis matrix - //
		Eigen::Matrix<double,7,7> get_C();

		// - Second time derivative of the Coriolis matrix - //
		Eigen::Matrix<double,7,7> get_C_ddot();

		// - Time derivative of the Coriolis matrix - //
		Eigen::Matrix<double,7,7> get_C_dot();

		// - Classic formulation of the manipulator Coriolis matrix - //
		Eigen::Matrix<double,7,7> get_C_std();

		// - Manipulator gravity terms - //
		Eigen::Matrix<double,7,1> get_G();

		// - Second time derivative of the gravity vector - //
		Eigen::Matrix<double,7,1> get_G_ddot();

		// - Time derivative of the gravity vector - //
		Eigen::Matrix<double,7,1> get_G_dot();

		// - Jacobian of frame 1 - //
		Eigen::Matrix<double,6,7> get_J_1();

		// - Jacobian of frame 2 - //
		Eigen::Matrix<double,6,7> get_J_2();

		// - Jacobian of frame 3 - //
		Eigen::Matrix<double,6,7> get_J_3();

		// - Jacobian of frame 4 - //
		Eigen::Matrix<double,6,7> get_J_4();

		// - Jacobian of frame 5 - //
		Eigen::Matrix<double,6,7> get_J_5();

		// - Jacobian of frame 6 - //
		Eigen::Matrix<double,6,7> get_J_6();

		// - Jacobian of frame 7 - //
		Eigen::Matrix<double,6,7> get_J_7();

		// - Jacobian of frame 8 - //
		Eigen::Matrix<double,6,7> get_J_8();

		// - Jacobian of center of mass of link 1 - //
		Eigen::Matrix<double,6,7> get_J_cm_1();

		// - Jacobian of center of mass of link 2 - //
		Eigen::Matrix<double,6,7> get_J_cm_2();

		// - Jacobian of center of mass of link 3 - //
		Eigen::Matrix<double,6,7> get_J_cm_3();

		// - Jacobian of center of mass of link 4 - //
		Eigen::Matrix<double,6,7> get_J_cm_4();

		// - Jacobian of center of mass of link 5 - //
		Eigen::Matrix<double,6,7> get_J_cm_5();

		// - Jacobian of center of mass of link 6 - //
		Eigen::Matrix<double,6,7> get_J_cm_6();

		// - Jacobian of center of mass of link 7 - //
		Eigen::Matrix<double,6,7> get_J_cm_7();

		// - Jacobian of the end-effector - //
		Eigen::Matrix<double,6,7> get_J_ee();

		// - Time second derivative of jacobian matrix - //
		Eigen::Matrix<double,6,7> get_J_ee_ddot();

		// - Time derivative of jacobian matrix - //
		Eigen::Matrix<double,6,7> get_J_ee_dot();

		// - Pseudo-Inverse of jacobian matrix - //
		Eigen::Matrix<double,7,6> get_J_ee_pinv();

		// - Manipulator mass matrix - //
		Eigen::Matrix<double,7,7> get_M();

		// - Second time derivative of the mass matrix - //
		Eigen::Matrix<double,7,7> get_M_ddot();

		// - Time derivative of the mass matrix - //
		Eigen::Matrix<double,7,7> get_M_dot();

		// - relative transformation from frame base to frame 1 - //
		Eigen::Matrix<double,4,4> get_T_0();

		// - absolute transformation from frame base to frame 1 - //
		Eigen::Matrix<double,4,4> get_T_0_0();

		// - absolute transformation from frame base to frame 1 - //
		Eigen::Matrix<double,4,4> get_T_0_1();

		// - absolute transformation from frame base to frame 2 - //
		Eigen::Matrix<double,4,4> get_T_0_2();

		// - absolute transformation from frame base to frame 3 - //
		Eigen::Matrix<double,4,4> get_T_0_3();

		// - absolute transformation from frame base to frame 4 - //
		Eigen::Matrix<double,4,4> get_T_0_4();

		// - absolute transformation from frame base to frame 5 - //
		Eigen::Matrix<double,4,4> get_T_0_5();

		// - absolute transformation from frame base to frame 6 - //
		Eigen::Matrix<double,4,4> get_T_0_6();

		// - absolute transformation from frame base to frame 7 - //
		Eigen::Matrix<double,4,4> get_T_0_7();

		// - absolute transformation from frame base to end_effector - //
		Eigen::Matrix<double,4,4> get_T_0_8();

		// - absolute transformation from frame 0 to end_effector - //
		Eigen::Matrix<double,4,4> get_T_0_ee();

		// - relative transformation from frame0to frame 1 - //
		Eigen::Matrix<double,4,4> get_T_1();

		// - relative transformation from frame1to frame 2 - //
		Eigen::Matrix<double,4,4> get_T_2();

		// - relative transformation from frame2to frame 3 - //
		Eigen::Matrix<double,4,4> get_T_3();

		// - relative transformation from frame3to frame 4 - //
		Eigen::Matrix<double,4,4> get_T_4();

		// - relative transformation from frame4to frame 5 - //
		Eigen::Matrix<double,4,4> get_T_5();

		// - relative transformation from frame5to frame 6 - //
		Eigen::Matrix<double,4,4> get_T_6();

		// - relative transformation from frame6to frame 7 - //
		Eigen::Matrix<double,4,4> get_T_7();

		// - Manipulator regressor matrix - //
		Eigen::Matrix<double,7,70> get_Yr();

		// - Regressor matrix of term C*dqr - //
		Eigen::Matrix<double,7,70> get_reg_C();

		// - Regressor matrix of term G - //
		Eigen::Matrix<double,7,70> get_reg_G();

		// - Regressor matrix of term M*ddqr - //
		Eigen::Matrix<double,7,70> get_reg_M();


};

#endif