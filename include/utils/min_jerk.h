#ifndef MIN_JERK_CLASS_H
#define MIN_JERK_CLASS_H

#include <eigen3/Eigen/Dense>

class min_jerk_class {
	// private:

	public:
		min_jerk_class();
		~min_jerk_class();
		Eigen::MatrixXd get_ddq(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i, 
									Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f, 
									double tf, double t);
		Eigen::MatrixXd get_dq(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i, 
									Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f, 
									double tf, double t);
		Eigen::MatrixXd get_q(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i, 
									Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f, 
									double tf, double t);
};

#endif