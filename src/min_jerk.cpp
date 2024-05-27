#include <cmath>
#include <eigen3/Eigen/Dense>
#include <utils/min_jerk.h>

min_jerk_class::min_jerk_class() = default;

min_jerk_class::~min_jerk_class() = default;

Eigen::MatrixXd min_jerk_class::get_ddq(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i,
											Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f,
											double tf, double t){
	Eigen::MatrixXd b_out1_tmp;
	Eigen::MatrixXd out1_tmp;
	double t2;
	Eigen::MatrixXd t3;
	Eigen::MatrixXd t4;
	Eigen::MatrixXd t4_tmp;
	t2 = tf * tf;
	t3 = ddq_f * t2;
	t4_tmp = ddq_i * t2;
	t4 = t4_tmp * 3.0;
	out1_tmp = dq_f * tf;
	b_out1_tmp = dq_i * tf;
	return ((ddq_i - t / pow(tf, 3.0) * (((((q_f * -20.0 + q_i * 20.0) + t4) - t3) + out1_tmp * 8.0) + b_out1_tmp * 12.0) * 3.0) - pow(t, 3.0) / pow(tf, 5.0) * (((((q_f * -12.0 + q_i * 12.0) - t3) + t4_tmp) + out1_tmp * 6.0) + b_out1_tmp * 6.0) * 10.0) + t * t / (t2 * t2) * (((((q_f * -30.0 + q_i * 30.0) - t3 * 2.0) + t4) + out1_tmp * 14.0) + b_out1_tmp * 16.0) * 6.0;
}

Eigen::MatrixXd min_jerk_class::get_dq(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i,
											Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f,
											double tf, double t){
	Eigen::MatrixXd b_out1_tmp;
	Eigen::MatrixXd out1_tmp;
	double t2;
	Eigen::MatrixXd t3;
	Eigen::MatrixXd t4;
	Eigen::MatrixXd t4_tmp;
	t2 = tf * tf;
	t3 = ddq_f * t2;
	t4_tmp = ddq_i * t2;
	t4 = t4_tmp * 3.0;
	out1_tmp = dq_f * tf;
	b_out1_tmp = dq_i * tf;
	return (((dq_i + ddq_i * t) - pow(t, 4.0) / pow(tf, 5.0) * (((((q_f * -12.0 + q_i * 12.0) - t3) + t4_tmp) + out1_tmp * 6.0) + b_out1_tmp * 6.0) * 2.5) - t * t / pow(tf, 3.0) * (((((q_f * -20.0 + q_i * 20.0) + t4) - t3) + out1_tmp * 8.0) + b_out1_tmp * 12.0) * 1.5) + pow(t, 3.0) / (t2 * t2) * (((((q_f * -30.0 + q_i * 30.0) - t3 * 2.0) + t4) + out1_tmp * 14.0) + b_out1_tmp * 16.0) * 2.0;
}

Eigen::MatrixXd min_jerk_class::get_q(Eigen::MatrixXd q_i, Eigen::MatrixXd dq_i, Eigen::MatrixXd ddq_i,
											Eigen::MatrixXd q_f, Eigen::MatrixXd dq_f, Eigen::MatrixXd ddq_f,
											double tf, double t){
	Eigen::MatrixXd b_out1_tmp;
	Eigen::MatrixXd out1_tmp;
	double t2;
	Eigen::MatrixXd t3;
	Eigen::MatrixXd t4;
	Eigen::MatrixXd t4_tmp;
	t2 = tf * tf;
	t3 = ddq_f * t2;
	t4_tmp = ddq_i * t2;
	t4 = t4_tmp * 3.0;
	out1_tmp = dq_f * tf;
	b_out1_tmp = dq_i * tf;
	return ((((q_i + dq_i * t) + ddq_i * (t * t) / 2.0) - pow(t, 5.0) / pow(tf, 5.0) * (((((q_f * -12.0 + q_i * 12.0) - t3) + t4_tmp) + out1_tmp * 6.0) + b_out1_tmp * 6.0) / 2.0) - pow(t, 3.0) / pow(tf, 3.0) * (((((q_f * -20.0 + q_i * 20.0) + t4) - t3) + out1_tmp * 8.0) + b_out1_tmp * 12.0) / 2.0) + pow(t, 4.0) / (t2 * t2) * (((((q_f * -30.0 + q_i * 30.0) - t3 * 2.0) + t4) + out1_tmp * 14.0) + b_out1_tmp * 16.0) / 2.0;
}