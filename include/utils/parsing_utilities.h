#ifndef PARSING_UTILS_H
#define PARSING_UTILS_H

#include "XmlRpcValue.h"
#include "XmlRpcException.h"
#include <Eigen/Dense>
#include "ros/ros.h"

/**
* @brief This h file contains utilities for parsing single parameters from 
* the ROS parameter server
*
*/

/* PARSEBOOLPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name);

/* PARSEINTPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, int& param, std::string param_name);

/* PARSEDOUBLEPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name);

/* PARSESTRINGPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name);

/* PARSEINTVECTORPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::vector<int>& param, std::string param_name, int min_size = 0);

/* PARSEDOUBLEVECTORPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::vector<double>& param, std::string param_name, int min_size = 0);

/* PARSEINTSTRINGMAPPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::map<int, std::string>& param, std::string param_name);

/* PARSESTRINGSTRINGMAPPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::map<std::string, std::string>& param, std::string param_name);

/* PARSESTRINGVECTORMAPPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, std::map<std::string, std::vector<double>>& param, std::string param_name);

/* PARSEMATRIXPARAMETER */
bool parseParameter(XmlRpc::XmlRpcValue& params, Eigen::MatrixXd& param, std::string param_name);

bool parseParameter(XmlRpc::XmlRpcValue& params, Eigen::MatrixXi& param, std::string param_name);

#endif //PARSING_UTILS_H