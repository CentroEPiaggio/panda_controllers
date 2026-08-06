#include "thunder_franka.h"
#include "franka_gen.h"


thunder_franka::thunder_franka(){
	// KIN_EE_xyzrpy << 0, 0, 0, 0, 0, 0;
	KIN_base_xyzrpy << 0, 0, 0, 0, 0, 0;
	d3q << 0, 0, 0, 0, 0, 0, 0;
	d4q << 0, 0, 0, 0, 0, 0, 0;
	ddq << 0, 0, 0, 0, 0, 0, 0;
	ddqr << 0, 0, 0, 0, 0, 0, 0;
	dq << 0, 0, 0, 0, 0, 0, 0;
	dqr << 0, 0, 0, 0, 0, 0, 0;
	par_DYN << 0.73552200000000001, 0.010517, -0.0042519999999999997, -0.045402999999999999, 0.012515999999999999, -0.000428, -0.001196, 0.010026999999999999, -0.00074100000000000001, 0.0048149999999999998;
	par_REG << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.62976900000000002, -0.025831864842, -8.816765999999999e-05, 0.031472076006000001, 0.0047227978697962445, -2.7874210778799998e-06, 0.0014409216136141078, 0.0065123569584130004, 1.263599064084e-05, 0.0053445837755615559, 4.9706840000000003, 0.019261400500000001, 0.010343993404000001, -0.23670397208000002, 0.71466336900072336, -0.00017908297444049997, 0.0076892278918100007, 0.71795648107738708, 0.019661580965898477, 0.0092131637772112241, 0.646926, -0.002031994566, -0.018579714719999998, 0.0022610063699999997, 0.0085035116240215498, -0.0039833588839355196, 0.010261101821008169, 0.028124284712194955, 0.00076893610294640001, 0.026534991901690206, 3.2286039999999998, 0.088844724872, 0.12672916420800001, -0.21470862320799999, 0.056494926014070829, -0.0082483331406757437, -0.0054876481065622559, 0.052878381999606108, -0.0043772571218395843, 0.018249202292520111, 3.5878950000000001, -0.19076837715, 0.37464440800499998, 0.098502069329999997, 0.067677270250859914, 0.02771584317362585, 0.0039053550262761003, 0.032399430424451321, -0.0016444875773692705, 0.077586149052539605, 1.225946, -0.014653732538, 0.050343472489999999, -0.047121686401999999, 0.039427570958035521, -0.0015152444733270301, -0.0046002455175631054, 0.031460372325260388, 0.0021640520520981297, 0.010869510762828563, 1.666555, 0.10024161669500001, -0.023526756935, -0.017527158935000002, 0.0024804603581707902, 0.0015241109028833149, -0.00010375891721868492, 0.010567766133106952, 9.3569097314604994e-05, 0.01179456023023895, 0.73552200000000001, 0.0077354848739999999, -0.0031274395439999996, -0.033394905365999997, 0.014045526761273585, -0.00039510871831575201, -0.00084478578026577801, 0.011624582982752355, -0.00088299513761623202, 0.0049096519673609458;
	q << 0, 0, 0, 0, 0, 0, 0;
	w << 0, 0, 0, 0, 0, 0;
}

// get the parameter: KIN_EE_xyzrpy
Matrix<double,6,1> thunder_franka::get_KIN_EE_xyzrpy() {return KIN_EE_xyzrpy;}
// get the parameter: KIN_base_xyzrpy
Matrix<double,6,1> thunder_franka::get_KIN_base_xyzrpy() {return KIN_base_xyzrpy;}
// get the parameter: d3q
Matrix<double,7,1> thunder_franka::get_d3q() {return d3q;}
// get the parameter: d4q
Matrix<double,7,1> thunder_franka::get_d4q() {return d4q;}
// get the parameter: ddq
Matrix<double,7,1> thunder_franka::get_ddq() {return ddq;}
// get the parameter: ddqr
Matrix<double,7,1> thunder_franka::get_ddqr() {return ddqr;}
// get the parameter: dq
Matrix<double,7,1> thunder_franka::get_dq() {return dq;}
// get the parameter: dqr
Matrix<double,7,1> thunder_franka::get_dqr() {return dqr;}
// get the parameter: par_DYN
Matrix<double,10,1> thunder_franka::get_par_DYN() {return par_DYN;}
// get the parameter: par_REG
Matrix<double,90,1> thunder_franka::get_par_REG() {return par_REG;}
// get the parameter: q
Matrix<double,7,1> thunder_franka::get_q() {return q;}
// get the parameter: w
Matrix<double,6,1> thunder_franka::get_w() {return w;}
// set the parameter: KIN_EE_xyzrpy
void thunder_franka::set_KIN_EE_xyzrpy(Matrix<double,6,1> value) {KIN_EE_xyzrpy = value;}
// set the parameter: KIN_base_xyzrpy
void thunder_franka::set_KIN_base_xyzrpy(Matrix<double,6,1> value) {KIN_base_xyzrpy = value;}
// set the parameter: d3q
void thunder_franka::set_d3q(Matrix<double,7,1> value) {d3q = value;}
// set the parameter: d4q
void thunder_franka::set_d4q(Matrix<double,7,1> value) {d4q = value;}
// set the parameter: ddq
void thunder_franka::set_ddq(Matrix<double,7,1> value) {ddq = value;}
// set the parameter: ddqr
void thunder_franka::set_ddqr(Matrix<double,7,1> value) {ddqr = value;}
// set the parameter: dq
void thunder_franka::set_dq(Matrix<double,7,1> value) {dq = value;}
// set the parameter: dqr
void thunder_franka::set_dqr(Matrix<double,7,1> value) {dqr = value;}
// set the parameter: par_DYN
void thunder_franka::set_par_DYN(Matrix<double,10,1> value) {par_DYN = value;}
// set the parameter: par_REG
void thunder_franka::set_par_REG(Matrix<double,90,1> value) {par_REG = value;}
// set the parameter: q
void thunder_franka::set_q(Matrix<double,7,1> value) {q = value;}
// set the parameter: w
void thunder_franka::set_w(Matrix<double,6,1> value) {w = value;}

// Save parameters to file
int thunder_franka::save_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "KIN_EE_xyzrpy"))){
		vector<double> KIN_EE_xyzrpy_vect(KIN_EE_xyzrpy.data(), KIN_EE_xyzrpy.data() + 6);
		yamlFile["KIN_EE_xyzrpy"] = KIN_EE_xyzrpy_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "KIN_base_xyzrpy"))){
		vector<double> KIN_base_xyzrpy_vect(KIN_base_xyzrpy.data(), KIN_base_xyzrpy.data() + 6);
		yamlFile["KIN_base_xyzrpy"] = KIN_base_xyzrpy_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		vector<double> d3q_vect(d3q.data(), d3q.data() + 7);
		yamlFile["d3q"] = d3q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		vector<double> d4q_vect(d4q.data(), d4q.data() + 7);
		yamlFile["d4q"] = d4q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		vector<double> ddq_vect(ddq.data(), ddq.data() + 7);
		yamlFile["ddq"] = ddq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		vector<double> ddqr_vect(ddqr.data(), ddqr.data() + 7);
		yamlFile["ddqr"] = ddqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		vector<double> dq_vect(dq.data(), dq.data() + 7);
		yamlFile["dq"] = dq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		vector<double> dqr_vect(dqr.data(), dqr.data() + 7);
		yamlFile["dqr"] = dqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		vector<double> par_DYN_vect(par_DYN.data(), par_DYN.data() + 10);
		yamlFile["par_DYN"] = par_DYN_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		vector<double> par_REG_vect(par_REG.data(), par_REG.data() + 90);
		yamlFile["par_REG"] = par_REG_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		vector<double> q_vect(q.data(), q.data() + 7);
		yamlFile["q"] = q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		vector<double> w_vect(w.data(), w.data() + 6);
		yamlFile["w"] = w_vect;
	}

	try {
		YAML::Emitter emitter;
		emitter.SetIndent(2);
		emitter.SetSeqFormat(YAML::Flow);
		emitter << yamlFile << YAML::Newline;
		std::ofstream fout(par_file);
		fout << emitter.c_str();
		fout.close();
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		return 0;
	}
	return 1;
}


// Load parameters from file
int thunder_franka::load_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;
	try {
		yamlFile = YAML::LoadFile(par_file);

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while loading parameters: " << e.what() << std::endl;
		return 0;
	}

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "KIN_EE_xyzrpy"))){
		if (yamlFile["KIN_EE_xyzrpy"]){
			vector<double> vec(yamlFile["KIN_EE_xyzrpy"].as<vector<double>>());
			KIN_EE_xyzrpy = Eigen::Map<Matrix<double,6,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: KIN_EE_xyzrpy not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "KIN_base_xyzrpy"))){
		if (yamlFile["KIN_base_xyzrpy"]){
			vector<double> vec(yamlFile["KIN_base_xyzrpy"].as<vector<double>>());
			KIN_base_xyzrpy = Eigen::Map<Matrix<double,6,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: KIN_base_xyzrpy not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		if (yamlFile["d3q"]){
			vector<double> vec(yamlFile["d3q"].as<vector<double>>());
			d3q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d3q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		if (yamlFile["d4q"]){
			vector<double> vec(yamlFile["d4q"].as<vector<double>>());
			d4q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d4q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		if (yamlFile["ddq"]){
			vector<double> vec(yamlFile["ddq"].as<vector<double>>());
			ddq = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		if (yamlFile["ddqr"]){
			vector<double> vec(yamlFile["ddqr"].as<vector<double>>());
			ddqr = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		if (yamlFile["dq"]){
			vector<double> vec(yamlFile["dq"].as<vector<double>>());
			dq = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		if (yamlFile["dqr"]){
			vector<double> vec(yamlFile["dqr"].as<vector<double>>());
			dqr = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		if (yamlFile["par_DYN"]){
			vector<double> vec(yamlFile["par_DYN"].as<vector<double>>());
			par_DYN = Eigen::Map<Matrix<double,10,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_DYN not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		if (yamlFile["par_REG"]){
			vector<double> vec(yamlFile["par_REG"].as<vector<double>>());
			par_REG = Eigen::Map<Matrix<double,90,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		if (yamlFile["q"]){
			vector<double> vec(yamlFile["q"].as<vector<double>>());
			q = Eigen::Map<Matrix<double,7,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		if (yamlFile["w"]){
			vector<double> vec(yamlFile["w"].as<vector<double>>());
			w = Eigen::Map<Matrix<double,6,1>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: w not found!" << std::endl;
			return 0;
		}
	}

	return 1;
}


// Manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_fun_SZ_IW];
	thread_local double p4[franka_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_ddot_fun_SZ_IW];
	thread_local double p4[franka_C_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), d3q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_dot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_dot_fun_SZ_IW];
	thread_local double p4[franka_C_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Classic formulation of the manipulator Coriolis matrix
Eigen::Matrix<double,7,7> thunder_franka::get_C_std() {
	thread_local double buffer[49];
	thread_local long long p3[franka_C_std_fun_SZ_IW];
	thread_local double p4[franka_C_std_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_C_std_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Manipulator gravity terms
Eigen::Matrix<double,7,1> thunder_franka::get_G() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_fun_SZ_IW];
	thread_local double p4[franka_G_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Second time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_franka::get_G_ddot() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_ddot_fun_SZ_IW];
	thread_local double p4[franka_G_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Time derivative of the gravity vector
Eigen::Matrix<double,7,1> thunder_franka::get_G_dot() {
	thread_local double buffer[7];
	thread_local long long p3[franka_G_dot_fun_SZ_IW];
	thread_local double p4[franka_G_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_G_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,1>>(buffer);
}

// Jacobian of frame 0
Eigen::Matrix<double,6,7> thunder_franka::get_J_0() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_0_fun_SZ_IW];
	thread_local double p4[franka_J_0_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 1
Eigen::Matrix<double,6,7> thunder_franka::get_J_1() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_1_fun_SZ_IW];
	thread_local double p4[franka_J_1_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 2
Eigen::Matrix<double,6,7> thunder_franka::get_J_2() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_2_fun_SZ_IW];
	thread_local double p4[franka_J_2_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 3
Eigen::Matrix<double,6,7> thunder_franka::get_J_3() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_3_fun_SZ_IW];
	thread_local double p4[franka_J_3_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 4
Eigen::Matrix<double,6,7> thunder_franka::get_J_4() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_4_fun_SZ_IW];
	thread_local double p4[franka_J_4_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 5
Eigen::Matrix<double,6,7> thunder_franka::get_J_5() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_5_fun_SZ_IW];
	thread_local double p4[franka_J_5_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 6
Eigen::Matrix<double,6,7> thunder_franka::get_J_6() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_6_fun_SZ_IW];
	thread_local double p4[franka_J_6_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 7
Eigen::Matrix<double,6,7> thunder_franka::get_J_7() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_7_fun_SZ_IW];
	thread_local double p4[franka_J_7_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame 8
Eigen::Matrix<double,6,7> thunder_franka::get_J_8() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_8_fun_SZ_IW];
	thread_local double p4[franka_J_8_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of frame EE
Eigen::Matrix<double,6,7> thunder_franka::get_J_EE() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_EE_fun_SZ_IW];
	thread_local double p4[franka_J_EE_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_EE_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time second derivative of jacobian matrix of frame EE
Eigen::Matrix<double,6,7> thunder_franka::get_J_EE_ddot() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_EE_ddot_fun_SZ_IW];
	thread_local double p4[franka_J_EE_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_EE_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Time derivative of jacobian matrix of frame EE
Eigen::Matrix<double,6,7> thunder_franka::get_J_EE_dot() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_EE_dot_fun_SZ_IW];
	thread_local double p4[franka_J_EE_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_EE_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Pseudo-Inverse of jacobian matrix of frame EE
Eigen::Matrix<double,7,6> thunder_franka::get_J_EE_pinv() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_EE_pinv_fun_SZ_IW];
	thread_local double p4[franka_J_EE_pinv_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_J_EE_pinv_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,6>>(buffer);
}

// Jacobian of center of mass of link 0
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_0() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_0_fun_SZ_IW];
	thread_local double p4[franka_J_cm_0_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 1
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_1() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_1_fun_SZ_IW];
	thread_local double p4[franka_J_cm_1_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 2
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_2() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_2_fun_SZ_IW];
	thread_local double p4[franka_J_cm_2_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 3
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_3() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_3_fun_SZ_IW];
	thread_local double p4[franka_J_cm_3_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 4
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_4() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_4_fun_SZ_IW];
	thread_local double p4[franka_J_cm_4_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 5
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_5() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_5_fun_SZ_IW];
	thread_local double p4[franka_J_cm_5_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 6
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_6() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_6_fun_SZ_IW];
	thread_local double p4[franka_J_cm_6_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 7
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_7() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_7_fun_SZ_IW];
	thread_local double p4[franka_J_cm_7_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Jacobian of center of mass of link 8
Eigen::Matrix<double,6,7> thunder_franka::get_J_cm_8() {
	thread_local double buffer[42];
	thread_local long long p3[franka_J_cm_8_fun_SZ_IW];
	thread_local double p4[franka_J_cm_8_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_J_cm_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,7>>(buffer);
}

// Manipulator mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_fun_SZ_IW];
	thread_local double p4[franka_M_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Second time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M_ddot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_ddot_fun_SZ_IW];
	thread_local double p4[franka_M_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// Time derivative of the mass matrix
Eigen::Matrix<double,7,7> thunder_franka::get_M_dot() {
	thread_local double buffer[49];
	thread_local long long p3[franka_M_dot_fun_SZ_IW];
	thread_local double p4[franka_M_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_M_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,7>>(buffer);
}

// relative transformation from frame-1to frame 0
Eigen::Matrix<double,4,4> thunder_franka::get_T_0() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_0_fun_SZ_IW];
	thread_local double p4[franka_T_0_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame0to frame 1
Eigen::Matrix<double,4,4> thunder_franka::get_T_1() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_1_fun_SZ_IW];
	thread_local double p4[franka_T_1_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame1to frame 2
Eigen::Matrix<double,4,4> thunder_franka::get_T_2() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_2_fun_SZ_IW];
	thread_local double p4[franka_T_2_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame2to frame 3
Eigen::Matrix<double,4,4> thunder_franka::get_T_3() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_3_fun_SZ_IW];
	thread_local double p4[franka_T_3_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame3to frame 4
Eigen::Matrix<double,4,4> thunder_franka::get_T_4() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_4_fun_SZ_IW];
	thread_local double p4[franka_T_4_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame4to frame 5
Eigen::Matrix<double,4,4> thunder_franka::get_T_5() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_5_fun_SZ_IW];
	thread_local double p4[franka_T_5_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame5to frame 6
Eigen::Matrix<double,4,4> thunder_franka::get_T_6() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_6_fun_SZ_IW];
	thread_local double p4[franka_T_6_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame6to frame 7
Eigen::Matrix<double,4,4> thunder_franka::get_T_7() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_7_fun_SZ_IW];
	thread_local double p4[franka_T_7_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame7to frame 8
Eigen::Matrix<double,4,4> thunder_franka::get_T_8() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_8_fun_SZ_IW];
	thread_local double p4[franka_T_8_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of a fixed joint
Eigen::Matrix<double,4,4> thunder_franka::get_T_JOINT_FIXED(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_JOINT_FIXED_fun_SZ_IW];
	thread_local double p4[franka_T_JOINT_FIXED_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = franka_T_JOINT_FIXED_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general prismatic joint R
Eigen::Matrix<double,4,4> thunder_franka::get_T_JOINT_P(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_JOINT_P_fun_SZ_IW];
	thread_local double p4[franka_T_JOINT_P_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = franka_T_JOINT_P_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general rotoidal joint R
Eigen::Matrix<double,4,4> thunder_franka::get_T_JOINT_R(Matrix<double,1,1> q_joint, Matrix<double,3,1> axis) {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_JOINT_R_fun_SZ_IW];
	thread_local double p4[franka_T_JOINT_R_fun_SZ_W];
	const double* input_[] = {q_joint.data(), axis.data()};
	double* output_[] = {buffer};
	int check = franka_T_JOINT_R_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 0
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_0() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_0_fun_SZ_IW];
	thread_local double p4[franka_T_w_0_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 1
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_1() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_1_fun_SZ_IW];
	thread_local double p4[franka_T_w_1_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 2
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_2() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_2_fun_SZ_IW];
	thread_local double p4[franka_T_w_2_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 3
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_3() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_3_fun_SZ_IW];
	thread_local double p4[franka_T_w_3_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 4
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_4() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_4_fun_SZ_IW];
	thread_local double p4[franka_T_w_4_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 5
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_5() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_5_fun_SZ_IW];
	thread_local double p4[franka_T_w_5_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 6
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_6() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_6_fun_SZ_IW];
	thread_local double p4[franka_T_w_6_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_6_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 7
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_7() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_7_fun_SZ_IW];
	thread_local double p4[franka_T_w_7_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_7_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame 8
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_8() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_8_fun_SZ_IW];
	thread_local double p4[franka_T_w_8_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_8_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to frame EE
Eigen::Matrix<double,4,4> thunder_franka::get_T_w_EE() {
	thread_local double buffer[16];
	thread_local long long p3[franka_T_w_EE_fun_SZ_IW];
	thread_local double p4[franka_T_w_EE_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_T_w_EE_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Manipulator regressor matrix
Eigen::Matrix<double,7,90> thunder_franka::get_Yr() {
	thread_local double buffer[630];
	thread_local long long p3[franka_Yr_fun_SZ_IW];
	thread_local double p4[franka_Yr_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_Yr_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Conversion from dynamic to regressor parameters
Eigen::Matrix<double,90,1> thunder_franka::get_dyn2reg() {
	thread_local double buffer[90];
	thread_local long long p3[franka_dyn2reg_fun_SZ_IW];
	thread_local double p4[franka_dyn2reg_fun_SZ_W];
	const double* input_[] = {par_DYN.data()};
	double* output_[] = {buffer};
	int check = franka_dyn2reg_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,90,1>>(buffer);
}

// Kinematic parameters
Eigen::Matrix<double,54,1> thunder_franka::get_par_KIN() {
	thread_local double buffer[54];
	thread_local long long p3[franka_par_KIN_fun_SZ_IW];
	thread_local double p4[franka_par_KIN_fun_SZ_W];
	const double* input_[] = {KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_par_KIN_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,54,1>>(buffer);
}

// Conversion from regressor to dynamic parameters
Eigen::Matrix<double,90,1> thunder_franka::get_reg2dyn() {
	thread_local double buffer[90];
	thread_local long long p3[franka_reg2dyn_fun_SZ_IW];
	thread_local double p4[franka_reg2dyn_fun_SZ_W];
	const double* input_[] = {par_REG.data()};
	double* output_[] = {buffer};
	int check = franka_reg2dyn_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,90,1>>(buffer);
}

// Regressor matrix of term C*dqr
Eigen::Matrix<double,7,90> thunder_franka::get_reg_C() {
	thread_local double buffer[630];
	thread_local long long p3[franka_reg_C_fun_SZ_IW];
	thread_local double p4[franka_reg_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_reg_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Regressor matrix of term G
Eigen::Matrix<double,7,90> thunder_franka::get_reg_G() {
	thread_local double buffer[630];
	thread_local long long p3[franka_reg_G_fun_SZ_IW];
	thread_local double p4[franka_reg_G_fun_SZ_W];
	const double* input_[] = {q.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_reg_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

// Regressor matrix of the quantity J^T*w of link EE
Eigen::Matrix<double,7,12> thunder_franka::get_reg_JTw_EE() {
	thread_local double buffer[84];
	thread_local long long p3[franka_reg_JTw_EE_fun_SZ_IW];
	thread_local double p4[franka_reg_JTw_EE_fun_SZ_W];
	const double* input_[] = {q.data(), w.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_reg_JTw_EE_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,12>>(buffer);
}

// Regressor matrix of the quantity J*dq of link EE
Eigen::Matrix<double,6,12> thunder_franka::get_reg_Jdq_EE() {
	thread_local double buffer[72];
	thread_local long long p3[franka_reg_Jdq_EE_fun_SZ_IW];
	thread_local double p4[franka_reg_Jdq_EE_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_reg_Jdq_EE_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,12>>(buffer);
}

// Regressor matrix of term M*ddqr
Eigen::Matrix<double,7,90> thunder_franka::get_reg_M() {
	thread_local double buffer[630];
	thread_local long long p3[franka_reg_M_fun_SZ_IW];
	thread_local double p4[franka_reg_M_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data(), KIN_base_xyzrpy.data(), KIN_EE_xyzrpy.data()};
	double* output_[] = {buffer};
	int check = franka_reg_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,7,90>>(buffer);
}

