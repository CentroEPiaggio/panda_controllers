#include <pluginlib/class_list_macros.h>
#include <panda_controllers/CT_mod_OS.h> //library of the computed torque 

namespace panda_controllers{

    // Definisco funzione oggetto thuderpanda per inizializzare il nodo di controllo
    bool CTModOS::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        this->cvc_nh = node_handle; // cvc_nh definito con tipo ros::NodeHandle in computerd_torque.h
 
        std::string arm_id; //checking up the arm id of the robot     
        if (!node_handle.getParam("arm_id", arm_id)) {
		    ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		    return false;
        }

        /* Inizializing the Kp and Kv gains */
    	double kp1, kp2, kp3, kv1, kv2, kv3;

        /* Chek corretta acquisizione dei parametri del computed torqued*/
        if (!node_handle.getParam("kp1", kp1) || 
		!node_handle.getParam("kp2", kp2) ||
		!node_handle.getParam("kp3", kp3) || 
		!node_handle.getParam("kv1", kv1) ||
		!node_handle.getParam("kv2", kv2) ||
		!node_handle.getParam("kv3", kv3)) {
		    ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		    return false;
	    }

        std::vector<double> gainLambda(6);
        if (!node_handle.getParam("gainLambda", gainLambda)){
            ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		    return false;
        }

        /* Assegno valori Kpi (valori assunti attraverso file panda_controllers_default.yaml quando si fa load nel launch)*/
        Kp = Eigen::MatrixXd::Identity(6, 6);
        Kp(0,0) = kp1; 
        Kp(1,1) = kp1; 
        Kp(2,2) = kp1; 
        Kp(3,3) = kp3; 
        Kp(4,4) = kp3; 
        Kp(5,5) = kp3; 

        /* Assegno valori Kvi*/
        Kv = Eigen::MatrixXd::Identity(6, 6);
        Kv(0,0) = kv1; 
        Kv(1,1) = kv1; 
        Kv(2,2) = kv1; 
        Kv(3,3) = kv3; 
        Kv(4,4) = kv3; 
        Kv(5,5) = kv3; 

        /*Set parametri Lambda*/
        Lambda.setIdentity();
	    for(int i=0;i<6;i++){
		    Lambda(i,i) = gainLambda[i];
	    }


        /* Assigning the time (dt definito come variabile double in computed_torque.h) */
	    if (!node_handle.getParam("dt", dt)) {
		    ROS_ERROR("Computed Torque: Could not get parameter dt!");
		    return false;
	    }

        /* Chek per il acquisizione corretta del nome dei giunti del franka*/
        std::vector<std::string> joint_names;
	    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		    ROS_ERROR("Computed Torque: Error in parsing joints name!");
		    return false;
	    }

        /* Coppia di gestione errore su corretta acquisizione e crezione di model_handel_ */
	    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	    if (model_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
		    return false;
	    }   
        /* Chek errori sul model_handle(necessario per usare i calcoli della dinamica del seriale)*/
        try {
		    model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	    } catch (hardware_interface::HardwareInterfaceException& ex) {
	    	ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
		    return false;
	    }
	    
        /* Coppia di gestione errore ma questa volta sulla corretta acquisizione dell'oggetto state_handle_ */
        franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    	if (state_interface == nullptr) {
	    	ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
	    	return false;
	    }
        try {
		    state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	    } catch (hardware_interface::HardwareInterfaceException& ex) {
		    ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
		    return false;
	    }

        /*Gestione di non so quale errore (in tutti i seguenti errori le variabili assumono valore direttamente facendo riferimento a hardware del robot)*/
        hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	    if (effort_joint_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
		    return false;
	    }

        /* Gestione errore sul joint_handles_ (per ogni giunto si controlla se vi è errore su trasmissione del comando) */
        for (size_t i = 0; i < 7; ++i) {
		    try {
		    	joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

	    	} catch (const hardware_interface::HardwareInterfaceException& ex) {
		    	ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
		    	return false;
	    	}
	    }

         /* Verifica corretta acquisizizone(da dove?) dei parametri inerziali del robot(stimati) */
        for(int i=0; i<NJ; i++){
            double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
            if (!node_handle.getParam("link"+std::to_string(i+1)+"/mass", mass) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_x", cmx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_y", cmy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_z", cmz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixx", xx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixy", xy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixz", xz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyy", yy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyz", yz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Izz", zz)){
                
                ROS_ERROR("Computed_torque: Error in parsing inertial parameters!");
                return 1;
            }
            param.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz; // inserisco ad ogni passo i parametri in unica colonna in ordine
	    }

        /* Credo serva a settare il parametri dinamici del sistema (nel file backsteppinh non è usato)?*/
        regrob::reg2dyn(NJ,PARAM,param,param_dyn);
        
        /* Inizializing the R gains (da capire in che modo si attribuiscono i valori) to update parameters*/
	    std::vector<double> gainRlinks(NJ), gainRparam(3);
	    Eigen::Matrix<double,PARAM,PARAM> Rlink;

        // Gestione di errore nel caso in cui parametri non trovati dal nodo di controllo
	    if (!node_handle.getParam("gainRlinks", gainRlinks) ||
	    	!node_handle.getParam("gainRparam", gainRparam) ||
	    	!node_handle.getParam("update_param", update_param_flag)) {
	
		    ROS_ERROR("Computed_torque Could not get gain parameter for R, Kd!");
		    return false;
	    }

        /* setting della matrice di pesi R per legge di aggiornamento dei parametri */
        Rlink.setZero();
        Rlink(0,0) = gainRparam[0];
        Rlink(1,1) = gainRparam[1];
        Rlink(2,2) = Rlink(1,1);
        Rlink(3,3) = Rlink(1,1);
        Rlink(4,4) = gainRparam[2];
        Rlink(5,5) = gainRparam[3];
        Rlink(6,6) = Rlink(5,5);
        Rlink(7,7) = Rlink(4,4);
        Rlink(8,8) = Rlink(5,5);
        Rlink(9,9) = Rlink(4,4);


        /* Calcolo la matrice inversa di R utile per la legge di controllo */
        Rinv.setZero();
        for (int i = 0; i<NJ; i++){	
            Rinv.block(i*PARAM, i*PARAM, PARAM, PARAM) = gainRlinks[i]*Rlink; // block permette di fare le operazioni blocco per blocco (dubbio su che principio calcola tale inversa).
        }

        /* Initialize joint (torque,velocity) limits */
        tau_limit << 87, 87, 87, 87, 12, 12, 12;
        q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

        /*Start command subscriber and publisher */
        this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command_cartesian", 1, &CTModOS::setCommandCB, this);   //it verify with the callback(setCommandCB) that the command joint has been received
        this->sub_flag_update_ = node_handle.subscribe<panda_controllers::flag> ("adaptiveFlag", 1, &CTModOS::setFlagUpdate, this);  
        this->pub_err_ = node_handle.advertise<panda_controllers::log_adaptive_cartesian> ("logging", 1); //dà informazione a topic loggin l'errore che si commette 
        this->pub_config_ = node_handle.advertise<panda_controllers::point>("current_config", 1); //dà informazione sulla configurazione usata
   
        /* Initialize regressor object (oggetto thunderpanda) */
        fastRegMat.init(NJ);

        return true;
    }

   
    void CTModOS::starting(const ros::Time& time){
    
        /* Getting Robot State in order to get q_curr and dot_q_curr */
	    robot_state = state_handle_->getRobotState();
        T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

        /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Matrix form  */
	    q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
	    dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());
        // J = Eigen::Map<Eigen::Matrix<double, 6, NJ>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	  

        /* Secure Initialization (all'inizio il comando ai giunti corrisponde a stato attuale -> errore iniziale pari a zero) */
	   	ee_pos_cmd = T0EE.translation();
	    ee_rot_cmd = T0EE.linear();
	
    	ee_vel_cmd.setZero();
	    ee_acc_cmd.setZero();

	    ee_ang_vel_cmd.setZero();
	    ee_ang_acc_cmd.setZero();
        
    	/* Compute error */
        error.setZero();
	    dot_error.setZero();

        /* Compute reference (Position Control) */
		dot_qr.setZero();
	    ddot_qr.setZero();


        // definizionw vettore fine corsa
        q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

        /* Defining the NEW gains */
        Kp_xi = Kp;
        Kv_xi = Kv;
        
        /* Update regressor */
        dot_param.setZero();
        fastRegMat.setInertialParam(param_dyn); // setta i parametri dinamici dell'oggetto fastRegMat e calcola una stima del regressore di M,C e G (che può differire da quella riportata dal franka)
        fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_q_curr); // setta i valori delle variabili di giunto di interresse e calcola il regressore Y attuale (oltre a calcolare jacobiani e simili e in maniera ridondante M,C,G)

    }


    void CTModOS::update(const ros::Time&, const ros::Duration& period){

	    Eigen::Matrix<double,6,6> tmp_conversion0, tmp_conversion1, tmp_conversion2;
        Eigen::VectorXd ee_vel_cmd_tot(6), ee_acc_cmd_tot(6);
        Eigen::VectorXd tmp_position(6), tmp_velocity(6);


        /* Solito mappaggio già visto nell'inizializzazione*/
        robot_state = state_handle_->getRobotState();
	    // std::array<double, 49> mass_array = model_handle_->getMass();
	    // std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    	// M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	    // C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	    G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());

        // jacobian = Eigen::Map<Eigen::Matrix<double, 6, NJ>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());

       /* Actual position and velocity of the joints */
        T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data()); // matrice di trasformazione omogenea che mi fa passare da s.d.r base a s.d.r EE
        q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
        dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());
        ddot_q_curr = (dot_q_curr - dot_q_curr_old) / dt; // ricavo accelerazione corrente del giunto attraverso definizione 
        dot_q_curr_old = dot_q_curr; // aggiorno il valore di dot_q_curr_old per il prossimo update

        /* tau_J_d is the desired link-side joint torque sensor signals "without gravity" (un concetto più teorico ideale per inseguimento dato da franka?)*/
	    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

        /* Saturate desired velocity to avoid limits*/  
        // for (int i = 0; i < 7; ++i){
		// double ith_des_vel = std::fabs(dot_qr(i)/q_dot_limit(i));
		//     if( ith_des_vel > 1){
		//         dot_qr = dot_qr / ith_des_vel; 
        //     }    
	    // }

        /* Update pseudo-inverse of J and its derivative */
    	fastRegMat.setArguments(q_curr,dot_q_curr);

        // Jacobians
        /* Compute pseudo-inverse of J and its derivative */
        
        Rs_tilde = ee_rot_cmd*ee_rot.transpose();
        J = fastRegMat.getJac_gen();
        J_dot = fastRegMat.getDotJac_gen();
        Eigen::Matrix<double,3,3> L_tmp, L_dot_tmp;
	    L_tmp = createL(ee_rot_cmd, ee_rot); 
	    L_dot_tmp = createDotL(ee_rot_cmd, ee_rot, ee_ang_vel_cmd, ee_omega);
        L.setIdentity();
        L.block(3, 3, 3, 3) = L_tmp;
        L_dot.setZero();
        L_dot.block(3, 3, 3, 3) = L_dot_tmp;
        // Analytical Jacobians
        Ja = L * J;
        Ja_dot = L_dot * J + L * J_dot;
	    // J_pinv = fastRegMat.getPinvJac_gen();
        J_pinv = J.transpose()*((J*J.transpose()).inverse()); // pseudo-inversa destra di J
        J_T_pinv = ((J*J.transpose()).inverse())*J; // pseudo-inversa sinistra di J trasposto
	    // J_dot_pinv = fastRegMat.getDotPinvJac_gen();
        Ja_pinv =  Ja.transpose()*((Ja*Ja.transpose()).inverse());
        Ja_T_pinv = ((Ja*Ja.transpose()).inverse())*Ja; // pseudo-inversa sinistra di J trasposto
	    
        
        // ROS_INFO_STREAM("Differenza jacobiano:" << J-J);

        /* Compute error translation */

    	ee_position = T0EE.translation();
	    ee_velocity = J.topRows(3)*dot_q_curr;
        ee_acceleration = J.topRows(3)*ddot_q_curr + J_dot.topRows(3)*dot_q_curr;


    	error.head(3) = ee_pos_cmd - ee_position;
    	dot_error.head(3) = ee_vel_cmd - ee_velocity;
        // ddot_error.head(3) = ee_acc_cmd - ee_acceleration;
        vel_cur.head(3) = ee_velocity;
        // ROS_INFO_STREAM(ee_vel_cmd);

    	/* Compute error orientation */

  	    ee_rot = T0EE.linear();
	    ee_omega = J.bottomRows(3)*dot_q_curr;
        vel_cur.tail(3) = ee_omega;
	
	    error.tail(3) = vect(Rs_tilde);
	    dot_error.tail(3) = L_tmp.transpose()*ee_ang_vel_cmd-L_tmp*ee_omega;
        // ddot_error.tail(3) =  ddot_error.head(3) = ee_acc_cmd - ee_acceleration; // da correggere


        /* Compute reference */
        ee_vel_cmd_tot << ee_vel_cmd, L_tmp.transpose()*ee_ang_vel_cmd;
        ee_acc_cmd_tot << ee_acc_cmd, L_dot_tmp.transpose()*ee_ang_vel_cmd + L_tmp.transpose()*ee_ang_acc_cmd;
        
        // tmp_position = ee_vel_cmd_tot + Lambda * error;
        // tmp_velocity = ee_acc_cmd_tot + Lambda * dot_error;
        
        tmp_conversion0.setIdentity();
        tmp_conversion0.block(3, 3, 3, 3) = L_tmp;
        tmp_conversion1.setIdentity();
        tmp_conversion1.block(3, 3, 3, 3) = L_tmp.inverse();
        tmp_conversion2.setZero();
        tmp_conversion2.block(3, 3, 3, 3) = -L_tmp.inverse() * L_dot_tmp *L_tmp.inverse();

        // dot_qr = J_pinv*tmp_conversion1*tmp_position;
	    // ddot_qr = J_pinv*tmp_conversion1*tmp_velocity + J_pinv*tmp_conversion2*tmp_position +J_dot_pinv*tmp_conversion1*tmp_position;


        /* Sempre stessi valori dei guadagni*/
    	Kp_xi = Kp;
    	Kv_xi = Kv;

        /* Update and Compute Regressor */
	    fastRegMat.setArguments(q_curr, dot_q_curr, J_pinv*ee_vel_cmd_tot, J_pinv*ee_acc_cmd_tot);
	    Y = fastRegMat.getReg_gen(); // calcolo del regressore
        
       
	    // dt = period.toSec();    

        /* se vi è stato aggiornamento, calcolo il nuovo valore che paramatri assumono secondo la seguente legge*/
        if (update_param_flag){
            dot_param = Rinv*Y.transpose()*J_pinv*dot_error; // legge aggiornamento parametri se vi è update(CAMBIARE RINV NEGLI ESPERIMENTI)
	        param = param + dt*dot_param;
	    }


        /* update dynamic for control law */
        regrob::reg2dyn(NJ,PARAM,param,param_dyn);	// conversion of updated parameters, nuovo oggetto thunderpsnda
        fastRegMat.setArguments(q_curr,dot_q_curr,param_dyn); // capire se usare questa variante si setArguments è la stessa cosa
        Mest = fastRegMat.getMass_gen(); // matrice di massa stimata usando regressore
        Cest = fastRegMat.getCoriolis_gen(); // matrice di coriolis stimata usando regressore
        Gest = fastRegMat.getGravity_gen(); // modello di gravità stimata usando regressore
        
        /*Matrici nello spazio operativo*/
        MestXi = J_T_pinv*Mest*J_pinv;
        // hestXi = J_pinv.transpose()*(Cest*dot_q_curr + Gest) - MestXi*J_dot*dot_q_curr;
        CestXi = (J_T_pinv*Cest - MestXi*J_dot)*J_pinv;
        GestXi = J_T_pinv*Gest;
        // Eigen::JacobiSVD<Eigen::MatrixXd> svd(CestXi);
        // double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1); 

        // ROS_INFO_STREAM("Differenza jacobiano:" << cond);

        // /*Command in operative space*/
        F_cmd = MestXi*tmp_conversion1*ee_acc_cmd_tot + CestXi*tmp_conversion1*ee_vel_cmd_tot + GestXi + L.transpose()*Kp*error + L.transpose()*Kv*dot_error - MestXi*tmp_conversion2*vel_cur - MestXi*J_dot*J_pinv*tmp_conversion1*ee_vel_cmd_tot;
        // F_cmd = MestXi*ddot_error.setZero() + CestXi*dot_error  + GestXi + Kp*error + Kv*dot_error + MestXi*J_dot*J_pinv*error - J_T_pinv*(Cest*dot_q_curr + Mest*ddot_q_curr);  // legge disperazione 

        // /* command torque to joint */
        tau_cmd = J.transpose()*F_cmd + 1.0*(I7.setIdentity() - J_pinv*J)*(3.0*(q_c-q_curr) - 3.0*dot_q_curr) - G; 
        // tau_cmd = -Mest*J_pinv*J_dot*dot_q_curr + Mest*J_pinv*ee_acc_cmd_tot + J.transpose()*Kv*dot_error + J.transpose()*Kp*error + Cest*dot_q_curr + Gest - G;
        // tau_cmd = tau_cmd - 0.1*(I7.setIdentity() - J_pinv*J)*dot_q_curr;

        // /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	    // tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

        /* Set the command for each joint */
	    for (size_t i = 0; i < 7; i++) {
	     	joint_handles_[i].setCommand(tau_cmd[i]);
	    }

        /* Publish messages (errore di posizione e velocità, coppia comandata ai giunti, parametri dinamici dei vari giunti stimati)*/
        time_now = ros::Time::now();
        msg_log.header.stamp = time_now;

        fillMsg(msg_log.error_pos_EE, error);
	    fillMsg(msg_log.dot_error_pos_EE, dot_error);
        fillMsgLink(msg_log.link1, param.segment(0, PARAM));
        fillMsgLink(msg_log.link2, param.segment(10, PARAM));
        fillMsgLink(msg_log.link3, param.segment(20, PARAM));
        fillMsgLink(msg_log.link4, param.segment(30, PARAM));
        fillMsgLink(msg_log.link5, param.segment(40, PARAM));
        fillMsgLink(msg_log.link6, param.segment(50, PARAM));
        fillMsgLink(msg_log.link7, param.segment(60, PARAM));
        fillMsg(msg_log.tau_cmd, tau_cmd);

        msg_config.header.stamp  = time_now; // publico tempo attuale nodo
        msg_config.xyz.x = T0EE.translation()(0); 
        msg_config.xyz.y = T0EE.translation()(1);
        msg_config.xyz.z = T0EE.translation()(2);

        this->pub_err_.publish(msg_log); // publico su nodo logging, i valori dei parametri aggiornati con la legge di controllo, e e dot_e (in pratica il vettori di stato del problema aumentato) 
        this->pub_config_.publish(msg_config); // publico la configurazione dell'EE?

    }

    void CTModOS::stopping(const ros::Time&)
    {
	//TO DO
    }

    /* Check for the effort commanded */
    Eigen::Matrix<double, 7, 1> CTModOS::saturateTorqueRate(
	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	const Eigen::Matrix<double, 7, 1>& tau_J_d)
    {
	    Eigen::Matrix<double, 7, 1> tau_d_saturated {};
	    for (size_t i = 0; i < 7; i++) {

		    double difference = tau_d_calculated[i] - tau_J_d[i];
		    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

	    }
	    return tau_d_saturated;
    }

    void CTModOS::setCommandCB(const panda_controllers::desTrajEE::ConstPtr& msg)
    {
 	ee_pos_cmd << msg->position.x, msg->position.y, msg->position.z;
	ee_vel_cmd << msg->velocity.x, msg->velocity.y, msg->velocity.z;
	ee_acc_cmd << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;
    }  

    void CTModOS::setFlagUpdate(const panda_controllers::flag::ConstPtr& msg){
        update_param_flag = msg->flag;
    }

    template <size_t N>
    void CTModOS::fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_) {
        int dim = data_.size();
        for (int i = 0; i < dim; i++) {
            msg_[i] = data_[i];
        }
    }

    void CTModOS::fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_) {      
        msg_.mass = data_[0];
        msg_.m_CoM_x = data_[1];
        msg_.m_CoM_y = data_[2];
        msg_.m_CoM_z = data_[3];
        msg_.Ixx = data_[4];
        msg_.Ixy = data_[5];
        msg_.Ixz = data_[6];
        msg_.Iyy = data_[7];
        msg_.Iyz = data_[8];
        msg_.Izz = data_[9];
    }

}
PLUGINLIB_EXPORT_CLASS(panda_controllers::CTModOS, controller_interface::ControllerBase);