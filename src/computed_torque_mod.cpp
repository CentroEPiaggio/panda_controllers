//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque_mod.h> //library of the computed torque 

namespace panda_controllers{

    // Definisco funzione oggetto thuderpanda per inizializzare il nodo di controllo
    bool ComputedTorqueMod::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        this->cvc_nh = node_handle; // cvc_nh definito con tipo ros::NodeHandle in computerd_torque.h
 
        std::string arm_id; //checking up the arm id of the robot     
        if (!node_handle.getParam("arm_id", arm_id)) {
		    ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		    return false;
        }

        /* Inizializing the Kp and Kv gains (da capire da dove si prendono valori forse dal rosbag_CT.launch) */
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

        /* Assegno valori Kpi (valori assunti attraverso file panda_controllers_default.yaml quando si fa load nel launch)*/
        Kp = Eigen::MatrixXd::Identity(7, 7);
        Kp(0,0) = kp1; 
        Kp(1,1) = kp1; 
        Kp(2,2) = kp1; 
        Kp(3,3) = kp1; 
        Kp(4,4) = kp2; 
        Kp(5,5) = kp2; 
        Kp(6,6) = kp3;
        /* Assegno valori Kvi*/
        Kv = Eigen::MatrixXd::Identity(7, 7);
        Kv(0,0) = kv1; 
        Kv(1,1) = kv1; 
        Kv(2,2) = kv1; 
        Kv(3,3) = kv1; 
        Kv(4,4) = kv2; 
        Kv(5,5) = kv2; 
        Kv(6,6) = kv3;

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
            double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz,d1, d2;
            if (!node_handle.getParam("link"+std::to_string(i+1)+"/mass", mass) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_x", cmx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_y", cmy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_z", cmz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixx", xx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixy", xy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixz", xz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyy", yy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyz", yz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Izz", zz)||
                !node_handle.getParam("link"+std::to_string(i+1)+"/d1", d1)||
                !node_handle.getParam("link"+std::to_string(i+1)+"/d2", d2)){
                
                ROS_ERROR("Computed_torque: Error in parsing inertial parameters!");
                return 1;
            }
            param.segment((PARAM)*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz; // inserisco ad ogni passo i parametri in unica colonna in ordine
            param_frict.segment((FRICTION)*i, FRICTION) << d1, d2;
	    }

        /* Credo serva a settare il parametri dinamici del sistema (nel file backsteppinh non è usato)?*/
        thunder_ns::reg2dyn(NJ,PARAM,param,param_dyn);
        
        /* Inizializing the R gains (da capire in che modo si attribuiscono i valori) to update parameters*/
	    std::vector<double> gainRlinks(NJ), gainRparam(3);
	    Eigen::Matrix<double,PARAM,PARAM> Rlink;
        Eigen::Matrix<double,FRICTION,FRICTION> Rlink_fric;

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
        Rlink(5,5) = gainRparam[3]; // Termini misti tensore di inerzia stimati meno
        Rlink(6,6) = Rlink(5,5);
        Rlink(7,7) = Rlink(4,4);
        Rlink(8,8) = Rlink(5,5);
        Rlink(9,9) = Rlink(4,4);

        Rlink_fric.setZero();
        Rlink_fric(0,0) = gainRparam[1];
        Rlink_fric(1,1) = gainRparam[2];


        /* Calcolo la matrice inversa di R utile per la legge di controllo */
        Rinv.setZero();
        Rinv_fric.setZero();
        for (int i = 0; i<NJ; i++){	
            Rinv.block(i*(PARAM), i*(PARAM), PARAM, PARAM) = gainRlinks[i]*Rlink; // block permette di fare le operazioni blocco per blocco (dubbio su che principio calcola tale inversa).
            Rinv_fric.block(i*(FRICTION), i*(FRICTION), FRICTION, FRICTION) = gainRlinks[i]*Rlink_fric; 
        }

        /* Initialize joint (torque,velocity) limits */
        tau_limit << 87, 87, 87, 87, 12, 12, 12;
        q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

        /*Start command subscriber and publisher */
        this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("/computed_torque_mod_controller/command_joints_opt", 1, &ComputedTorqueMod::setCommandCB, this);   //it verify with the callback(setCommandCB) that the command joint has been received
        this->sub_flag_update_ = node_handle.subscribe<panda_controllers::flag> ("/computed_torque_mod_controller/adaptiveFlag", 1, &ComputedTorqueMod::setFlagUpdate, this);  
        // this->sub_flag_opt_ = node_handle.subscribe<panda_controllers::flag>("/computed_torque_mod_controller/optFlag", 1, &ComputedTorqueMod::setFlagOpt, this);
        
        this->pub_err_ = node_handle.advertise<panda_controllers::log_adaptive_joints> ("logging", 1); //dà informazione a topic loggin l'errore che si commette 
        this->pub_config_ = node_handle.advertise<panda_controllers::point>("current_config", 1); //dà informazione sulla configurazione usata
        this->pub_opt_ = node_handle.advertise<panda_controllers::udata>("opt_data", 1); //Public for optimal problem 

        /* Initialize regressor object (oggetto thunderpanda) */
        // fastRegMat.init(NJ);

        /*Resize*/
        H_vec.resize(700);
        H.setZero(10,70);
        E.setZero(70);
        l = 0;

        Y_stack_sum.setZero();
        redY_stack_sum.setZero();

        return true;
    }

   
    void ComputedTorqueMod::starting(const ros::Time& time){
    
        /* Getting Robot State in order to get q_curr and dot_q_curr */
	    franka::RobotState robot_state = state_handle_->getRobotState();

        /* Getting Mass and coriolis (usando le stime attuate dal franka per i calcolo di queste?) 
        std::array<double, 49> mass_array = model_handle_->getMass();
	    std::array<double, 7> coriolis_array = model_handle_->getCoriolis(); */

        /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Matrix form  */
	    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
	  /*  M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data()); // matrice di massa calcolata usando librerie/stime del franka */
	  /* C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data()); // matrice di coriolis calcolata usando librerie/stime del franka */

        /* Secure Initialization (all'inizio il comando ai giunti corrisponde a stato attuale -> errore iniziale pari a zero) */
	    command_q_d = q_curr; // comando desiderato di posizione
        command_q_d_old = q_curr;
        command_dot_q_d = dot_q_curr; // comando desiderato di velocità
	    command_dot_q_d_old = dot_q_curr;
        command_dot_dot_q_d.setZero(); // inizialmente il comando di accelerazione si setta a zero per ogni giunto

        dot_q_curr_old = dot_q_curr;
        ddot_q_curr_old = (dot_q_curr - dot_q_curr_old) / dt;
        ddot_q_curr.setZero();
        dot_param.setZero();
        dot_param_frict.setZero();

        q_est = q_curr;
        dq_est = dot_q_curr;
        buffer_dq.push_back(dot_q_curr);

        /* Compute error (PARTE AGGIUNTA DI INIZIALIZZAZIONE)*/
        error.setZero();
	    dot_error.setZero();
        param_tot.setZero();

        /* Defining the NEW gains */
        Kp_apix = Kp;
        Kv_apix = Kv;

        
        dot_param.setZero();
        dot_param_frict.setZero();
        
        /* Update regressor */
        // fastRegMat.setInertialParam(param_dyn); // setta i parametri dinamici dell'oggetto fastRegMat e calcola una stima del regressore di M,C e G (che può differire da quella riportata dal franka)
        fastRegMat.set_inertial_REG(param);
        fastRegMat.setArguments(q_curr, dot_q_curr, command_dot_q_d, command_dot_dot_q_d); // setta i valori delle variabili di giunto di interresse e calcola il regressore Y attuale (oltre a calcolare jacobiani e simili e in maniera ridondante M,C,G)
    
    }


    void ComputedTorqueMod::update(const ros::Time&, const ros::Duration& period){
        
        /* Solito mappaggio già visto nell'inizializzazione*/
        franka::RobotState robot_state = state_handle_->getRobotState();
	    std::array<double, 49> mass_array = model_handle_->getMass();
	    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	    C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	    G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());

    	/* =============================================================================== */
        /* check matrix per vedere le stime riprodotte seguendo il calcolo del regressore*/
        /*
        fastRegMat.setArguments(q_curr,dot_q_curr,param_dyn);
        Mest = fastRegMat.getMass_gen();
        Cest = fastRegMat.getCoriolis_gen();
        Gest = fastRegMat.getGravity_gen();
        */
       /* =============================================================================== */
      
       /* Actual position and velocity of the joints */
        T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data()); // matrice di trasformazione omogenea che mi fa passare da s.d.r base a s.d.r EE
        q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
        dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

        

    
        // q_est_old = q_est;
        // q_est = q_est + 0.9*(q_curr - q_est);
        // dot_q_curr = (q_curr - q_est_old)/dt;

        ddot_q_curr = (dot_q_curr - dot_q_curr_old)/dt;
        dot_q_curr_old = dot_q_curr;


        // dq_est.setZero();
        // ddot_q_curr_old.setZero();
        // Filro semplice per ddot_q_curr(tipo feeding filter)
        // ddot_q_curr = 0.8*ddot_q_curr + 0.2*ddot_q_curr_old;
        // ddot_q_curr_old = ddot_q_curr;

        /* tau_J_d is the desired link-side joint torque sensor signals "without gravity" (un concetto più teorico ideale per inseguimento dato da franka?)*/
	    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
        // tau_J = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());

        /* Saturate desired velocity to avoid limits*/  
        // for (int i = 0; i < 7; ++i){
		// double ith_des_vel = std::fabs(command_dot_q_d(i)/q_dot_limit(i));
		//     if( ith_des_vel > 1){
		//         command_dot_q_d = command_dot_q_d / ith_des_vel; 
        //     }    
	    // }

        error = command_q_d - q_curr; // errore di posizione(posizione desiderata - quella reale)
	    dot_error = command_dot_q_d - dot_q_curr; // errore di velocità ai giunti 
	    x.head(7) = error; // x indica la variabile di stato composta da [error dot_error]'
	    x.tail(7) = dot_error;

     /*   // Publish tracking errors as joint states(ad ogni update nodo mi mostra il valore di errore di posizione e velocità ai giunti e l'istante di riferimento)
        sensor_msgs::JointState error_msg;
        std::vector<double> err_vec(error.data(), error.data() + error.rows()*error.cols());
        std::vector<double> dot_err_vec(dot_error.data(), dot_error.data() + dot_error.rows()*dot_error.cols());
        
        error_msg.header.stamp = ros::Time::now();
        error_msg.position = err_vec;
        error_msg.velocity = dot_err_vec;
        this->pub_err_.publish(error_msg);*/

        /* Sempre stessi valori dei guadagni*/
    	Kp_apix = Kp;
    	Kv_apix = Kv;

        /*Compute Friction matrix before filter*/
        Dest.setZero();
        for(int i = 0; i < 7; ++i){
            // Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*fabs(dot_q_curr(i));
            Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*deltaCompute(dot_q_curr(i));
        }
        

        // Filtro velocità e accelerazioni dopo calcolo errore
        aggiungiDato(buffer_dq, dot_q_curr, WIN_LEN);
        dq_est = calcolaMedia(buffer_dq);
        dot_q_curr = dq_est;
        aggiungiDato(buffer_ddq, ddot_q_curr, WIN_LEN);
        ddot_q_curr = calcolaMedia(buffer_ddq);

        /* Update and Compute Regressor mod e Regressor Classic*/
	    fastRegMat.setArguments(q_curr, dot_q_curr, command_dot_q_d, command_dot_dot_q_d);
	    Y_mod = fastRegMat.getReg(); // calcolo del regressore
        fastRegMat.setArguments(q_curr, dot_q_curr, dot_q_curr, ddot_q_curr);
        Y_norm = fastRegMat.getReg();
        // ROS_INFO_STREAM(Y_norm.transpose());
        redY_norm = Y_norm.block(0,(NJ-1)*PARAM,NJ,PARAM);


        /*Calcolo a meno del regressore di attrito*/
        Y_D.setZero();
        Y_D_norm.setZero();
        for (int i = 0; i < 7; ++i) {
            Y_D(i, i * 2) = command_dot_q_d(i); // Imposta 1 sulla diagonale principale
            // Y_D(i, i * 2 + 1) = command_dot_q_d(i)*fabs(dot_q_curr(i)); // Imposta q_i sulla colonna successiva alla diagonale
            Y_D(i, i * 2 + 1) = command_dot_q_d(i)*deltaCompute(dot_q_curr(i));
            Y_D_norm(i, i * 2) = dot_q_curr(i); // Imposta 1 sulla diagonale principale
            // Y_D_norm(i, i * 2 + 1) = dot_q_curr(i)*fabs(dot_q_curr(i)); // Imposta q_i sulla colonna successiva alla diagonale
            Y_D_norm(i, i * 2 + 1) = dot_q_curr(i)*deltaCompute(dot_q_curr(i));
        }

        // ROS_INFO_STREAM("Y_D:" << Y_D << "Dest:" << Dest);

        /*Gravità compensata non va nel calcolo del residuo*/
        tau_J = tau_cmd;
        aggiungiDato(buffer_tau, tau_J, WIN_LEN);

        // Media dei dati nella finestra del filtro
        tau_J = calcolaMedia(buffer_tau);
        // Y_mod_D << Y_mod, Y_D; // concatenation
        // Y_norm_D << Y_norm, Y_D; // concatenation
        
        err_param = tau_J - Y_norm*param; // - Y_D_norm*param_frict;
        
        redY_norm = Y_norm.block(0,(NJ-1)*PARAM,NJ,PARAM);
        redtau_J = tau_J - Y_norm.block(0,0,NJ,(NJ-1)*PARAM)*param.segment(0,(NJ-1)*PARAM);
        param7 = param.segment((NJ-1)*PARAM, PARAM);

        /* se vi è stato aggiornamento, calcolo il nuovo valore che paramatri assumono secondo la seguente legge*/
        if (update_param_flag){
            // ROS_INFO("ciao");
            H_vec = Eigen::Map<Eigen::VectorXd> (H.data(), 700);

            redStackCompute(redY_norm, H, l, tau_J, E);

            Y_stack_sum.segment((NJ-1)*PARAM, PARAM) = redY_stack_sum;
            // dot_param = 0.01*Rinv*(Y_mod.transpose()*dot_error + 0.3*Y_norm.transpose()*(err_param)); // legge aggiornamento parametri se vi è update(CAMBIARE RINV NEGLI ESPERIMENTI)
            dot_param = 0.01*Rinv*(Y_mod.transpose()*dot_error + 0.2*Y_stack_sum + 0.3*Y_norm.transpose()*(err_param));
            param = param + dt*dot_param;
            // dot_param_frict = 0.01*Rinv_fric*(Y_D.transpose()*dot_error + 0.3*Y_D_norm.transpose()*(err_param));
            // param_frict = param_frict + dt*dot_param_frict;
	    }


        /*Riordino parametri per ogni link*/
        for(int i = 0; i < 7; ++i){
            param_tot.segment(i*(PARAM+FRICTION),PARAM) = param.segment(i*(PARAM),PARAM);
            param_tot.segment(i*(PARAM+FRICTION) + PARAM,FRICTION) = param_frict.segment(i*FRICTION,FRICTION);
        }
        // ROS_INFO_STREAM(param_tot);

        /* update dynamic for control law */
        // thunder_ns::reg2dyn(NJ,PARAM,param,param_dyn);	// conversion of updated parameters, nuovo oggetto thunderpsnda
        fastRegMat.set_inertial_REG(param); // capire se usare questa variante si setArguments è la stessa cosa
        Mest = fastRegMat.getMass(); // matrice di massa stimata usando regressore
        Cest = fastRegMat.getCoriolis(); // matrice di coriolis stimata usando regressore
        Gest = fastRegMat.getGravity(); // modello di gravità stimata usando regressore

        /* command torque to joint */
        tau_cmd = Mest * command_dot_dot_q_d + Cest * command_dot_q_d  + Kp_apix * error + Kv_apix * dot_error + Gest;// + Dest*command_dot_q_d; // perchè si sottrae G a legge controllo standard?  legge controllo computed torque (usare M,C e G dovrebbe essere la stessa cosa)
        // tau_cmd = Mest * command_dot_dot_q_d + Cest * dot_q_curr  + Kp_apix * error + Kv_apix * dot_error + Gest - G; // perchè si sottrae G a legge controllo standard?  legge controllo computed torque (usare M,C e G dovrebbe essere la stessa cosa)



        // /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d+G);

        /* Set the command for each joint */
	    for (size_t i = 0; i < 7; i++) {
	    	joint_handles_[i].setCommand(tau_cmd[i]-G[i]);
	    }

        /* Publish messages (errore di posizione e velocità, coppia comandata ai giunti, parametri dinamici dei vari giunti stimati)*/
        time_now = ros::Time::now();
        msg_log.header.stamp = time_now;

        fillMsg(msg_log.error_q, error);
        fillMsg(msg_log.dot_error_q, dot_error);
        fillMsg(msg_log.q_cur, q_curr);
        fillMsgLink(msg_log.link1, param_tot.segment(0, PARAM+FRICTION));
        fillMsgLink(msg_log.link2, param_tot.segment(12, PARAM+FRICTION));
        fillMsgLink(msg_log.link3, param_tot.segment(24, PARAM+FRICTION));
        fillMsgLink(msg_log.link4, param_tot.segment(36, PARAM+FRICTION));
        fillMsgLink(msg_log.link5, param_tot.segment(48, PARAM+FRICTION));
        fillMsgLink(msg_log.link6, param_tot.segment(60, PARAM+FRICTION));
        fillMsgLink(msg_log.link7, param_tot.segment(72, PARAM+FRICTION));
        fillMsg(msg_log.tau_cmd, err_param);
        fillMsg(msg_log.ddot_q_curr, ddot_q_curr);

        msg_config.header.stamp  = time_now; // publico tempo attuale nodo
        msg_config.xyz.x = T0EE.translation()(0); 
        msg_config.xyz.y = T0EE.translation()(1);
        msg_config.xyz.z = T0EE.translation()(2);

        fillMsg(msg_opt.q_cur, q_curr);
        fillMsg(msg_opt.dot_q_curr, dot_q_curr);
        fillMsg(msg_opt.ddot_q_curr, ddot_q_curr);
        fillMsg(msg_opt.H_stack, H_vec);

        this->pub_opt_.publish(msg_opt);
        this->pub_err_.publish(msg_log); // publico su nodo logging, i valori dei parametri aggiornati con la legge di controllo, e e dot_e (in pratica il vettori di stato del problema aumentato) 
        this->pub_config_.publish(msg_config); // publico la configurazione dell'EE?
    }

    void ComputedTorqueMod::stopping(const ros::Time&)
    {
	//TO DO
    }

    double ComputedTorqueMod::redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& red_tau_J, Eigen::VectorXd& E){
        const int P = 10;
        const double epsilon = 0.1;
        double Vmax = 0;

        if (l <= (P-1)){
            if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
                H.block(0,l*NJ,P,NJ) = red_Y.transpose();
                E.segment(l*NJ,NJ) = red_tau_J;
                l = l+1;
                // ROS_INFO_STREAM(H*H.transpose());
            }
                                
        }else{
            if ((red_Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
                Eigen::MatrixXd Th = H;
                Eigen::MatrixXd Te = E;
                
                Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_V(H*H.transpose());
                double V = (solver_V.singularValues()).minCoeff();
                // double V = (red_Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(red_Y.transpose()).norm();

                Eigen::VectorXd S(P);
                for (int i = 0; i < P; ++i) {
                    H.block(0,i*NJ,P,NJ) = red_Y.transpose();
                    Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_S(H*H.transpose());
                    S(i) = (solver_S.singularValues()).minCoeff();
                    H = Th;
                    // ROS_INFO_STREAM(solver_S.singularValues());

                    /*Information Approach(minor compute load?)*/
                    // S(i) = (Y.transpose()-H.block(0,i*NJ,P,NJ)).norm()/(Y.transpose()).norm();
                }
                Vmax = S.maxCoeff();
                Eigen::Index m; //index max eigvalues 
                S.maxCoeff(&m);
                

                if(Vmax >= V){
                    H.block(0,m*NJ,P,NJ) = red_Y.transpose();
                    // Te = E;
                    E.segment(m*NJ,NJ) = red_tau_J;
                    ROS_INFO_STREAM(Vmax);
                }
                else{
                    H = Th;
                    E = Te;
                }
            }
        }
        // ROS_INFO_STREAM(Vmax);
        return Vmax;
    }


    // Funzione per l'aggiunta di un dato al buffer_dq
    void ComputedTorqueMod::aggiungiDato(std::vector<Eigen::Matrix<double,7, 1>>& buffer_, const Eigen::Matrix<double,7, 1>& dato_, int lunghezza_finestra) {
        buffer_.push_back(dato_);
        if (buffer_.size() > lunghezza_finestra) {
            buffer_.erase(buffer_.begin());
        }
    }

    // Funzione per il calcolo della media
    Eigen::Matrix<double,7, 1> ComputedTorqueMod::calcolaMedia(const std::vector<Eigen::Matrix<double,7, 1>>& buffer_) {
        Eigen::Matrix<double,7, 1> media = Eigen::Matrix<double,7, 1>::Zero();
        for (const auto& vettore : buffer_) {
            media += vettore;
        }
        media /= buffer_.size();
        return media;
    }

    double ComputedTorqueMod::deltaCompute (double a){
        double delta;
        
        if (fabs(a) < 0.01){
            delta = 0.0; 
        }else{
            delta = 1/fabs(a);
        }
        return delta;
    }

    /* Check for the effort commanded */
    Eigen::Matrix<double, 7, 1> ComputedTorqueMod::saturateTorqueRate(
	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	const Eigen::Matrix<double, 7, 1>& tau_J_d)
    {
	    Eigen::Matrix<double, 7, 1> tau_d_saturated;
	    for (size_t i = 0; i < 7; i++) {

		    double difference = tau_d_calculated[i] - tau_J_d[i];
		    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

	    }
	    return tau_d_saturated;
    }

    void ComputedTorqueMod::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());
        // command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
        // command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());
        command_dot_q_d = command_dot_q_d + dt*command_dot_dot_q_d;
        command_q_d = command_q_d + dt*command_dot_q_d;

    }

    void ComputedTorqueMod::setFlagUpdate(const panda_controllers::flag::ConstPtr& msg){
        update_param_flag = msg->flag;
    }


    template <size_t N>
    void ComputedTorqueMod::fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_) {
        int dim = data_.size();
        for (int i = 0; i < dim; i++) {
            msg_[i] = data_[i];
        }
    }

    void ComputedTorqueMod::fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_) {      
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
        msg_.d1 = data_[10];
        msg_.d2 = data_[11];
    }

}
PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorqueMod, controller_interface::ControllerBase);