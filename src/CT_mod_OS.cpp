#include <pluginlib/class_list_macros.h>
#include <panda_controllers/CT_mod_OS.h> //library of the computed torque 


namespace panda_controllers{

    bool CTModOS::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        this->cvc_nh = node_handle; 
 
        std::string arm_id; //checking up the arm id of the robot     
        if (!node_handle.getParam("arm_id", arm_id)) {
		    ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		    return false;
        }

        /* Inizializing the Kp and Kv gains */
    	double kp1, kp2, kp3, kv1, kv2, kv3, kn1, kn2, kn3;

        /* Chek paramaters acquisition*/
        if (!node_handle.getParam("kp1", kp1) || 
		!node_handle.getParam("kp2", kp2) ||
		!node_handle.getParam("kp3", kp3) || 
		!node_handle.getParam("kv1", kv1) ||
		!node_handle.getParam("kv2", kv2) ||
		!node_handle.getParam("kv3", kv3)) {
		    ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		    return false;
	    }
        // std::vector<double> gainLambda(DOF);
        // if (!node_handle.getParam("gainLambda", gainLambda)){
        //     ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		//     return false;
        // }
        
        /*Set parametri Lambda*/
        // Lambda.setIdentity();
	    // for(int i=0;i<6;i++){
		//     Lambda(i,i) = gainLambda[i];
	    // }

        /* Assign Kp values*/
        Kp = Eigen::MatrixXd::Identity(DOF, DOF);
        Kp(0,0) = kp1; 
        Kp(1,1) = kp1; 
        Kp(2,2) = kp1; 
        Kp(3,3) = kp2; 
        Kp(4,4) = kp2; 
        Kp(5,5) = kp2; 

        /* Assign Kv values*/
        Kv = Eigen::MatrixXd::Identity(DOF, DOF);
        Kv(0,0) = kv1; 
        Kv(1,1) = kv1; 
        Kv(2,2) = kv1; 
        Kv(3,3) = kv2; 
        Kv(4,4) = kv2; 
        Kv(5,5) = kv2;


        if (!node_handle.getParam("kn1", kn1) || 
		!node_handle.getParam("kn2", kn2) ||
		!node_handle.getParam("kn3", kn3)) {
		    ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		    return false;
	    }
        
        /* Assign Kn values*/
        Kn = Eigen::MatrixXd::Identity(NJ, NJ);
        Kn(0,0) = kn1; 
        Kn(1,1) = kn1; 
        Kn(2,2) = kn1; 
        Kn(3,3) = kn1; 
        Kn(4,4) = kn2; 
        Kn(5,5) = kn2;
        Kn(6,6) = kn3; 


        /* Assigning the time */
	    if (!node_handle.getParam("dt", dt)) {
		    ROS_ERROR("Computed Torque: Could not get parameter dt!");
		    return false;
	    }

        /* Chek joint names acquisition*/
        std::vector<std::string> joint_names;
	    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		    ROS_ERROR("Computed Torque: Error in parsing joints name!");
		    return false;
	    }

        /* check model_interface error */
	    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	    if (model_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
		    return false;
	    }   
        /* Chek model_handle error acquisition*/
        try {
		    model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	    } catch (hardware_interface::HardwareInterfaceException& ex) {
	    	ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
		    return false;
	    }
	    
        /* Same check */
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

        
        hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	    if (effort_joint_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
		    return false;
	    }

        for (size_t i = 0; i < 7; ++i) {
		    try {
		    	joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

	    	} catch (const hardware_interface::HardwareInterfaceException& ex) {
		    	ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
		    	return false;
	    	}
	    }
     
         /* Initial parameters acquisition */
        for(int i=0; i<NJ; i++){
            double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz, d1, d2;
            if (!node_handle.getParam("link"+std::to_string(i+1)+"/mass", mass) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_x", cmx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_y", cmy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_z", cmz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixx", xx) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixy", xy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Ixz", xz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyy", yy) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Iyz", yz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/Izz", zz) ||
                !node_handle.getParam("link"+std::to_string(i+1)+"/d1", d1)||
                !node_handle.getParam("link"+std::to_string(i+1)+"/d2", d2)){
                ROS_ERROR("Computed_torque: Error in parsing inertial parameters!");
                return 1;
            }
            param.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
            param_frict.segment((FRICTION)*i, FRICTION) << d1, d2;
	    }
        param_real << 0.73552200000000001, 0.0077354848739999999, -0.0031274395439999996, -0.033394905365999997, 0.014045526761273585, -0.00039510871831575201, -0.00084478578026577801, 0.011624582982752355, -0.00088299513761623202, 0.0049096519673609458;

        /* Param_dyn initial calculation*/
        regrob::reg2dyn(NJ,PARAM,param,param_dyn);
        
        /* Inizializing the R gains to update parameters*/
	    std::vector<double> gainRlinks(NJ), gainRparam(3);
	    Eigen::Matrix<double,PARAM,PARAM> Rlink;
        Eigen::Matrix<double,FRICTION,FRICTION> Rlink_fric;
	    if (!node_handle.getParam("gainRlinks", gainRlinks) ||
	    	!node_handle.getParam("gainRparam", gainRparam) ||
	    	!node_handle.getParam("update_param", update_param_flag)) {
	
		    ROS_ERROR("Computed_torque Could not get gain parameter for R, Kd!");
		    return false;
	    }

        /* setting R weight */
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

        Rlink_fric.setZero();
        Rlink_fric(0,0) = gainRparam[1];
        Rlink_fric(1,1) = gainRparam[2];


        /* Inverse matrix R calculation */
        Rinv.setZero();
        Rinv_fric.setZero();
        for (int i = 0; i<NJ; i++){	
            Rinv.block(i*PARAM, i*PARAM, PARAM, PARAM) = gainRlinks[i]*Rlink; // block permette di fare le operazioni blocco per blocco (dubbio su che principio calcola tale inversa).
            Rinv_fric.block(i*(FRICTION), i*(FRICTION), FRICTION, FRICTION) = gainRlinks[i]*Rlink_fric; 
        }

        /* Initialize joint (torque,velocity) limits */
        tau_limit << 87, 87, 87, 87, 12, 12, 12;
        
        /*Start command subscriber and publisher */
        // this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command_cartesian", 1, &CTModOS::setCommandCB, this);   //it verify with the callback(setCommandCB) that the command joint has been received
        this->sub_flag_update_ = node_handle.subscribe<panda_controllers::flag> ("adaptiveFlag", 1, &CTModOS::setFlagUpdate, this); // Set adaptive_flag to true  
        this->sub_command_j_ = node_handle.subscribe<sensor_msgs::JointState> ("command_joints_opt", 1, &CTModOS::setCommandCBJ, this);

        
        this->pub_err_ = node_handle.advertise<panda_controllers::log_adaptive_cartesian> ("logging", 1); //Public error variables and tau
        this->pub_config_ = node_handle.advertise<panda_controllers::point>("current_config", 1); //Public Xi,dot_XI,ddot_XI 
        this->pub_opt_ = node_handle.advertise<panda_controllers::udata>("opt_data", 1); //Public for optimal problem 
   
        /* Initialize regressor object*/
        fastRegMat.init(NJ);

        /*Initialize Stack Procedure*/
        l = 0;
        t = 0;
        epsilon = 0.1;
        inf1 = 0;
        inf2 = 0;
        H.setZero(10,70);
        E.setZero(70); 
        H_old.setZero(10,70);
        E_old.setZero(70); 
        Y_stack_sum.setZero();
        redY_stack_sum.setZero();
        H_vec.resize(700);

        return true;
    }

   
    void CTModOS::starting(const ros::Time& time){
    
        /* Getting Robot State in order to get q_curr and dot_q_curr */
	    robot_state = state_handle_->getRobotState();
        T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

        
        /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Matrix form  */
	    q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
        dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());
        dot_q_curr_old = dot_q_curr;
        ddot_q_curr.setZero();

        dot_error_q.setZero(); 
        dot_error_Nq0.setZero();

        /* Secure Initialization  */
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
        qr = q_curr; 
		dot_qr.setZero();
	    ddot_qr.setZero();

        ddq_opt.setZero();
        dq_opt.setZero();
        q_opt.setZero();


        // Vector center of limit's joints
        q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

        /*Limiti del franka*/
        q_max_limit << 1.5, 0.5, 1.5, -0.5, 1.50, 2.50, 1.50;
	    q_min_limit << -1.5, -0.5, -1.5, -2.00, -1.50, -0.50, -1.50;
	    dq_limit << 2.1, 2.1, 2.1, 2.1, 2.6, 2.6, 2.6;
        // ddq_limit << 15, 7.5, 10, 12.5, 15, 20, 20;
        ddq_limit << 1.0, 1.0, 1.0, 1.0, 1.3, 1.3, 1.3;


        /* Defining the NEW gains */
        Kp_xi = Kp;
        Kv_xi = Kv;
        
        /* Update regressor */
        dot_param.setZero();
        dot_param_frict.setZero();

        fastRegMat.setInertialParam(param_dyn); // To compute Extimate Matrix M,G,C
        fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_q_curr); // To compute jacobian and regressor

    }


    void CTModOS::update(const ros::Time&, const ros::Duration& period){

	    Eigen::Matrix<double,DOF,DOF> tmp_conversion0, tmp_conversion1, tmp_conversion2;
        Eigen::VectorXd ee_vel_cmd_tot(DOF), ee_acc_cmd_tot(DOF);
        Eigen::VectorXd tmp_position(DOF), tmp_velocity(DOF);

        robot_state = state_handle_->getRobotState();
	    G = Eigen::Map<Eigen::Matrix<double, NJ, 1>> (model_handle_->getGravity().data());
        T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data()); // Homogeneous Transformation Matrix


        // qr = q_opt;
        // dot_qr = dq_opt;
        // ddot_qr = ddq_opt;

        /* Actual position, velocity and acceleration of the joints */
        q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
        dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());
        ddot_q_curr = (dot_q_curr - dot_q_curr_old) / dt; 
        // q_curr_old = q_curr;
        dot_q_curr_old = dot_q_curr; 
        ddot_q_curr_old = ddot_q_curr;


        // ROS_INFO_STREAM(q_curr);
        
    
        // ROS_INFO_STREAM(T0EE.translation()-computeT0EE(q_curr).translation());
        // ROS_INFO_STREAM(T0EE.linear()-computeT0EE(q_curr).linear());

        /* tau_J_d is the desired link-side joint torque sensor signals "without gravity" */
	    tau_J_d = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.tau_J_d.data());

        /* Update pseudo-inverse of J and its derivative */
    	fastRegMat.setArguments(q_curr,dot_q_curr);

        /* Compute pseudo-inverse of J and its derivative */ 
        J = Eigen::Map<Eigen::Matrix<double, DOF, NJ>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
        // J = fastRegMat.getJac_gen();
        J_dot = fastRegMat.getDotJac_gen();
	    //J_pinv = fastRegMat.getPinvJac_gen();
        J_pinv = J.transpose()*((J*J.transpose()).inverse()); // Right pseudo-inverse of J
        //J_T_pinv = J_pinv.transpose(); // Left pseudo-inverse of J'
        J_T_pinv = ((J*J.transpose()).inverse())*J; // Left pseudo-inverse of J'
        J_dot_pinv = fastRegMat.getDotPinvJac_gen();

        /* NullSpace Calculation*/
        N1 = (I7.setIdentity() - J_pinv*J);

        // ROS_INFO_STREAM(qr-q_opt);

        
	    
        /* Compute error translation */
    	ee_position = T0EE.translation();
	    ee_velocity = J.topRows(3)*dot_q_curr;
        // ee_acceleration = J.topRows(3)*ddot_q_curr + J_dot.topRows(3)*dot_q_curr;
        

    	// error.head(3) = ee_pos_cmd - ee_position;
        // dot_error.head(3) = ee_vel_cmd - ee_velocity;
        error.head(3) = computeT0EE(qr).translation() - ee_position;
        dot_error.head(3) = J.topRows(3)*dot_qr - ee_velocity;
        
    
        // ROS_INFO_STREAM(ddq_opt);

    	/* Compute error orientation */
  	    ee_rot = T0EE.linear();
	    ee_omega = J.bottomRows(3)*dot_q_curr;
        ee_rot_cmd = computeT0EE(qr).linear();
        ee_ang_vel_cmd = J.bottomRows(3)*dot_qr;



        Rs_tilde = ee_rot_cmd*ee_rot.transpose();
        Eigen::Matrix<double,3,3> L_tmp, L_dot_tmp;
	    L_tmp = createL(ee_rot_cmd, ee_rot); 
	    L_dot_tmp = createDotL(ee_rot_cmd, ee_rot, ee_ang_vel_cmd, ee_omega);
        L.setIdentity();
        L.block(3, 3, 3, 3) = L_tmp;
        L_dot.setZero();
        L_dot.block(3, 3, 3, 3) = L_dot_tmp;
        
        // vel_cur.tail(3) = L_tmp*ee_omega;
	    error.tail(3) = vect(Rs_tilde);
	    dot_error.tail(3) = L_tmp.transpose()*ee_ang_vel_cmd-L_tmp*ee_omega;
        // ROS_INFO_STREAM(dot_error.tail(3));


        /* Compute reference */
        ee_vel_cmd_tot << ee_vel_cmd, L_tmp.transpose()*ee_ang_vel_cmd;
        ee_acc_cmd_tot << ee_acc_cmd, L_dot_tmp.transpose()*ee_ang_vel_cmd + L_tmp.transpose()*ee_ang_acc_cmd;
        
        tmp_conversion0.setIdentity();
        tmp_conversion0.block(3, 3, 3, 3) = L_tmp;
        tmp_conversion1.setIdentity();
        tmp_conversion1.block(3, 3, 3, 3) = L_tmp.inverse();
        tmp_conversion2.setZero();
        tmp_conversion2.block(3, 3, 3, 3) = -L_tmp.inverse() * L_dot_tmp *L_tmp.inverse();

        // dot_qr = J_pinv*tmp_conversion1*ee_vel_cmd_tot; 
	    // ddot_qr = J_pinv*tmp_conversion1*ee_acc_cmd_tot + J_pinv*tmp_conversion2*ee_vel_cmd_tot +J_dot_pinv*tmp_conversion1*ee_vel_cmd_tot; 
        // /*?*/
        // qr = qr + dot_qr*dt;



        /* Error definition in Joints Space*/
        // dot_error_Nq0 = (dot_error_q - J_pinv*dot_error); //errore derivata nullo(si vuole a zero per adesso)
        dot_error_q = dot_qr - dot_q_curr;
        dot_error_Nq0 = -N1*(dot_q_curr);

    	Kp_xi = Kp;
    	Kv_xi = Kv;

        /* Friction matrix online creation*/
        Dest.setZero();
        for(int i = 0; i < 7; ++i){
            Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*deltaCompute(dot_q_curr[i]);
            // Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*fabs(dot_q_curr[i]);
        }


        /* Application of FIR to velocity and acceleration*/
        // aggiungiDato(buffer_q, q_curr, WIN_LEN);
        // q_curr = calcolaMedia(buffer_q);
        aggiungiDato(buffer_dq, dot_q_curr, WIN_LEN);
        dot_q_curr = calcolaMedia(buffer_dq);
        aggiungiDato(buffer_ddq, ddot_q_curr, WIN_LEN);
        ddot_q_curr = calcolaMedia(buffer_ddq);
      
        // aggiungiDato(buffer_dqr, dot_qr, WIN_LEN);
        // dot_qr_est = calcolaMedia(buffer_dqr);
        // aggiungiDato(buffer_ddqr, ddot_qr, WIN_LEN);
        // ddot_qr_est = calcolaMedia(buffer_ddqr);
        


        /* Update and Compute Regressor */
	    fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);
        Y_mod = fastRegMat.getReg_gen(); // Regressor computetion
        fastRegMat.setArguments(q_curr, dot_q_curr, dot_q_curr, ddot_q_curr);
        Y_norm = fastRegMat.getReg_gen();


        /* Application of FIR to tau*/
        tau_J = tau_cmd + G;
        // tau_J = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.tau_J.data());
        aggiungiDato(buffer_tau, tau_J, WIN_LEN);
        tau_J = calcolaMedia(buffer_tau);
        // ROS_INFO_STREAM(tau_J - Y_norm*param); // - Y_norm.block(0,0,NJ,(NJ-1)*PARAM)*param.segment(0,(NJ-1)*PARAM) - Y_norm.block(0,(NJ-1)*PARAM,NJ,PARAM)*param_real);
        // redtau_J = tau_J;

        Y_D.setZero();
        Y_D_norm.setZero();
        for (int i = 0; i < 7; ++i) {

            /*Friction regressor*/
            Y_D(i, i * 2) = dot_qr(i); 
            Y_D(i, i * 2 + 1) = dot_qr(i)*deltaCompute(dot_q_curr(i)); 
            Y_D_norm(i, i * 2) = dot_q_curr(i); 
            Y_D_norm(i, i * 2 + 1) = dot_q_curr(i)*deltaCompute(dot_q_curr(i));   
        }

        // ROS_INFO_STREAM(Rinv_fric);
        redY_norm = Y_norm.block(0,(NJ-1)*PARAM,NJ,PARAM);
        redtau_J = tau_J - Y_norm.block(0,0,NJ,(NJ-1)*PARAM)*param.segment(0,(NJ-1)*PARAM);
        param7 = param.segment((NJ-1)*PARAM, PARAM);
        // ROS_INFO_STREAM(param7);
        // ROS_INFO_STREAM(redY_norm.transpose());

        /* Update parameters law*/
        if (update_param_flag){             
    
            // fastRegMat.setArguments(qr, dot_qr, dot_qr, ddot_qr);
            // Y_norm_pred = fastRegMat.getReg_gen();

            H_old = H;
            E_old = E;

            /*Saturation command*/
            // for(int i = 0; i < NJ; i++){
            
            //     /*Saturation position*/
            //     if((q_opt(i) > q_max_limit(i))){
            //         qr(i) = q_max_limit(i);
            //     }else if((q_opt(i) < q_min_limit(i))){
            //         qr(i) = q_min_limit(i);
            //     }else{
            //             qr(i) = q_opt(i);
            //     }
                
            //     /*Saturation velocity*/
            //     if(std::abs(dq_opt(i)) > dq_limit(i)){
            //         if(dq_opt(i) < 0){
            //             dot_qr(i) = -dq_limit(i);
            //         }else{
            //             dot_qr(i) = dq_limit(i);    
            //         }
            //     }else{
            //             dot_qr(i) = dq_opt(i);
            //     }

            //     /*Saturation acceleration*/
            //     if(std::abs(ddq_opt(i)) > ddq_limit(i)){
            //         // ROS_INFO_STREAM(ddot_qr(i));
            //         if(ddq_opt(i) < 0){
            //             ddot_qr(i) = -ddq_limit(i);
            //         }else{
            //             ddot_qr(i) = ddq_limit(i);    
            //         }
            //     }else{
            //             ddot_qr(i) = ddq_opt(i);
            //     }
            // }
            
            // inf1 = redStackCompute(Y_norm_pred.block(0,(NJ-1)*PARAM,NJ,PARAM), H, l, redtau_J, E);
            // H = H_old;
            // E = E_old;

            // ROS_INFO_STREAM(inf2);
            // ROS_INFO_STREAM(inf1);
            
            redStackCompute(redY_norm, H, l, redtau_J, E); 

            /*Choose of command*/
            // if (inf2 > inf1){
            //     // H = H_old;
                
            //     // ROS_INFO_STREAM(q_opt);
            //     qr = q_opt;
            //     dot_qr = dq_opt;
            //     ddot_qr = ddq_opt;
            //     dot_error_q = dot_qr - dot_q_curr;
            //     dot_error_Nq0 = -N1*(dot_q_curr);
            
            //     ee_rot_cmd = computeT0EE(qr).linear();
            //     ee_ang_vel_cmd = J.bottomRows(3)*dot_qr;
            //     error.head(3) = computeT0EE(qr).translation() - ee_position;
            //     dot_error.head(3) = J.topRows(3)*dot_qr - ee_velocity;

            //     Rs_tilde = ee_rot_cmd*ee_rot.transpose();
            //     error.tail(3) = vect(Rs_tilde);
	        //     dot_error.tail(3) = L_tmp.transpose()*ee_ang_vel_cmd-L_tmp*ee_omega;
           
            // }     

            /*Casting in vettore*/
            H_vec = Eigen::Map<Eigen::VectorXd> (H.data(), 700);
            
            redY_stack_sum = H*E - H*H.transpose()*param7;
            // redY_stack_sum = H*H.transpose()*(param_real-param7);
            // ROS_INFO_STREAM(H*H.transpose()*param_real -  H*E);
            // ROS_INFO_STREAM(redtau_J - redY_norm*param7);
            
            Y_stack_sum.segment((NJ-1)*PARAM, PARAM) = redY_stack_sum;
                  
            /* Residual computation */
            // err_param = tau_J - Y_norm*param; // - Y_D_norm*param_frict;
    
            dot_param = 0.01*Rinv*(Y_mod.transpose()*dot_error_q + 0.5*Y_stack_sum); // + 0.1*Y_norm.transpose()*(err_param)); 
	        param = param + dt*dot_param; 
            // dot_param_frict = 0.1*Rinv_fric*(Y_D_norm.transpose()*dot_error_q + 0.1*Y_D_norm.transpose()*(err_param));
            // param_frict = param_frict + dt*dot_param_frict;
            
	    }

         /*Reshape parameters vector*/
        for(int i = 0; i < 7; ++i){
            param_tot.segment(i*(PARAM+FRICTION),PARAM) = param.segment(i*(PARAM),PARAM);
            param_tot.segment(i*(PARAM+FRICTION) + PARAM,FRICTION) = param_frict.segment(i*FRICTION,FRICTION);
        }


        /* update dynamic for control law */
        regrob::reg2dyn(NJ,PARAM,param,param_dyn);	// conversion of updated parameters
        fastRegMat.setArguments(q_curr,dot_q_curr_old,param_dyn); 

        Mest = fastRegMat.getMass_gen(); // Estimate Mass Matrix
        Cest = fastRegMat.getCoriolis_gen(); // Estimate Coriollis Matrix
        Gest = fastRegMat.getGravity_gen(); // Estimate Gravity Matrix

        
        /*Matrici nello spazio operativo*/
        // MestXi = J_T_pinv*Mest*J_pinv;
        // CestXi = (J_T_pinv*Cest - MestXi*J_dot)*J_pinv;
        // GestXi = J_T_pinv*Gest;
      
        // /* command torque to joint */
        tau_cmd = Mest*ddot_qr + (Cest)*dot_q_curr_old + Gest + J.transpose()*Kp_xi*error + J.transpose()*Kv_xi*dot_error - G + Kn*dot_error_Nq0 + Cest*dot_error_q; // (- J.transpose()*J_T_pinv*Kn*dot_error_Nq0);

        /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
        tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

        /* Set the command for each joint */
	    for (size_t i = 0; i < 7; i++) {
	     	joint_handles_[i].setCommand(tau_cmd[i]);
	    }

        /* Publish messages */
        time_now = ros::Time::now();
        msg_log.header.stamp = time_now;

        fillMsg(msg_log.error_pos_EE, error);
	    fillMsg(msg_log.dot_error_pos_EE, dot_error);
        fillMsgLink(msg_log.link1, param_tot.segment(0, PARAM+FRICTION));
        fillMsgLink(msg_log.link2, param_tot.segment(12, PARAM+FRICTION));
        fillMsgLink(msg_log.link3, param_tot.segment(24, PARAM+FRICTION));
        fillMsgLink(msg_log.link4, param_tot.segment(36, PARAM+FRICTION));
        fillMsgLink(msg_log.link5, param_tot.segment(48, PARAM+FRICTION));
        fillMsgLink(msg_log.link6, param_tot.segment(60, PARAM+FRICTION));
        fillMsgLink(msg_log.link7, param_tot.segment(72, PARAM+FRICTION));
        fillMsg(msg_log.tau_cmd, dot_error_Nq0);

        msg_config.header.stamp  = time_now; 
        msg_config.xyz.x = T0EE.translation()(0); 
        msg_config.xyz.y = T0EE.translation()(1);
        msg_config.xyz.z = T0EE.translation()(2);

        /*Pubblico posizione e velocità desiderata al passo corrente*/
        fillMsg(msg_opt.q_cur, q_curr);
        fillMsg(msg_opt.dot_q_curr, dot_q_curr);
        fillMsg(msg_opt.ddot_q_curr, ddot_q_curr);
        fillMsg(msg_opt.H_stack, H_vec);

        // ROS_INFO_STREAM(q_curr);
        this->pub_opt_.publish(msg_opt);
        this->pub_err_.publish(msg_log); 
        this->pub_config_.publish(msg_config); 

    }

    void CTModOS::stopping(const ros::Time&)
    {
	//TO DO
    }


    void CTModOS::aggiungiDato(std::vector<Eigen::Matrix<double,NJ, 1>>& buffer_, const Eigen::Matrix<double,NJ, 1>& dato_, int lunghezza_finestra) {
        buffer_.push_back(dato_);
        if (buffer_.size() > lunghezza_finestra) {
            buffer_.erase(buffer_.begin());
        }
    }

    // Funzione per il calcolo della media
    Eigen::Matrix<double,NJ, 1> CTModOS::calcolaMedia(const std::vector<Eigen::Matrix<double,NJ, 1>>& buffer_) {
        Eigen::Matrix<double,NJ, 1> media = Eigen::Matrix<double,NJ, 1>::Zero();
        for (const auto& vettore : buffer_) {
            media += vettore;
        }
        media /= buffer_.size();
        return media;
    }

    void CTModOS::aggiungiDatoXi(std::vector<Eigen::Matrix<double,DOF, 1>>& buffer_, const Eigen::Matrix<double,DOF, 1>& dato_, int lunghezza_finestra) {
        buffer_.push_back(dato_);
        if (buffer_.size() > lunghezza_finestra) {
            buffer_.erase(buffer_.begin());
        }
    }

    // Funzione per il calcolo della media
    Eigen::Matrix<double,DOF, 1> CTModOS::calcolaMediaXi(const std::vector<Eigen::Matrix<double,DOF, 1>>& buffer_) {
        Eigen::Matrix<double,6, 1> media = Eigen::Matrix<double,6, 1>::Zero();
        for (const auto& vettore : buffer_) {
            media += vettore;
        }
        media /= buffer_.size();
        return media;
    }

    double CTModOS::deltaCompute(double a){
        double delta;
        
        if (fabs(a) < 0.01){
            delta = 0.0; 
        }else{
            delta = 1/fabs(a);
        }
        return delta;
    }

    // void CTModOS::stackCompute(const Eigen::Matrix<double, NJ, NJ*PARAM>& Y, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& tau_J, Eigen::VectorXd& E){
    //     const int P = 70;
    //     const double epsilon = 0.1;
    //     Eigen::Matrix<double, NJ, NJ*PARAM> Y_old;

    //     if (l <= (P-1)){
    //         if ((Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(Y.transpose()).norm() >= epsilon){
    //             H.block(0,l*NJ,P,NJ) = Y.transpose();
    //             E.segment(l*NJ,NJ) = tau_J;
    //             l = l+1;
    //             // ROS_INFO_STREAM(H*H.transpose());
    //         }
                                
    //     }else{
    //         if ((Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(Y.transpose()).norm() >= epsilon){
    //             Eigen::MatrixXd Th = H;
                
    //             // Eigen::JacobiSVD<Eigen::Matrix<double, NJ*PARAM, NJ*PARAM>> solver_V(H*H.transpose());
    //             // double V = (solver_V.singularValues()).minCoeff();
    //             double V = (Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(Y.transpose()).norm();

    //             Eigen::VectorXd S(P);
    //             for (int i = 0; i < P; ++i) {
    //                 H.block(0,i*NJ,P,NJ) = Y.transpose();
    //                 // Eigen::JacobiSVD<Eigen::Matrix<double, NJ*PARAM/NJ, NJ*PARAM/NJ>> solver_S(H*H.transpose());
    //                 // S(i) = (solver_S.singularValues()).minCoeff();
    //                 H = Th;

    //                 /*Information Approach(minor compute load?)*/
    //                 S(i) = (Y.transpose()-H.block(0,i*NJ,P,NJ)).norm()/(Y.transpose()).norm();
    //             }
    //             double Vmax = S.maxCoeff();
    //             Eigen::Index m; //index max eigvalues 
    //             S.maxCoeff(&m);
                

    //             if(Vmax >= V){
    //                 H.block(0,m*NJ,P,NJ) = Y.transpose();
    //                 E.segment(m*NJ,NJ) = tau_J;
    //             }else{
    //                 H = Th;
    //                 // E = Te;
    //             }
    //         }
    //     }
    // }

    double CTModOS::redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l, const Eigen::Matrix<double, NJ, 1>& red_tau_J, Eigen::VectorXd& E){
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
                
                Eigen::JacobiSVD<Eigen::Matrix<double, NJ*PARAM, NJ*PARAM>> solver_V(H*H.transpose());
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
                    // ROS_INFO_STREAM(Te-E);
                }
                else{
                    H = Th;
                    E = Te;
                }
            }
        }
        return Vmax;
    }
    

    /* Check for the effort commanded */
    Eigen::Matrix<double, NJ, 1> CTModOS::saturateTorqueRate(
	const Eigen::Matrix<double, NJ, 1>& tau_d_calculated,
	const Eigen::Matrix<double, NJ, 1>& tau_J_d)
    {
	    Eigen::Matrix<double, NJ, 1> tau_d_saturated;
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

    void CTModOS::setCommandCBJ(const sensor_msgs::JointStateConstPtr& msg)
    {
        q_opt = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
        // inf2 = -q_opt(0);

        // ddq_opt = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());
        // dq_opt = dq_opt + dt*ddq_opt;
        // q_opt = q_opt + dt*dq_opt;

        if(update_param_flag){
            qr = q_opt;
        //     dot_qr = dq_opt;
        //     ddot_qr = ddq_opt;
        }

    }

    Eigen::Affine3d CTModOS::computeT0EE(const Eigen::VectorXd& q){
       
        Eigen::Matrix<double, NJ, 4> DH; // matrice D-H
        Eigen::Affine3d T0i = Eigen::Affine3d::Identity();
        Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

        // Riempio sezione distanza "a"
        // DH.block(0,0,NJ,1) << 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
        DH.block(0,0,NJ,1) << 0, 0, 0,0.0825, -0.0825, 0, 0.088;   
        // Riempio sezione angolo "alpha"
        // DH.block(0,1,NJ,1) << -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2, 0;
        DH.block(0,1,NJ,1) << 0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2;
        // Riempio sezione distanza "d"
        // DH.block(0,2,NJ,1) << 0.3330, 0, 0.3160, 0, 0.384, 0, 0.107;
        DH.block(0,2,NJ,1) << 0.3330, 0, 0.3160, 0, 0.384, 0, 0.107;
        // Riempio sezione angolo giunto "theta"
        DH.block(0,3,NJ,1) = q;     

        for (int i = 0; i < NJ; ++i)
        {
            double a_i = DH(i,0);
            double alpha_i = DH(i,1);
            double d_i = DH(i,2);
            double q_i = DH(i,3);

            T << cos(q_i), -sin(q_i), 0, a_i,
                sin(q_i)*cos(alpha_i), cos(q_i)*cos(alpha_i), -sin(alpha_i), -sin(alpha_i)*d_i,
                sin(q_i)*sin(alpha_i), cos(q_i)*sin(alpha_i), cos(alpha_i), cos(alpha_i)*d_i,
                0, 0, 0, 1;

            // Avanzamento perice i 
            T0i.matrix() = T0i.matrix()*T;
        }  
        return T0i;
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
        msg_.d1 = data_[10];
        msg_.d2 = data_[11];
    }

}
PLUGINLIB_EXPORT_CLASS(panda_controllers::CTModOS, controller_interface::ControllerBase);