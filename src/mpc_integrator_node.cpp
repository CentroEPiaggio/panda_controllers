#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <panda_controllers/MpcSolution.h>
#include <panda_controllers/ObstacleStatus.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace panda_controllers
{

    class MpcIntegratorNode
    {
    private:
        ros::NodeHandle nh;

        // Subscriber
        ros::Subscriber sub_mpc_sol;
        ros::Subscriber sub_joint_states;
        // ros::Subscriber sub_palla;
        ros::Subscriber sub_obstacle;
        ros::Subscriber sub_franka_pos;
        
        

        // Publisher
        ros::Publisher pub_cmd;
        ros::Publisher pub_filtered_state; // pubblico q, dq, ddq filtrati verso menu
        ros::Publisher pub_palla_filt;;
        ros::Timer timer;

        // Stato interno dell'integratore
        Eigen::VectorXd q_int_, dq_int_, ddq_int_, jerk_opt_;
        double dt_ctrl_ = 0.001;
        bool has_solution_ = false;

        // Watchdog MPC
        ros::Time last_msg_time_;
        double timeout_ = 0.07;

        // Stato filtrato dal sensore (1kHz)
        Eigen::VectorXd q_filt_, dq_filt_, ddq_filt_;
        Eigen::VectorXd dq_prev_;
        bool first_joint_msg_ = true;

        // Guadagni filtro passa-basso (IIR)
        double alpha_q_ = 0.5;    // posizione: poco filtro, è già pulita
        double alpha_dq_ = 0.2;   // velocità: filtro medio
        double alpha_ddq_ = 0.05; // accelerazione: filtro molto aggressivo (era 0.1)

        // Stato palla
        Eigen::Vector3d p_palla_filt_, v_palla_filt_, p_palla_prev_;
        ros::Time last_palla_time_;
        bool first_palla_msg_ = true;

        // Guadagni filtro palla
        double alpha_p_palla_ = 0.1; // Posizione
        double alpha_v_palla_ = 0.1; // Velocità

        // Pose del robot nel sistema Qualisys
        Eigen::Vector3d franka_pos_qualisys_;
        Eigen::Quaterniond franka_quat_qualisys_;
        bool has_franka_pose_ = false;

        // Limiti accelerazione per clamp
        const double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        const double q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        const double dq_min[7] = {-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61};
        const double dq_max[7] = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
        const double ddq_min[7] = {-15, -7.5, -10, -12.5, -15, -20, -20};
        const double ddq_max[7] = {15, 7.5, 10, 12.5, 15, 20, 20};

        ros::Time last_control_time_;
        ros::Time last_joint_time_;
        bool first_control_ = true;

    public:
        MpcIntegratorNode()
        {
            q_int_.resize(7);
            q_int_.setZero();
            dq_int_.resize(7);
            dq_int_.setZero();
            ddq_int_.resize(7);
            ddq_int_.setZero();
            jerk_opt_.resize(7);
            jerk_opt_.setZero();

            q_filt_.resize(7);
            q_filt_.setZero();
            dq_filt_.resize(7);
            dq_filt_.setZero();
            ddq_filt_.resize(7);
            ddq_filt_.setZero();
            dq_prev_.resize(7);
            dq_prev_.setZero();

            p_palla_filt_.setZero();
            v_palla_filt_.setZero();
            p_palla_prev_.setZero();

            // Inizializzazione pose Qualisys
            franka_pos_qualisys_.setZero();
            franka_quat_qualisys_.setIdentity();

            // Subscriber
            sub_mpc_sol = nh.subscribe(
                "/mpc_solution", 1,
                &MpcIntegratorNode::mpcCallback, this);

            sub_joint_states = nh.subscribe(
                "/franka/joint_states_1khz", 1,
                &MpcIntegratorNode::jointStatesCallback, this);

            // sub_palla = nh.subscribe("/gazebo/model_states", 1,
            //                          &MpcIntegratorNode::pallaCallback, this);

            sub_obstacle = nh.subscribe("/qualisys/mpc_obstacle/pose", 1,
                                        &MpcIntegratorNode::obstacleCallback, this);

            sub_franka_pos = nh.subscribe("/qualisys/mpc_franka/pose", 1,
                                          &MpcIntegratorNode::frankaCallback, this);

            // Publisher
            pub_cmd = nh.advertise<sensor_msgs::JointState>(
                "/computed_torque_controller/command", 1);

            pub_filtered_state = nh.advertise<sensor_msgs::JointState>(
                "/mpc/filtered_joint_state", 1);

            pub_palla_filt = nh.advertise<panda_controllers::ObstacleStatus>(
                "/mpc/obstacle_status", 1);
            // Timer integratore a 1kHz
            timer = nh.createTimer(
                ros::Duration(dt_ctrl_),
                &MpcIntegratorNode::controlLoop, this);
        }

        // ----------------------------------------------------------------
        // Metodo per pubblicare lo stato filtrato
        // ----------------------------------------------------------------
        void publishFilteredStateNow()
        {
            if (first_joint_msg_)
                return;

            sensor_msgs::JointState state_msg;
            state_msg.header.stamp = ros::Time::now();
            state_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                              "panda_joint4", "panda_joint5", "panda_joint6",
                              "panda_joint7"};
            for (int i = 0; i < 7; i++)
            {
                state_msg.position.push_back(q_filt_(i));
                state_msg.velocity.push_back(dq_filt_(i));
                state_msg.effort.push_back(ddq_filt_(i));
            }
            pub_filtered_state.publish(state_msg);
        }

        // ----------------------------------------------------------------
        // CALLBACK JOINT STATES: stima filtrata a 1kHz + pubblicazione immediata
        // ----------------------------------------------------------------
        void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
        {
            if (msg->position.size() < 7 || msg->velocity.size() < 7)
                return;

            Eigen::VectorXd q_raw(7), dq_raw(7);
            for (int i = 0; i < 7; i++)
            {
                q_raw(i) = msg->position[i];
                dq_raw(i) = msg->velocity[i];
            }

            if (first_joint_msg_)
            {
                q_filt_ = q_raw;
                dq_filt_ = dq_raw;
                ddq_filt_.setZero();
                dq_prev_ = dq_raw;
                last_joint_time_ = msg->header.stamp;
                first_joint_msg_ = false;

                // Pubblica subito anche il primo stato
                publishFilteredStateNow();
                return;
            }

            // Stima accelerazione con derivata numerica a 1kHz
            double dt = (msg->header.stamp - last_joint_time_).toSec();
            last_joint_time_ = msg->header.stamp;

            // if (dt > 0.0001 && dt < 0.01)
            // { // dt realistico tra 0.1ms e 10ms
            Eigen::VectorXd ddq_raw = (dq_raw - dq_prev_) / dt;

            // Clamp — limite fisico Franka
            for (int i = 0; i < 7; i++)
                ddq_raw(i) = std::max(ddq_min[i], std::min(ddq_max[i], ddq_raw(i)));

            // Filtro IIR passa-basso
            q_filt_ = alpha_q_ * q_raw + (1.0 - alpha_q_) * q_filt_;
            dq_filt_ = alpha_dq_ * dq_raw + (1.0 - alpha_dq_) * dq_filt_;
            ddq_filt_ = alpha_ddq_ * ddq_raw + (1.0 - alpha_ddq_) * ddq_filt_;

            dq_prev_ = dq_raw;

            // Pubblico immediatamente lo stato filtrato (così il menu ha dati sempre freschi)
            publishFilteredStateNow();
        }

        // ----------------------------------------------------------------
        // CALLBACK MPC: riceve soluzione a ~30Hz (dal menu)
        // ----------------------------------------------------------------
        void mpcCallback(const panda_controllers::MpcSolutionConstPtr &msg)
        {
            last_msg_time_ = ros::Time::now();

            // // Reset stato integrato con feedback reale (q e dq dal sensore filtrato)
            // q_int_ = q_filt_;
            // dq_int_ = dq_filt_;

            // // Usa ddq_start dal messaggio MPC come condizione iniziale
            // for (int i = 0; i < 7; i++)
            //     ddq_int_(i) = msg->ddq_start[i];

            if (first_control_)
            {
                for (int i = 0; i < 7; i++)
                {
                    q_int_(i)= msg->q_start[i];
                    dq_int_(i)= msg->dq_start[i];
                    ddq_int_(i)= msg->ddq_start[i];
                }
                // ddq_int_.setZero;
                first_control_ = false;
            }

            has_solution_ = true;

            if (msg->jerk.size() >= 7)
                for (int i = 0; i < 7; i++)
                    jerk_opt_(i) = msg->jerk[i];
        }

        // Callback per la posa del robot nel sistema Qualisys
        void frankaCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            franka_pos_qualisys_ = Eigen::Vector3d(
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
                
            franka_quat_qualisys_ = Eigen::Quaterniond(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z);
                
            has_franka_pose_ = true;
        }
        // void pallaCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
        // {
        //     int idx = -1;
        //     for (size_t i = 0; i < msg->name.size(); ++i)
        //     {
        //         if (msg->name[i] == "palla")
        //         {
        //             idx = i;
        //             break;
        //         }
        //     }
        //     if (idx == -1)
        //         return;

        //     Eigen::Vector3d p_raw(msg->pose[idx].position.x, msg->pose[idx].position.y, msg->pose[idx].position.z);
        //     ros::Time now = ros::Time::now();

        //     if (first_palla_msg_)
        //     {
        //         p_palla_filt_ = p_raw;
        //         p_palla_prev_ = p_raw;
        //         v_palla_filt_.setZero();
        //         last_palla_time_ = now;
        //         first_palla_msg_ = false;
        //         return;
        //     }

        //     double dt = (now - last_palla_time_).toSec();
        //     if (dt <= 0.0001)
        //         return;

        //     // Derivata e Filtro
        //     Eigen::Vector3d v_raw = (p_raw - p_palla_prev_) / dt;
        //     p_palla_prev_ = p_raw;
        //     last_palla_time_ = now;

        //     p_palla_filt_ = alpha_p_palla_ * p_raw + (1.0 - alpha_p_palla_) * p_palla_filt_;
        //     v_palla_filt_ = alpha_v_palla_ * v_raw + (1.0 - alpha_v_palla_) * v_palla_filt_;

        //     // Pubblicazione Status
        //     panda_controllers::ObstacleStatus obs_msg;
        //     obs_msg.header.stamp = now;
        //     obs_msg.position.x = p_palla_filt_.x();
        //     obs_msg.position.y = p_palla_filt_.y();
        //     obs_msg.position.z = p_palla_filt_.z();
        //     obs_msg.velocity.x = v_palla_filt_.x();
        //     obs_msg.velocity.y = v_palla_filt_.y();
        //     obs_msg.velocity.z = v_palla_filt_.z();
        //     pub_palla_filt.publish(obs_msg);
        // }

        // Callback ostacolo che calcola posizione relativa al robot
        void obstacleCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            if (!has_franka_pose_)
            {
                ROS_WARN_THROTTLE(1.0, "Qualisys: robot pose not yet received");
                return;
            }

            // Posizione ostacolo nel sistema Qualisys
            Eigen::Vector3d p_obs_qualisys(
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);

            // TRASFORMAZIONE: posizione ostacolo rispetto al centro del robot
            Eigen::Vector3d p_rel = franka_quat_qualisys_.inverse() *
                                    (p_obs_qualisys - franka_pos_qualisys_);

            ros::Time now = msg->header.stamp;

            if (first_palla_msg_)
            {
                p_palla_filt_ = p_rel;
                p_palla_prev_ = p_rel;
                v_palla_filt_.setZero();
                last_palla_time_ = now;
                first_palla_msg_ = false;
                return;
            }

            double dt = (now - last_palla_time_).toSec();

            // Protezione da dt troppo piccoli o negativi
            if (dt <= 0.0001)
            {
                p_palla_filt_ = alpha_p_palla_ * p_rel + (1.0 - alpha_p_palla_) * p_palla_filt_;
                return;
            }

            // Calcola velocità per differenze finite
            Eigen::Vector3d v_raw = (p_rel - p_palla_prev_) / dt;

            // Aggiorna storico
            p_palla_prev_ = p_rel;
            last_palla_time_ = now;

            // Filtra posizione e velocità
            p_palla_filt_ = alpha_p_palla_ * p_rel + (1.0 - alpha_p_palla_) * p_palla_filt_;
            v_palla_filt_ = alpha_v_palla_ * v_raw + (1.0 - alpha_v_palla_) * v_palla_filt_;

            // Pubblica stato dell'ostacolo (ora relativo al robot)
            panda_controllers::ObstacleStatus obs_msg;
            obs_msg.header.stamp = now;
            obs_msg.header.frame_id = "panda_link0"; // Sistema di riferimento del robot
            obs_msg.position.x = p_palla_filt_.x();
            obs_msg.position.y = p_palla_filt_.y();
            obs_msg.position.z = p_palla_filt_.z();
            obs_msg.velocity.x = v_palla_filt_.x();
            obs_msg.velocity.y = v_palla_filt_.y();
            obs_msg.velocity.z = v_palla_filt_.z();

            pub_palla_filt.publish(obs_msg);
        }

        // ----------------------------------------------------------------
        // TIMER 1kHz: integrazione triple integrator
        // ----------------------------------------------------------------
        void controlLoop(const ros::TimerEvent &event)
        {
            if (!has_solution_)
                return;

            // Watchdog
            if ((ros::Time::now() - last_msg_time_).toSec() > timeout_)
            {
                has_solution_ = false;
                return;
            }

            double dt = dt_ctrl_;
            if (!first_control_)
            {
                dt = (event.current_real - last_control_time_).toSec();
                if (dt > 0.01)
                    dt = 0.01;
                if (dt < 0.0001)
                    dt = 0.001;
            }
            last_control_time_ = event.current_real;
            // first_control_ = false;

            // Integrazione (usando la dt già calcolata, senza ridefinirla)
            q_int_ += dq_int_ * dt + 0.5 * ddq_int_ * dt * dt + (1.0 / 6.0) * jerk_opt_ * dt * dt * dt;
            dq_int_ += ddq_int_ * dt + 0.5 * jerk_opt_ * dt * dt;
            ddq_int_ += jerk_opt_ * dt;

            for (int i = 0; i < 7; i++)
            {
                q_int_(i) = std::max(q_min[i], std::min(q_max[i], q_int_(i)));
                dq_int_(i) = std::max(dq_min[i], std::min(dq_max[i], dq_int_(i)));
                ddq_int_(i) = std::max(ddq_min[i], std::min(ddq_max[i], ddq_int_(i)));
            }

            // Pubblico comando al computed_torque
            sensor_msgs::JointState cmd_msg;
            cmd_msg.header.stamp = ros::Time::now();
            for (int i = 0; i < 7; i++)
            {
                cmd_msg.position.push_back(q_int_(i));
                cmd_msg.velocity.push_back(dq_int_(i));
                cmd_msg.effort.push_back(ddq_int_(i));
            }
            pub_cmd.publish(cmd_msg);
        }
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_integrator_node");
    panda_controllers::MpcIntegratorNode node;
    ros::spin();
    return 0;
}