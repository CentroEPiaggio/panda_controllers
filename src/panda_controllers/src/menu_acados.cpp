#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>

// ROS Includes
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"
#include <panda_controllers/MpcSolution.h>

// --- INCLUSIONI CUSTOM & ACADOS ---
#include "MinJerkTrajectory.h"
#include "utils/thunder_frankino.h"
extern "C"
{
#include "acados_solver_frankino_tracking_mpc.h"
#include "acados_sim_solver_frankino_tracking_mpc.h"
#include "acados_c/ocp_nlp_interface.h"
}
#include "utils/KinematicsSolver.h" // Aggiunto per CLIK

const std::string conf_file = "../config/frankino_conf.yaml";
using namespace std;
using namespace Eigen;

// --- PARAMETRI ACADOS ---
#define N_HORIZON 20
#define NX 21                                                                                // [q, dq, ddq]
#define NU 7                                                                                 // [jerk]
#define NY 28                                                                                // [q, dq, ddq, jerk] nel cost stage
#define NYN 21                                                                               // [q,dq,ddq] nel cost terminale
#define NJ 7                                                                                 // Numero giunti Franka
#define NP 9                                                                                 // N Parametri ostacoli
int N_sfere = 2;                                                                             // Numero di sfere per approssimazione ostacoli
int N_capsule = 12;                                                                          // Numero di capsule per approssimazione robot (dipende da come segmentiamo il robot)
int N_piani = 0;                                                                             // Numero di piani per approssimazione ambiente.
int N_autocollisioni = 2;                                                                    // Numero di auto-collisioni che vogliamo considerare (es. tra 2 coppie di capsule)
const int N_DIST = N_autocollisioni + (N_capsule - 1) * N_sfere + (N_capsule - 3) * N_piani; // 2 auto-collisioni + 9 capsule * 3 sfere + 7 capsule * piano  2 e 3 capsile escluse
const int N_SH_TOT = NX + N_DIST;

// Globals
bool init_q0 = false;
bool has_velocity = false;
bool has_acceleration = false;

struct traj_struct
{
    Eigen::Matrix<double, 7, 1> pos_des;
    Eigen::Matrix<double, 7, 1> vel_des;
    Eigen::Matrix<double, 7, 1> acc_des;
} traj;

// State variables
Eigen::Matrix<double, 7, 1> q0;
Eigen::Matrix<double, 7, 1> dq0;
Eigen::Matrix<double, 7, 1> ddq0;

VectorXd q_start, dq_start, ddq_start;

// utili scelta 3
const double q_lim_upp[] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
const double q_lim_low[] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

// Limiti per [q, dq, ddq]
double lb[NX] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                 -2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61,
                 -15, -7.5, -10, -12.5, -15, -20, -20};
double ub[NX] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
                 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61,
                 15, 7.5, 10, 12.5, 15, 20, 20};

// Funzione di utilità per l'errore di predizione
double simulate_prediction(
    const VectorXd &q_start,
    const VectorXd &dq_start,
    const VectorXd &ddq_start,
    const MatrixXd &U_sequence,
    const VectorXd &q_target,
    double dt_step,
    int horizon_len)
{
    VectorXd q_corrente = q_start;
    VectorXd dq_corrente = dq_start;
    VectorXd ddq_corrente = ddq_start;
    for (int k = 0; k < horizon_len; ++k)
    {
        VectorXd u_k = U_sequence.col(k);
        q_corrente += dq_corrente * dt_step + 0.5 * ddq_corrente * std::pow(dt_step, 2) + (1.0 / 6.0) * u_k * std::pow(dt_step, 3);
        dq_corrente += ddq_corrente * dt_step + 0.5 * u_k * std::pow(dt_step, 2);
        ddq_corrente += u_k * dt_step;
    }
    return (q_corrente - q_target).norm();
}

void signal_callback_handler(int signum)
{
    cout << "Caught signal " << signum << endl;
    exit(signum);
}

// Callback per leggere POSIZIONE, VELOCITÀ
// void jointsCallback(const sensor_msgs::JointStateConstPtr &msg)
// {
//     q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());

//     if (msg->velocity.size() == 7)
//     {
//         dq0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
//         has_velocity = true;
//     }
//     else
//     {
//         dq0.setZero();
//         has_velocity = true;
//     }

//     // if (msg->acceleration.size() == 7)
//     // {
//     //     ddq0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->acceleration).data());
//     //     has_acceleration = true;
//     // }
//     // else
//     // {
//     //     ddq0.setZero();
//     //     has_acceleration = true;
//     // }

//     // NOTA: Il JointState non ha l'accelerazione,
//     // dovremmo stimarla o avere un sensore dedicato

//     init_q0 = true;
// }

void jointsCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->position.data());
    dq0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->velocity.data());
    // if (msg->effort.size() == 7)
    //     ddq0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->effort.data());
    init_q0 = true;
}

// Interpolatore Classico (Scelta 1)
void interpolator_pos(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t)
{
    traj.pos_des << pos_i + (pos_i - pos_f) * (15 * pow((t / tf), 4) - 6 * pow((t / tf), 5) - 10 * pow((t / tf), 3));
    traj.vel_des << (pos_i - pos_f) * (60 * (pow(t, 3) / pow(tf, 4)) - 30 * (pow(t, 4) / pow(tf, 5)) - 30 * (pow(t, 2) / pow(tf, 3)));
    traj.acc_des << (pos_i - pos_f) * (180 * (pow(t, 2) / pow(tf, 4)) - 120 * (pow(t, 3) / pow(tf, 5)) - 60 * (t / pow(tf, 3)));
}

// Funzione di utilità per salvare la sequenza di controllo U in un file CSV (debugging e analisi)
void saveUSequenceToCSV(std::ofstream &file, double t_curr, const Eigen::MatrixXd &U_seq)
{
    if (file.is_open())
    {
        // Formato: tempo_corrente, u_0_j0, u_0_j1, ..., u_N_j6
        file << t_curr;
        for (int r = 0; r < U_seq.cols(); ++r)
        { // Per ogni nodo dell'orizzonte (N_HORIZON)
            for (int j = 0; j < U_seq.rows(); ++j)
            { // Per ogni giunto (NJ)
                file << "," << U_seq(j, r);
            }
        }
        file << "\n";
    }
}

int main(int argc, char **argv)
{
    thunder_frankino robot;
    ros::init(argc, argv, "menu");
    ros::NodeHandle node_handle;
    robot.load_conf(conf_file);
    // ros::Subscriber sub_joints = node_handle.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1, &jointsCallback);
    ros::Subscriber sub_joints = node_handle.subscribe<sensor_msgs::JointState>("/mpc/filtered_joint_state", 1, &jointsCallback);
    ros::Publisher pub_mpc_solution = node_handle.advertise<panda_controllers::MpcSolution>("/mpc_solution", 1);
    ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1);
    // ros::Publisher pub_jerk_cmd = node_handle.advertise<sensor_msgs::JointState>("/mpc_jerk_command", 1000);

    sensor_msgs::JointState traj_msg;
    // panda_controllers::MpcSolution mpc_msg;

    // Setup loop rates
    double loop_hz = 1000.0; // Per scelta 1-2-3
    double loop_mpc = 30.0;
    ros::Rate loop_rate(loop_hz);
    ros::Rate mpc_rate(loop_mpc);

    srand(time(NULL));
    double tf;
    Eigen::Matrix<double, 7, 1> qf;
    Eigen::Matrix<double, 7, 1> dqf;
    Eigen::Matrix<double, 7, 1> ddqf;
    double x_stop[NX];

    signal(SIGINT, signal_callback_handler);

    ros::Time t_init;
    init_q0 = false;
    double t = 0;
    int choice;
    int yaml = 0;

    // Variabili per controllo MPC
    VectorXd jerk_opt = VectorXd::Zero(NJ);              // Jerk ottimo da MPC
    MatrixXd U_sequence = MatrixXd::Zero(NJ, N_HORIZON); // Sequenza di jerk ottima da MPC
    double solve_time_ms = 0.0;                          // Tempo di soluzione MPC

    while (ros::ok())
    {
        cout << "\n------------------------------------------------" << endl;
        cout << "choice: (1: Min-Jerk Standard, 2: Init, 3: Random, 6: MPC Acados, 7: Init + MPC Acados)" << endl;
        if (yaml == 1)
        {
            choice = 5;
        }
        else
        {
            cin >> choice;
        }

        // --- GESTIONE INPUT ---
        if (choice == 1)
        {
            ros::spinOnce();
            tf = 5.0;
            qf << -0.157, -0.504, -0.623, -2.258, -0.332, 1.637, -1.728;
            dqf.setZero();
            ddqf.setZero();
        }
        else if (choice == 2)
        {
            ros::spinOnce();
            std::vector<double> qf_array;
            if (!node_handle.getParam("/menu/Q0_INIT", qf_array))
                ROS_ERROR("Failed to get parameter from server.");
            qf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qf_array.data(), qf_array.size());
            dqf.setZero();
            ddqf.setZero();
            choice = 1;
            tf = 3.0;
        }
        else if (choice == 3)
        {
            ros::spinOnce();
            for (int i = 0; i < 7; i++)
            {
                double q_low = q_lim_low[i];
                double q_upp = q_lim_upp[i];
                qf(i) = q_low + (float(rand()) / RAND_MAX) * (q_upp - q_low);
            }
            dqf.setZero();
            ddqf.setZero();
            choice = 1;
            tf = 3.0;
        }
        else if (choice == 4)
        {
            cout << "-not implemented yet-" << endl;
        }
        else if (choice == 7) // Scelta 7: Scelta 2 + MPC
        {
            ros::spinOnce();
            loop_rate.sleep();
            cout << "Inserisci l'orizzonte temporale desiderato: ";
            cin >> tf; // Leggi orizzonte temporale desiderato
            cout << "Orizzonte temporale: " << tf << endl;
            std::vector<double> qf_array;
            if (!node_handle.getParam("/menu/Q0_INIT", qf_array))
                ROS_ERROR("Failed to get parameter from server.");
            qf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qf_array.data(), qf_array.size());
            dqf.setZero();
            ddqf.setZero();
            ddq0.setZero();
        }
        else if (choice == 6)
        {
            // Stato iniziale
            ros::spinOnce();
            loop_rate.sleep();

            cout << "Inserisci l'orizzonte temporale desiderato: ";
            cin >> tf; // orizzonte temporale desiderato
            cout << "Orizzonte temporale: " << tf << endl;

            ddq0.setZero(); // Setto a 0 l'accelerazione iniziale, dato che so che partirò da fermo quando scelgo

            // --- DEFINIZIONE TARGET CARTESIANO E CLIK ---
            Vector3d p_des(0.33, -0.35, 0.45);
            Vector3d v_des(0.0, 0.0, -0.5);

            cout << "Posizione target cartesiana: " << p_des.transpose() << endl;
            cout << "Velocità target cartesiana: " << v_des.transpose() << endl;

            // Calcola target giunti usando CLIK di secondo ordine
            JointStateTarget target_state = KinematicsUtils::computeFullTarget(robot, p_des, v_des, q0, dq0, ddq0);

            qf = target_state.q;
            dqf = target_state.dq;
            ddqf = target_state.ddq;

            // qf << -0.14724, -0.526, -0.565, -2.1099, -0.312, 1.557, -1.733;
            // dqf.setZero();
            // ddqf.setZero();
        }

        // Attesa primo messaggio joint states
        while (!init_q0 && ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
            cout << "Waiting for joint states..." << endl;
        }
        t_init = ros::Time::now();
        t = 0;

        // =================================================================================
        // SCELTA 6: MPC ACADOS
        // =================================================================================
        if (choice == 6 || choice == 7)
        {
            cout << ">>> Initializing ACADOS MPC (Jerk version)..." << endl;

            // ros::spinOnce();

            q_start = q0;
            dq_start = dq0;
            ddq_start = ddq0;

            cout << "Start q: " << q_start.transpose() << endl;
            cout << "Start dq: " << dq_start.transpose() << endl;
            cout << "Start ddq: " << ddq_start.transpose() << endl;
            cout << "\n"
                 << endl;
            cout << "Target q: " << qf.transpose() << endl;
            cout << "Target dq: " << dqf.transpose() << endl;
            cout << "Target ddq: " << ddqf.transpose() << endl;

            // --- DEFINIZIONE PATH PERSONALIZZATA ---
            std::string custom_path = "/home/frankino/Tesi/thunder_MPC_Acados/src/panda_controllers/esperimenti/u_sequence_log.csv";

            std::ofstream u_seq_file;
            u_seq_file.open(custom_path, std::ios::out);

            if (!u_seq_file.is_open())
            {
                ROS_ERROR("ERRORE: Impossibile creare il file in %s. Controlla che la cartella esista!", custom_path.c_str());
            }
            else
            {
                ROS_INFO("File di log creato correttamente in: %s", custom_path.c_str());
            }

            // Scrittura Header
            u_seq_file << "time";
            for (int i = 0; i < N_HORIZON; ++i)
            {
                for (int j = 0; j < NJ; ++j)
                {
                    u_seq_file << ",node" << i << "_j" << j;
                }
            }
            u_seq_file << "\n";

            // 1. Setup Acados Solver
            frankino_tracking_mpc_solver_capsule *capsule = frankino_tracking_mpc_acados_create_capsule();
            if (frankino_tracking_mpc_acados_create(capsule) != 0)
            {
                cerr << "Error creating Acados solver!" << endl;
                continue;
            }

            // // Setup Acados Simulator
            // frankino_tracking_mpc_sim_solver_capsule *sim_capsule = frankino_tracking_mpc_acados_sim_solver_create_capsule();
            // int status_sim = frankino_tracking_mpc_acados_sim_create(sim_capsule);
            // if (status_sim != 0)
            // {
            //     cerr << "Error creating Acados simulator! Status: " << status_sim << endl;
            //     frankino_tracking_mpc_acados_free(capsule);
            //     frankino_tracking_mpc_acados_free_capsule(capsule);
            //     continue;
            // }

            auto nlp_config = frankino_tracking_mpc_acados_get_nlp_config(capsule);
            auto nlp_dims = frankino_tracking_mpc_acados_get_nlp_dims(capsule);
            auto nlp_in = frankino_tracking_mpc_acados_get_nlp_in(capsule);
            auto nlp_out = frankino_tracking_mpc_acados_get_nlp_out(capsule);

            // auto sim_config = frankino_tracking_mpc_acados_get_sim_config(sim_capsule);
            // auto sim_in = frankino_tracking_mpc_acados_get_sim_in(sim_capsule);
            // auto sim_out = frankino_tracking_mpc_acados_get_sim_out(sim_capsule);
            // auto sim_dims = frankino_tracking_mpc_acados_get_sim_dims(sim_capsule);

            // Target per vincoli terminali
            double x_target_ub[NX], x_target_lb[NX], x_target_ub_terminal[NX], x_target_lb_terminal[NX];
            double tolerance = 1e-6;
            double eps = 1e-6;
            for (int i = 0; i < NJ; i++)
            {
                x_target_ub[i] = qf(i) + eps;
                x_target_ub_terminal[i] = qf(i) + tolerance;
                x_target_lb[i] = qf(i) - eps;
                x_target_lb_terminal[i] = qf(i) - tolerance;
                x_target_ub[NJ + i] = dqf(i) + eps;
                x_target_ub_terminal[NJ + i] = dqf(i) + tolerance;
                x_target_lb[NJ + i] = dqf(i) - eps;
                x_target_lb_terminal[NJ + i] = dqf(i) - tolerance;
                x_target_ub[2 * NJ + i] = ddqf(i) + eps;
                x_target_ub_terminal[2 * NJ + i] = ddqf(i) + tolerance;
                x_target_lb[2 * NJ + i] = ddqf(i) - eps;
                x_target_lb_terminal[2 * NJ + i] = ddqf(i) - tolerance;
            }
            // Setup MinJerk Planner
            MinJerkTrajectory planner;

            // Inizializza planner con stato corrente e target
            planner.init(q_start, qf, dq_start, dqf, ddq_start, ddqf, t, tf);

            // Variabili per controllo sample-and-hold
            double t_hor_lim = N_HORIZON / loop_mpc; // Orizzonte minimo
            bool first_mpc_call = true;
            VectorXd current_jerk = VectorXd::Zero(NJ); // Jerk corrente da applicare

            // // Parametri per ostacoli (da aggiornare ad ogni loop)
            // double p_values[NP] = {tf, 110.11, -0.35, 0.53, 0.05,
            //                        0.31, 0.2, 0.5, 0.05,
            //                        //    1000.0, 1000.0, 900.0, 0.05,
            //                        0.0, 0.0, 0.0,
            //                        0.0, 0.0, 1.0};

            // Parametri per ostacoli (da aggiornare ad ogni loop)
            double p_values[NP] = {tf, 0.11, -0.35, 0.53, 0.05,
                                   0.31, 0.2, 0.5, 0.05};

            std::vector<std::string> constraint_names;

            // 1. Aggiungi le 8 auto-collisioni
            int N_capsule = 12; // Numero totale di capsule
            constraint_names.push_back("Self: Cap 1 vs Cap 7");
            constraint_names.push_back("Self: Cap 1 vs Cap 8");
            // constraint_names.push_back("Self: Cap 1 vs Cap 10");
            // constraint_names.push_back("Self: Cap 1 vs Cap 11");
            // constraint_names.push_back("Self: Cap 2 vs Cap 10");
            // constraint_names.push_back("Self: Cap 2 vs Cap 11");
            // constraint_names.push_back("Self: Cap 3 vs Cap 10");
            // constraint_names.push_back("Self: Cap 3 vs Cap 11");

            // 2. Aggiungi Sfera 1
            for (int i = 1; i <= N_capsule - 1; i++)
                constraint_names.push_back("Sfera 1 vs Cap " + std::to_string(i));

            // 3. Aggiungi Sfera 2
            for (int i = 1; i <= N_capsule - 1; i++)
                constraint_names.push_back("Sfera 2 vs Cap " + std::to_string(i));

            // // 4. Aggiungi Sfera 3
            // for (int i = 1; i <= N_capsule - 1; i++)
            //     constraint_names.push_back("Sfera 3 vs Cap " + std::to_string(i));

            // // 5. Aggiungi Piano (Capsule da 3)
            // for (int i = 3; i <= N_capsule - 1; i++)
            //     constraint_names.push_back("Piano vs Cap " + std::to_string(i));

            // Loop di controllo MPC
            while (t <= tf + eps && ros::ok())
            {
                ros::spinOnce(); // Aggiorna q0, dq0 dal robot reale dovrei aggiornare anche ddq0 se avessi un sensore o stima

                std::cout << "Stato: " << "q = " << q0.transpose() << ", dq = " << dq0.transpose() << ", ddq = " << ddq0.transpose() << std::endl;

                double time_to_go = tf - t;
                double Tf = max(time_to_go, t_hor_lim);
                // Tf = 1.0; // Per testare con orizzonte fisso a 1 secondo (debug)
                double dt_mpc_node = Tf / N_HORIZON;
                p_values[0] = Tf; // Aggiorna l'orizzonte temporale per il solver
                // if (t < 2.0)
                // {
                //     // L'ostacolo parte da x = 1.0 e "scivola"
                //     double progresso = t / 1.0; // va da 0 a 1
                //     p_values[1] = 1.0 - (1.0 - 0.11) * progresso;
                // }
                // else if (t >= 2.0 && t < 2.5)
                // {
                //     p_values[1] = 0.11; // Raggiunge la posizione finale
                // }
                // else
                // {
                //     p_values[1] = 10.11; // Se ne va
                // }

                // //--CASO 2: Movimento avanti-indietro lungo x
                // // Definiamo i parametri del movimento
                // double t_inizio_movimento = 0.0;
                // double t_fine_movimento = 5.0; // Durata del transito (es. 4 secondi)
                // double x_start = 0.3;
                // double x_end = -0.3;

                // if (t>= t_inizio_movimento && t <= t_fine_movimento)
                // {
                //     // Calcoliamo il progresso normalizzato (da 0 a 1)
                //     double progresso = (t - t_inizio_movimento) / (t_fine_movimento - t_inizio_movimento);

                //     // Interpolazione lineare tra x_start e x_end
                //     p_values[1] = x_start + (x_end - x_start) * progresso;
                // }
                // else if (t > t_fine_movimento)
                // {
                //     p_values[1] = x_start; // Rimane ferma nel punto di arrivo
                // }
                // else
                // {
                //     p_values[1] = x_start; // Ferma al punto di partenza prima dell'inizio
                // }
                // --- FEEDBACK STATO CORRENTE ---
                double x0[NX];
                for (int i = 0; i < NJ; i++)
                {
                    x0[i] = q0(i);
                    x0[NJ + i] = dq0(i);
                    x0[2 * NJ + i] = ddq0(i);
                }

                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0);

                int NODO_corrente = N_HORIZON;

                // --- B. LOGICA MPC  ---
                if (time_to_go >= t_hor_lim)
                {
                    // Ricalcola traiettoria MinJerk dal punto corrente
                    planner.init(q0, qf, dq0, dqf, ddq0, ddqf, t, tf);

                    // Genera reference e warm start
                    for (int i = 0; i <= N_HORIZON; i++)
                    {
                        double ti = t + i * dt_mpc_node;
                        auto s = planner.evaluate(ti);

                        // Stage cost reference [q_des, dq_des, ddq_des, jerk_des]
                        double yref_stage[NY];
                        for (int j = 0; j < NJ; j++)
                        {
                            yref_stage[j] = s.pos(j);      // Target posizione
                            yref_stage[NJ + j] = s.vel(j); // Target velocità
                            yref_stage[2 * NJ + j] = 0.0;  // Target accelerazione
                            yref_stage[3 * NJ + j] = 0.0;  // Target jerk qualsiasi tanto W_u = 0
                        }

                        if (i < N_HORIZON)
                        {
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
                        }
                        else
                        {
                            // Terminal cost reference
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_HORIZON, "yref", x_target_ub);
                            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "lbx", x_target_lb);
                            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "ubx", x_target_ub);
                        }

                        // Warm start
                        if (!first_mpc_call && i < N_HORIZON)
                        {
                            double x_prev[NX], u_prev[NU];
                            // GET
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_prev);
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", u_prev);
                            // SET
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_prev);
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_prev);
                        }
                        else
                        {
                            first_mpc_call = false;
                            // Prima chiamata: MinJerk come initial guess
                            double x_guess[NX];
                            for (int j = 0; j < NJ; j++)
                            {
                                x_guess[j] = s.pos(j);
                                x_guess[NJ + j] = s.vel(j);
                                x_guess[2 * NJ + j] = s.acc(j);
                            }
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_guess);
                            if (i < N_HORIZON)
                                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", s.jerk.data());
                        }

                        // Set parametri
                        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "parameter_values", p_values);
                    }
                }
                else
                {
                    int NODO = (int)std::round(time_to_go / dt_mpc_node);
                    std::cout << "NODO = " << NODO << std::endl;
                    NODO_corrente = NODO;

                    for (int j = 0; j < NJ; j++)
                    {
                        x_stop[j] = qf(j);
                        x_stop[NJ + j] = dqf(j);
                        x_stop[2 * NJ + j] = 0.0;
                    }
                    double u_zero[NU] = {0.0};

                    // --- PREPARAZIONE ARRAY SLACK DINAMICI ---

                    double Zl_normal[N_SH_TOT], zl_normal[N_SH_TOT];
                    double Zu_normal[N_SH_TOT], zu_normal[N_SH_TOT];
                    double Zl_target[N_SH_TOT], zl_target[N_SH_TOT];
                    double Zu_target[N_SH_TOT], zu_target[N_SH_TOT];

                    for (int s = 0; s < NX; s++)
                    {
                        // Nodi normali: limiti fisici (q, dq, ddq) INVALICABILI
                        Zl_normal[s] = 1e3;
                        zl_normal[s] = 1e3;
                        Zu_normal[s] = 1e3;
                        zu_normal[s] = 1e3;

                        // Nodo target: slack stato tolleranti ma punitivi
                        Zl_target[s] = 1e3;
                        zl_target[s] = 1e4;
                        Zu_target[s] = 1e3;
                        zu_target[s] = 1e4;
                    }
                    for (int s = 21; s < N_SH_TOT; s++)
                    {
                        // LIMITI INFERIORI (Distanze minime di sicurezza):
                        Zl_normal[s] = 1e7;
                        zl_normal[s] = 1e6;
                        Zl_target[s] = 1e7;
                        zl_target[s] = 1e6;

                        // LIMITI SUPERIORI (Distanze massime)
                        // Non c'è limite a quanto il robot possa allontanarsi da un ostacolo!
                        Zu_normal[s] = 0.0;
                        zu_normal[s] = 0.0;
                        Zu_target[s] = 0.0;
                        zu_target[s] = 0.0;
                    }
                    for (int i = 0; i <= N_HORIZON; i++)
                    {
                        // --- 1. WARM START ---
                        if (i < NODO)
                        {
                            double x_prev[NX], u_prev[NU];
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i + 1, "x", x_prev);
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i + 1, "u", u_prev);
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_prev);
                            if (i < N_HORIZON)
                                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_prev);
                        }
                        else
                        {
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_stop);
                            if (i < N_HORIZON)
                                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_zero);
                        }

                        // --- 2. VINCOLI E RIFERIMENTI ---
                        if (i == NODO)
                        {
                            // 1. PROTEZIONE NODO 0: Imponiamo i limiti rigidi al target SOLO se ci troviamo nel futuro.
                            // Se i == 0, lasciamo i limiti intatti (già vincolati a x0 reale) per evitare Infeasible QP.
                            if (i > 0)
                            {
                                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", x_target_lb_terminal);
                                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", x_target_ub_terminal);
                            }

                            // 2. SLACK DINAMICI: Riduciamo la penalità da 10000 a 100 negli ultimi 3 nodi (circa 100-150ms).
                            // Questo elimina il "colpo di frusta" finale, permettendo un assestamento fluido.
                            double current_slack_penalty = (NODO <= 3) ? 1e2 : 1e4;
                            double Zl_dyn[N_SH_TOT], zl_dyn[N_SH_TOT], Zu_dyn[N_SH_TOT], zu_dyn[N_SH_TOT];

                            for (int s = 0; s < NX; s++)
                            {
                                Zl_dyn[s] = Zl_target[s];
                                zl_dyn[s] = current_slack_penalty; // Usa la penalità dinamica
                                Zu_dyn[s] = Zu_target[s];
                                zu_dyn[s] = current_slack_penalty; // Usa la penalità dinamica
                            }
                            for (int s = 21; s < N_SH_TOT; s++)
                            {
                                // Gli ostacoli rimangono intoccabili e rigidissimi
                                Zl_dyn[s] = Zl_target[s];
                                zl_dyn[s] = zl_target[s];
                                Zu_dyn[s] = Zu_target[s];
                                zu_dyn[s] = zu_target[s];
                            }

                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl_dyn);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl_dyn);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu_dyn);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu_dyn);

                            // 3. REFERENCE (yref): Fondamentale specialmente quando NODO == 0.
                            // Dicendo alla funzione di costo dov'è il target, l'MPC continuerà a inseguirlo
                            // anche se abbiamo tolto i limiti rigidi lbx/ubx al Nodo 0.
                            if (i < N_HORIZON)
                            {
                                double yref_stage[NY] = {0.0};
                                for (int j = 0; j < NJ; j++)
                                {
                                    yref_stage[j] = qf(j);
                                    yref_stage[NJ + j] = dqf(j);
                                    yref_stage[2 * NJ + j] = ddqf(j);
                                    yref_stage[3 * NJ + j] = 0.0;
                                }
                                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
                            }
                            else
                            {
                                double yref_e[NYN] = {0.0}; // NYN = 21 nel terminal cost
                                for (int j = 0; j < NJ; j++)
                                {
                                    yref_e[j] = qf(j);
                                    yref_e[NJ + j] = dqf(j);
                                    yref_e[2 * NJ + j] = ddqf(j);
                                }
                                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
                            }
                        }
                        else if (i > NODO)
                        {
                            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lb);
                            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ub);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu_normal);

                            // Distinzione Stage/Terminale per la size di yref
                            if (i < N_HORIZON)
                            {
                                double yref_stop[NY] = {0.0};
                                for (int j = 0; j < NJ; j++)
                                {
                                    yref_stop[j] = qf(j) + dqf(j) * dt_mpc_node * (i - NODO) + 0.5 * ddqf(j) * pow(dt_mpc_node * (i - NODO), 2);
                                    yref_stop[NJ + j] = dqf(j) + ddqf(j) * dt_mpc_node * (i - NODO);
                                    yref_stop[2 * NJ + j] = ddqf(j);
                                    yref_stop[3 * NJ + j] = 0.0; // Jerk target a 0 per i nodi dopo il target, tanto W_u = 0
                                }
                                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stop);
                            }
                            else
                            {
                                double yref_e[NYN] = {0.0};
                                for (int j = 0; j < NJ; j++)
                                {
                                    yref_e[j] = qf(j) + dqf(j) * dt_mpc_node * (i - NODO) + 0.5 * ddqf(j) * pow(dt_mpc_node * (i - NODO), 2);
                                    yref_e[NJ + j] = dqf(j) + ddqf(j) * dt_mpc_node * (i - NODO);
                                    yref_e[2 * NJ + j] = ddqf(j);
                                }
                                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
                            }
                        }
                        else // i < NODO
                        {
                            // Applica multe altissime per simulare vincoli HARD sugli stati
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu_normal);
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu_normal);

                            double ti = t + i * dt_mpc_node;
                            auto s = planner.evaluate(std::min(ti, tf));
                            double yref_stage[NY] = {0.0};
                            for (int j = 0; j < NJ; j++)
                            {
                                yref_stage[j] = s.pos(j);
                                yref_stage[NJ + j] = s.vel(j);
                                yref_stage[2 * NJ + j] = 0.0;
                                yref_stage[3 * NJ + j] = 0.0;
                            }
                            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
                        }

                        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "parameter_values", p_values);
                    }
                }

                // --- SOLVE MPC ---
                auto start_solve = chrono::high_resolution_clock::now();
                int status = frankino_tracking_mpc_acados_solve(capsule);
                auto end_solve = chrono::high_resolution_clock::now();
                solve_time_ms = chrono::duration_cast<chrono::microseconds>(end_solve - start_solve).count() / 1000.0;

                if (status != 0 && status != 2)
                {
                    ROS_WARN_THROTTLE(1.0, "Acados status: %d", status);
                    break;
                }

                // Controlla slack su tutti i nodi
                for (int node = 1; node <= N_HORIZON; node++)
                {
                    double sl_check[N_SH_TOT];
                    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, node, "sl", sl_check);
                    double max_s = 0.0;
                    for (int idx = 0; idx < N_DIST; idx++)
                        max_s = std::max(max_s, sl_check[NX + idx]);
                    if (max_s > 1e-4)
                        printf("  >> nodo %d: slack_obs=%.5f\n", node, max_s);
                }
                printf("t=%.3f | dt_node=%.4f | Tf=%.3f\n", t, dt_mpc_node, Tf);

                // --- F. SUPERVISORE: CONTROLLO VIOLAZIONE OSTACOLI TRAMITE SLACK ---
                bool collision_risk = false;
                if (collision_risk==true)
                {
                    double slacks[N_SH_TOT];
                    bool collision_detected = false;
                    const double tolleranza_slack = 1e-2; // 1 cm

                    for (int node = 1; node <= N_HORIZON; node++)
                    {
                        // Salta i nodi che seguono il nodo target: i vincoli lì sono stati ridefiniti
                        // e non rappresentano più distanze di sicurezza reali
                        if (NODO_corrente < N_HORIZON && node > NODO_corrente)
                            continue;

                        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, node, "sl", slacks);

                        for (int idx = 0; idx < N_DIST; idx++)
                        {
                            // I primi 21 slot sono gli slack di stato (q, dq, ddq)
                            // Dal 21 in poi ci sono gli slack delle distanze ostacoli
                            double slack_val = slacks[NX + idx];
                            if (slack_val > tolleranza_slack)
                            {
                                collision_detected = true;
                                ROS_ERROR("\n>>> ALLARME PREDIZIONE! Rischio collisione al nodo %d (t=%.3f s)", node, t);

                                if (idx < (int)constraint_names.size())
                                {
                                    ROS_WARN(">>> CAUSA: %s (violazione di %.4f m)",
                                             constraint_names[idx].c_str(), slack_val);
                                }
                                else
                                {
                                    ROS_WARN(">>> CAUSA: Vincolo sconosciuto (indice %d, violazione di %.4f m)",
                                             idx, slack_val);
                                }
                                break;
                            }
                        }
                        if (collision_detected)
                            break;
                    }

                    if (collision_detected)
                    {
                        ROS_FATAL("\n>>> TRAIETTORIA NON SICURA: Fermo il robot a t = %.3f s.\n", t);
                        break;
                    }
                }

                // Recupera jerk ottimo
                double u0_temp[NU];
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0_temp);
                for (int j = 0; j < NJ; j++)
                {
                    current_jerk(j) = u0_temp[j];
                }

                // Salva sequenza per prediction error
                for (int r = 0; r < N_HORIZON; r++)
                {
                    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, r, "u", u0_temp);
                    for (int j = 0; j < NJ; j++)
                    {
                        U_sequence(j, r) = u0_temp[j];
                    }
                }

                // SALVATAGGIO SU FILE
                saveUSequenceToCSV(u_seq_file, t, U_sequence);

                // --- CREAZIONE E PUBBLICAZIONE MESSAGGIO MPC_SOLUTION ---
                panda_controllers::MpcSolution mpc_msg;
                mpc_msg.header.stamp = ros::Time::now();
                mpc_msg.dt_mpc = dt_mpc_node; // Il dt dell'orizzonte calcolato

                // Riempio lo stato ottimo attuale (quello da cui parte l'integratore)
                for (int i = 0; i < 7; i++)
                {
                    mpc_msg.q_start.push_back(q0(i));
                    mpc_msg.dq_start.push_back(dq0(i));
                    mpc_msg.ddq_start.push_back(ddq0(i));
                    mpc_msg.jerk.push_back(current_jerk(i));
                }

                // Pubblico la soluzione per il nodo integratore
                pub_mpc_solution.publish(mpc_msg);

                // Calcola prediction error
                double pred_error = simulate_prediction(q0, dq0, ddq0, U_sequence, qf, dt_mpc_node, N_HORIZON);

                cout << ">>> MPC solve time: " << solve_time_ms << " ms, pred_error: " << pred_error << endl;

                double t_now = (ros::Time::now() - t_init).toSec();

                // INTEGRAZIONE DI EULERO PER RIMEDIARE ALLA MANCANZA DI FEEDBACK REALE SULL'ACCELERAZIONE (ddq0)
                // current_jerk mantiene l'ultimo valore valido grazie al sample-and-hold
                ddq0 += current_jerk * 1.0 / loop_mpc;
                loop_rate.sleep(); // preferisco usare questo sleep per non tardare ancor più la pubblicazione del messaggio MPC_SOLUTION, dato che ci mette già solve di per se a risolvere, integrare e pubblicare tutto entro i 40 ms
                // mpc_rate.sleep(); // FREQUENZA DI CONTROLLO MPC
                t = (ros::Time::now() - t_init).toSec();
            }

            cout << ">>> MPC Trajectory Finished." << endl;
            // pred_file.close();

            u_seq_file.close();

            // Cleanup Acados
            frankino_tracking_mpc_acados_free(capsule);
            frankino_tracking_mpc_acados_free_capsule(capsule);
            // frankino_tracking_mpc_acados_sim_free(sim_capsule);
            // frankino_tracking_mpc_acados_sim_solver_free_capsule(sim_capsule);
        }

        // =================================================================================
        // SCELTA 1-5: LOGICA STANDARD ESISTENTE
        // =================================================================================
        else
        {
            ros::spinOnce(); // Aggiorna q0, dq0 dal robot reale

            while (t <= tf && init_q0 && ros::ok())
            {
                if (choice == 1)
                {
                    interpolator_pos(q0, qf, tf, t);
                }
                else
                {
                    break;
                }

                traj_msg.header.stamp = ros::Time::now();
                traj_msg.position = {traj.pos_des[0], traj.pos_des[1], traj.pos_des[2], traj.pos_des[3], traj.pos_des[4], traj.pos_des[5], traj.pos_des[6]};
                traj_msg.velocity = {traj.vel_des[0], traj.vel_des[1], traj.vel_des[2], traj.vel_des[3], traj.vel_des[4], traj.vel_des[5], traj.vel_des[6]};
                traj_msg.effort = {traj.acc_des[0], traj.acc_des[1], traj.acc_des[2], traj.acc_des[3], traj.acc_des[4], traj.acc_des[5], traj.acc_des[6]};

                pub_cmd.publish(traj_msg);
                loop_rate.sleep();
                t = (ros::Time::now() - t_init).toSec();
            }
        }
    }
    return 0;
}