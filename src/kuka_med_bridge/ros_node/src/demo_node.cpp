#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────────
//  Control mode values (mirror of ControlType enum in RobotData.hpp)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int MODE_JOINT_PTP          = 0;
static constexpr int MODE_CARTESIAN_PTP      = 1;
static constexpr int MODE_JOINT_IMPEDANCE    = 2;
static constexpr int MODE_CARTESIAN_IMPEDANCE= 3;
static constexpr int MODE_PTP_JOINT_IMP      = 4;   // PTP_AND_JOINT_IMPEDANCE
static constexpr int MODE_PTP_CART_IMP       = 5;   // PTP_AND_CART_IMPEDANCE

// ─────────────────────────────────────────────────────────────────────────────
//  Shared robot state
// ─────────────────────────────────────────────────────────────────────────────
std::array<double, 7> currentJoints  = {0};
std::array<double, 6> currentCart    = {0};
std::array<double, 6> currentWrench  = {0};
std::mutex            stateMutex;

// ─────────────────────────────────────────────────────────────────────────────
//  Publishers
// ─────────────────────────────────────────────────────────────────────────────
ros::Publisher pub_mode;
ros::Publisher pub_joint_ptp;
ros::Publisher pub_cart_ptp;
ros::Publisher pub_joint_stiff;
ros::Publisher pub_joint_damp;
ros::Publisher pub_cart_stiff;
ros::Publisher pub_cart_damp;
ros::Publisher pub_ptp_imp;

// ─────────────────────────────────────────────────────────────────────────────
//  Subscribers callbacks
// ─────────────────────────────────────────────────────────────────────────────
void cbJointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() < 7) return;
    std::lock_guard<std::mutex> lock(stateMutex);
    for (int i = 0; i < 7; i++) currentJoints[i] = msg->position[i];
}

void cbCartPose(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 6) return;
    std::lock_guard<std::mutex> lock(stateMutex);
    for (int i = 0; i < 6; i++) currentCart[i] = msg->data[i];
}

void cbWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    currentWrench[0] = msg->wrench.force.x;
    currentWrench[1] = msg->wrench.force.y;
    currentWrench[2] = msg->wrench.force.z;
    currentWrench[3] = msg->wrench.torque.x;
    currentWrench[4] = msg->wrench.torque.y;
    currentWrench[5] = msg->wrench.torque.z;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Send helpers
// ─────────────────────────────────────────────────────────────────────────────
void sendControlMode(int mode)
{
    std_msgs::Int32 msg;
    msg.data = mode;
    pub_mode.publish(msg);
}

void sendJointPtp(const std::array<double, 7>& q, double vel = 0.1)
{
    std_msgs::Float64MultiArray msg;
    msg.data.assign(q.begin(), q.end());
    msg.data.push_back(vel);
    pub_joint_ptp.publish(msg);
}

void sendCartPtp(const std::array<double, 6>& p, double vel = 0.1)
{
    std_msgs::Float64MultiArray msg;
    msg.data.assign(p.begin(), p.end());
    msg.data.push_back(vel);
    pub_cart_ptp.publish(msg);
}

void sendJointStiffness(const std::array<double, 7>& k)
{
    std_msgs::Float64MultiArray msg;
    msg.data.assign(k.begin(), k.end());
    pub_joint_stiff.publish(msg);
}

void sendJointDamping(const std::array<double, 7>& d)
{
    std_msgs::Float64MultiArray msg;
    msg.data.assign(d.begin(), d.end());
    pub_joint_damp.publish(msg);
}

// /robot/cmd/ptp_with_impedance
// Layout: [motionType, impedanceType, target(7 or 6), velocity, stiffness(7 or 6), damping(7 or 6)]
void sendPtpWithJointImpedance(
    const std::array<double, 7>& q, double vel,
    const std::array<double, 7>& stiff,
    const std::array<double, 7>& damp)
{
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(static_cast<double>(MODE_JOINT_PTP));        // motionType
    msg.data.push_back(static_cast<double>(MODE_JOINT_IMPEDANCE));  // impedanceType
    for (auto v : q)     msg.data.push_back(v);
    msg.data.push_back(vel);
    for (auto v : stiff) msg.data.push_back(v);
    for (auto v : damp)  msg.data.push_back(v);
    pub_ptp_imp.publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print helpers
// ─────────────────────────────────────────────────────────────────────────────
void printJoints()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Joint positions [rad]: ";
    for (int i = 0; i < 7; i++) std::cout << currentJoints[i] << (i<6 ? "  " : "\n");
}

void printCart()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Cartesian pose  [mm / deg]:\n";
    std::cout << "    x=" << currentCart[0] << "  y=" << currentCart[1]
              << "  z=" << currentCart[2] << "\n";
    std::cout << "    A=" << currentCart[3] << "  B=" << currentCart[4]
              << "  C=" << currentCart[5] << "\n";
}

void printWrench()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  External wrench [N / Nm]:\n";
    std::cout << "    Fx=" << currentWrench[0] << "  Fy=" << currentWrench[1]
              << "  Fz=" << currentWrench[2] << "\n";
    std::cout << "    Tx=" << currentWrench[3] << "  Ty=" << currentWrench[4]
              << "  Tz=" << currentWrench[5] << "\n";
}

void waitEnter(const std::string& prompt = "  [INVIO per continuare]")
{
    std::cout << prompt << std::endl;
    std::string s;
    std::getline(std::cin, s);
}

// ─────────────────────────────────────────────────────────────────────────────
//  DEMO STEPS
// ─────────────────────────────────────────────────────────────────────────────

// ── Step 1 — Joint PTP ───────────────────────────────────────────────────────
void demoJointPtp()
{
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout <<   "║  DEMO 1 — Joint PTP                              ║\n";
    std::cout <<   "╚══════════════════════════════════════════════════╝\n";
    std::cout << "  Il robot si muoverà verso una configurazione di home\n"
                 "  con moto PTP in spazio giunto al 10% della velocità.\n\n";
    printJoints();

    waitEnter("  Premi INVIO per muovere verso home (q = [0, -0.5, 0, 1, 0, 0.5, 0])...");

    std::array<double, 7> q_home = {0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0};
    sendJointPtp(q_home, 0.1);
    ROS_INFO("[demo] Joint PTP → home inviato.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printJoints();

    // Second pose: slight offset
    waitEnter("\n  Premi INVIO per muovere verso una seconda posa...");

    std::array<double, 7> q2 = {0.3, -0.3, 0.2, 0.8, -0.2, 0.4, 0.3};
    sendJointPtp(q2, 0.15);
    ROS_INFO("[demo] Joint PTP → posa 2 inviata.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printJoints();
}

// ── Step 2 — Cartesian PTP ──────────────────────────────────────────────────
void demoCartesianPtp()
{
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout <<   "║  DEMO 2 — Cartesian PTP                          ║\n";
    std::cout <<   "╚══════════════════════════════════════════════════╝\n";
    std::cout << "  Il robot si muoverà in spazio cartesiano.\n"
                 "  Le orientazioni sono in gradi (A, B, C = ZYX Euler).\n\n";
    printCart();

    // Read current cart pose to compute a relative offset
    std::array<double, 6> p_now;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        p_now = currentCart;
    }

    // Target: +80 mm along Z, same orientation
    std::array<double, 6> p_up = p_now;
    p_up[2] += 80.0;

    std::cout << "  Target: z attuale + 80 mm\n";
    std::cout << "    x=" << p_up[0] << "  y=" << p_up[1] << "  z=" << p_up[2]
              << "  A=" << p_up[3] << "  B=" << p_up[4] << "  C=" << p_up[5] << "\n";

    waitEnter("  Premi INVIO per inviare il comando...");
    sendCartPtp(p_up, 0.1);
    ROS_INFO("[demo] Cartesian PTP → +80 mm su Z inviato.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printCart();

    // Return to original pose
    waitEnter("\n  Premi INVIO per tornare alla posa precedente...");
    sendCartPtp(p_now, 0.1);
    ROS_INFO("[demo] Cartesian PTP → ritorno inviato.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printCart();
}

// ── Step 3 — Joint Impedance (gravity compensation) ─────────────────────────
void demoGravityComp()
{
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout <<   "║  DEMO 3 — Joint Impedance (gravity compensation) ║\n";
    std::cout <<   "╚══════════════════════════════════════════════════╝\n";
    std::cout << "  Stiffness = 0 su tutti gli assi → il robot è guidabile\n"
                 "  manualmente. Damping = 0.7 per smorzamento fluido.\n\n";

    std::array<double, 7> stiff_zero = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> damp_07    = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

    sendJointStiffness(stiff_zero);
    sendJointDamping(damp_07);
    sendControlMode(MODE_JOINT_IMPEDANCE);
    ROS_INFO("[demo] Joint Impedance (gravity comp) attivo.");

    std::cout << "  Guida il robot manualmente.\n";
    std::cout << "  Premi INVIO per stampare la posa corrente (ripeti a piacere).\n";
    std::cout << "  Digita 'q' + INVIO per uscire dalla gravity comp.\n\n";

    while (ros::ok())
    {
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "Q") break;
        printJoints();
        printCart();
        printWrench();
        std::cout << "  (INVIO = aggiorna, 'q' = esci)\n";
    }

    ROS_INFO("[demo] Uscita da gravity comp.");
}

// ── Step 4 — Joint PTP + Joint Impedance ────────────────────────────────────
void demoPtpWithImpedance()
{
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout <<   "║  DEMO 4 — Joint PTP + Joint Impedance            ║\n";
    std::cout <<   "╚══════════════════════════════════════════════════╝\n";
    std::cout << "  Il robot esegue un PTP con la modalità Joint Impedance\n"
                 "  attiva durante il moto (stiffness ridotta = moto cedevole).\n\n";
    printJoints();

    // ── Move 1: go to home with medium stiffness ──────────────────────────
    std::array<double, 7> q_home  = {0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0};
    std::array<double, 7> stiff_a = {500, 500, 500, 300, 300, 200, 100};
    std::array<double, 7> damp_a  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

    std::cout << "  Movimento 1: home con stiffness [500,500,500,300,300,200,100]\n";
    waitEnter("  Premi INVIO per avviare...");

    sendPtpWithJointImpedance(q_home, 0.1, stiff_a, damp_a);
    ROS_INFO("[demo] PTP+JointImpedance → home inviato.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printJoints();

    // ── Move 2: offset pose with lower stiffness (more compliant) ─────────
    std::array<double, 7> q2      = {0.5, -0.2, 0.0, 0.9, 0.0, 0.3, 0.0};
    std::array<double, 7> stiff_b = {200, 200, 200, 100, 100, 80, 50};
    std::array<double, 7> damp_b  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

    std::cout << "\n  Movimento 2: posa offset con stiffness ridotta [200,200,200,100,100,80,50]\n"
                 "  (il robot cederà maggiormente alle forze esterne durante il moto)\n";
    waitEnter("  Premi INVIO per avviare...");

    sendPtpWithJointImpedance(q2, 0.1, stiff_b, damp_b);
    ROS_INFO("[demo] PTP+JointImpedance → posa 2 inviata.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printJoints();

    // ── Return home ───────────────────────────────────────────────────────
    waitEnter("\n  Premi INVIO per tornare a home...");
    sendPtpWithJointImpedance(q_home, 0.1, stiff_a, damp_a);
    ROS_INFO("[demo] PTP+JointImpedance → ritorno home inviato.");

    waitEnter("  Premi INVIO quando il robot si è fermato...");
    printJoints();
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN MENU
// ─────────────────────────────────────────────────────────────────────────────
void printMenu()
{
    std::cout << "\n┌──────────────────────────────────────────────────┐\n";
    std::cout <<   "│  KUKA Med — Demo Node                            │\n";
    std::cout <<   "├──────────────────────────────────────────────────┤\n";
    std::cout <<   "│  1  Joint PTP                                    │\n";
    std::cout <<   "│  2  Cartesian PTP                                │\n";
    std::cout <<   "│  3  Joint Impedance (gravity compensation)       │\n";
    std::cout <<   "│  4  Joint PTP + Joint Impedance                  │\n";
    std::cout <<   "│  s  Stampa stato corrente                        │\n";
    std::cout <<   "│  q  Esci                                         │\n";
    std::cout <<   "└──────────────────────────────────────────────────┘\n";
    std::cout <<   "  Scelta: ";
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka_demo_node");
    ros::NodeHandle nh;

    // ── Publishers → kuka_base_node ──────────────────────────────────────
    pub_mode       = nh.advertise<std_msgs::Int32>            ("/robot/control_mode",           1);
    pub_joint_ptp  = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/joint_ptp",          1);
    pub_cart_ptp   = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/cart_ptp",           1);
    pub_joint_stiff= nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/joint_stiffness",    1);
    pub_joint_damp = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/joint_damping",      1);
    pub_cart_stiff = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/cart_stiffness",     1);
    pub_cart_damp  = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/cart_damping",       1);
    pub_ptp_imp    = nh.advertise<std_msgs::Float64MultiArray>("/robot/cmd/ptp_with_impedance", 1);

    // ── Subscribers ← kuka_base_node ─────────────────────────────────────
    ros::Subscriber sub_js    = nh.subscribe("/robot/state/joint_states", 1, cbJointStates);
    ros::Subscriber sub_cart  = nh.subscribe("/robot/state/cart_pose",    1, cbCartPose);
    ros::Subscriber sub_wrench= nh.subscribe("/robot/state/ext_wrench",   1, cbWrench);

    // ── Wait for kuka_base_node ───────────────────────────────────────────
    std::cout << "\n[demo] In attesa del kuka_base_node...\n";
    ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/state/joint_states");
    std::cout << "[demo] kuka_base_node connesso.\n";
    ros::Duration(0.5).sleep();     // let publishers register

    // ── Spinner in background (keeps subscribers alive) ───────────────────
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ── Interactive menu ──────────────────────────────────────────────────
    while (ros::ok())
    {
        printMenu();
        std::string choice;
        std::getline(std::cin, choice);

        if      (choice == "1") demoJointPtp();
        else if (choice == "2") demoCartesianPtp();
        else if (choice == "3") demoGravityComp();
        else if (choice == "4") demoPtpWithImpedance();
        else if (choice == "s" || choice == "S") {
            printJoints();
            printCart();
            printWrench();
        }
        else if (choice == "q" || choice == "Q") {
            std::cout << "[demo] Uscita.\n";
            break;
        }
        else {
            std::cout << "  Scelta non valida.\n";
        }
    }

    spinner.stop();
    return 0;
}