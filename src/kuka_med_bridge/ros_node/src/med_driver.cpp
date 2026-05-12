#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#include "Controller.hpp"

// --- Global controller instance ---
Controller* robot = nullptr;

// ************ SUBSCRIBER CALLBACKS **********************

// /robot/control_mode  [std_msgs/Int32]
// Sets the active control mode (see ControlType enum in RobotData.hpp)
void cbControlMode(const std_msgs::Int32::ConstPtr& msg)
{
    robot->setControlMode(static_cast<ControlType>(msg->data));
    ROS_INFO("[med_driver] Control mode set to %d", msg->data);
}

// /robot/cmd/joint_ptp  [std_msgs/Float64MultiArray]
// data: [q0..q6, velocity]  — 8 values total
void cbJointPtp(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 8) {
        ROS_WARN("[med_driver] joint_ptp: expected 8 values (7 joints + velocity), got %zu", msg->data.size());
        return;
    }
    std::array<double, 7> q;
    for (int i = 0; i < 7; i++) q[i] = msg->data[i];
    double vel = msg->data[7];
    robot->moveJointsPtp(q, vel);
}

// /robot/cmd/cart_ptp  [std_msgs/Float64MultiArray]
// data: [x, y, z, A, B, C, velocity]  — 7 values total
void cbCartPtp(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 7) {
        ROS_WARN("[med_driver] cart_ptp: expected 7 values (6 pose + velocity), got %zu", msg->data.size());
        return;
    }
    std::array<double, 6> p;
    for (int i = 0; i < 6; i++) p[i] = msg->data[i];
    double vel = msg->data[6];
    robot->moveCartesianPtp(p, vel);
}

// /robot/cmd/joint_stiffness  [std_msgs/Float64MultiArray]
// data: [k0..k6]  — 7 values
void cbJointStiffness(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 7) {
        ROS_WARN("[med_driver] joint_stiffness: expected 7 values, got %zu", msg->data.size());
        return;
    }
    std::array<double, 7> k;
    for (int i = 0; i < 7; i++) k[i] = msg->data[i];
    robot->setJointStiffness(k);
}

// /robot/cmd/joint_damping  [std_msgs/Float64MultiArray]
// data: [d0..d6]  — 7 values
void cbJointDamping(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 7) {
        ROS_WARN("[med_driver] joint_damping: expected 7 values, got %zu", msg->data.size());
        return;
    }
    std::array<double, 7> d;
    for (int i = 0; i < 7; i++) d[i] = msg->data[i];
    robot->setJointDamping(d);
}

// /robot/cmd/cart_stiffness  [std_msgs/Float64MultiArray]
// data: [kx, ky, kz, krx, kry, krz]  — 6 values
void cbCartStiffness(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 6) {
        ROS_WARN("[med_driver] cart_stiffness: expected 6 values, got %zu", msg->data.size());
        return;
    }
    std::array<double, 6> k;
    for (int i = 0; i < 6; i++) k[i] = msg->data[i];
    robot->setCartesianStiffness(k);
}

// /robot/cmd/cart_damping  [std_msgs/Float64MultiArray]
// data: [dx, dy, dz, drx, dry, drz]  — 6 values, clamped to [0, 1]
void cbCartDamping(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 6) {
        ROS_WARN("[med_driver] cart_damping: expected 6 values, got %zu", msg->data.size());
        return;
    }
    std::array<double, 6> d;
    for (int i = 0; i < 6; i++) d[i] = msg->data[i];
    robot->setCartesianDamping(d);
}

// /robot/cmd/ptp_with_impedance  [std_msgs/Float64MultiArray]
//
// Joint PTP + Joint Impedance   — data: [motionType=0, impedanceType=2, q0..q6, vel, k0..k6, d0..d6]  — 17 values
// Joint PTP + Cart Impedance    — data: [motionType=0, impedanceType=3, q0..q6, vel, kx..krz, dx..drz] — 16 values
// Cart PTP + Joint Impedance    — data: [motionType=1, impedanceType=2, x..C,   vel, k0..k6, d0..d6]  — 16 values
// Cart PTP + Cart Impedance     — data: [motionType=1, impedanceType=3, x..C,   vel, kx..krz, dx..drz] — 15 values

// Layout: [motionType, impedanceType, target..., velocity, stiffness..., damping...]
void cbPtpWithImpedance(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Minimum: 2 type fields + 6 target + 1 velocity + 6 stiff + 6 damp = 21 — but varies.
    // Validate based on motion/impedance type combination.
    if (msg->data.size() < 2) {
        ROS_WARN("[med_driver] ptp_with_impedance: message too short (%zu values)", msg->data.size());
        return;
    }

    const ControlType motionType    = static_cast<ControlType>(static_cast<int>(msg->data[0]));
    const ControlType impedanceType = static_cast<ControlType>(static_cast<int>(msg->data[1]));

    // Validate motion type
    if (motionType != ControlType::JOINT_PTP && motionType != ControlType::CARTESIAN_PTP) {
        ROS_WARN("[med_driver] ptp_with_impedance: invalid motionType %d", static_cast<int>(motionType));
        return;
    }

    // Validate impedance type
    if (impedanceType != ControlType::JOINT_IMPEDANCE && impedanceType != ControlType::CARTESIAN_IMPEDANCE) {
        ROS_WARN("[med_driver] ptp_with_impedance: invalid impedanceType %d", static_cast<int>(impedanceType));
        return;
    }

    const int targetSize    = (motionType    == ControlType::JOINT_PTP)       ? 7 : 6;
    const int impedanceSize = (impedanceType == ControlType::JOINT_IMPEDANCE) ? 7 : 6;
    const int expected      = 2 + targetSize + 1 + impedanceSize + impedanceSize;

    if ((int)msg->data.size() != expected) {
        ROS_WARN("[med_driver] ptp_with_impedance: expected %d values, got %zu", expected, msg->data.size());
        return;
    }

    int idx = 2;

    // Target
    std::array<double, 7> target7{};
    std::array<double, 6> target6{};
    double* target_ptr = nullptr;

    if (motionType == ControlType::JOINT_PTP) {
        for (int i = 0; i < 7; i++) target7[i] = msg->data[idx++];
        target_ptr = target7.data();
    } else {
        for (int i = 0; i < 6; i++) target6[i] = msg->data[idx++];
        target_ptr = target6.data();
    }

    // Velocity
    const double velocity = msg->data[idx++];

    // Stiffness
    std::array<double, 7> stiff7{};
    std::array<double, 6> stiff6{};
    double* stiff_ptr = nullptr;

    if (impedanceType == ControlType::JOINT_IMPEDANCE) {
        for (int i = 0; i < 7; i++) stiff7[i] = msg->data[idx++];
        stiff_ptr = stiff7.data();
    } else {
        for (int i = 0; i < 6; i++) stiff6[i] = msg->data[idx++];
        stiff_ptr = stiff6.data();
    }

    // Damping
    std::array<double, 7> damp7{};
    std::array<double, 6> damp6{};
    double* damp_ptr = nullptr;

    if (impedanceType == ControlType::JOINT_IMPEDANCE) {
        for (int i = 0; i < 7; i++) damp7[i] = msg->data[idx++];
        damp_ptr = damp7.data();
    } else {
        for (int i = 0; i < 6; i++) damp6[i] = msg->data[idx++];
        damp_ptr = damp6.data();
    }

    robot->movePtpWithImpedance(motionType, impedanceType, target_ptr, velocity, stiff_ptr, damp_ptr);
}

// *********************************************************************************************************

// MAIN


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka_base_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    int port;
    nh_priv.param("port", port, 5005);

    // Connect to robot
    Controller controller(port);
    robot = &controller;

    if (!robot->start()) {
        ROS_ERROR("[med_driver] Failed to connect to robot on port %d", port);
        return -1;
    }
    ROS_INFO("[med_driver] Robot connected on port %d", port);

    // --- Subscribers ---
    ros::Subscriber sub_mode      = nh.subscribe("/robot/control_mode",           1, cbControlMode);
    ros::Subscriber sub_jptp      = nh.subscribe("/robot/cmd/joint_ptp",          1, cbJointPtp);
    ros::Subscriber sub_cptp      = nh.subscribe("/robot/cmd/cart_ptp",           1, cbCartPtp);
    ros::Subscriber sub_jstiff    = nh.subscribe("/robot/cmd/joint_stiffness",    1, cbJointStiffness);
    ros::Subscriber sub_jdamp     = nh.subscribe("/robot/cmd/joint_damping",      1, cbJointDamping);
    ros::Subscriber sub_cstiff    = nh.subscribe("/robot/cmd/cart_stiffness",     1, cbCartStiffness);
    ros::Subscriber sub_cdamp     = nh.subscribe("/robot/cmd/cart_damping",       1, cbCartDamping);
    ros::Subscriber sub_ptp_imp   = nh.subscribe("/robot/cmd/ptp_with_impedance", 1, cbPtpWithImpedance);

    // --- Publishers ---
    auto pub_js     = nh.advertise<sensor_msgs::JointState>      ("/robot/state/joint_states", 10);
    auto pub_wrench = nh.advertise<geometry_msgs::WrenchStamped> ("/robot/state/ext_wrench",   10);
    auto pub_cart   = nh.advertise<std_msgs::Float64MultiArray>  ("/robot/state/cart_pose",    10);
    auto pub_extq   = nh.advertise<std_msgs::Float64MultiArray>  ("/robot/state/ext_torques",  10);
    auto pub_meastq = nh.advertise<std_msgs::Float64MultiArray>  ("/robot/state/meas_torques", 10);

    const std::vector<std::string> joint_names = 
    {
        "lbr_joint_0","lbr_joint_1","lbr_joint_2","lbr_joint_3",
        "lbr_joint_4","lbr_joint_5","lbr_joint_6"
    };

    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        auto now = ros::Time::now();

        // Joint states
        auto q    = robot->getCurrentJointPositions();
        auto meas = robot->getMeasuredTorques();
        sensor_msgs::JointState js;
        js.header.stamp = now;
        js.name         = joint_names;
        js.position.assign(q.begin(), q.end());
        js.effort.assign(meas.begin(), meas.end());
        pub_js.publish(js);

        // External wrench
        auto w = robot->getExternalWrench();
        geometry_msgs::WrenchStamped ws;
        ws.header.stamp    = now;
        ws.header.frame_id = "lbr_link_ee";
        ws.wrench.force.x  = w[0]; ws.wrench.force.y  = w[1]; ws.wrench.force.z  = w[2];
        ws.wrench.torque.x = w[3]; ws.wrench.torque.y = w[4]; ws.wrench.torque.z = w[5];
        pub_wrench.publish(ws);

        // Cartesian pose
        auto cart = robot->getCurrentCartesianPose();
        std_msgs::Float64MultiArray cart_msg;
        cart_msg.data.assign(cart.begin(), cart.end());
        pub_cart.publish(cart_msg);

        // External torques
        auto ext = robot->getExternalTorques();
        std_msgs::Float64MultiArray ext_msg;
        ext_msg.data.assign(ext.begin(), ext.end());
        pub_extq.publish(ext_msg);

        // Measured torques
        std_msgs::Float64MultiArray meas_msg;
        meas_msg.data.assign(meas.begin(), meas.end());
        pub_meastq.publish(meas_msg);

        rate.sleep();
    }

    robot->stop();
    return 0;
}