#include "Controller.hpp"
#include <iostream>
#include <cstring>
#include <string.h>
#include <stdexcept>
#include <string>
#include <algorithm>

double Controller::_default_joint_stiff[7] = {1000,1000,1000,1000,1000,1000,1000};
double Controller::_default_joint_damp[7]  = {0.7,0.7,0.7,0.7,0.7,0.7,0.7};
double Controller::_default_cart_stiff[6] = {100,100,100,100,100,100};
double Controller::_default_cart_damp[6]  = {0.7,0.7,0.7,0.7,0.7,0.7};

Controller::Controller(int p) : port(p), running(false), isInitialPoseSet(false)
{
    std::memset(&currentCommand, 0, sizeof(RobotCommand));
    std::memset(&_lastTelemetry, 0, sizeof(RobotTelemetry));
     _currentControlMode = ControlType::JOINT_IMPEDANCE;
    
    // Initialize currentCommand
    currentCommand.mode = ControlType::JOINT_PTP;
    currentCommand.cartVelocity = 0.1;
    currentCommand.ptp_velocity = 0.1;

    // Joint Parameters
    for(int i = 0; i < 7; i++)
    {
        currentCommand.jointStiffness[i] = _default_joint_stiff[i];
        currentCommand.jointDamping[i] = _default_joint_damp[i];
        currentCommand.joint_target[i] = 0.0;
    }

    // Cartesian Parameters
    for(int i = 0; i < 6; i++)
    {
        currentCommand.cartStiffness[i] = 100;
        currentCommand.cartDamping[i] = 0.7;
        currentCommand.cartTarget[i] = 0.0;
    }
}

Controller::~Controller()
{
    stop();
}

bool Controller::start()
{
    // Start socket server on the local PC
    if(!server.start(port))
    {
        return false;
    }

    // Start communication thread
    running = true;
    workerThread = std::thread(&Controller::workerLoop, this);
    return true;
}

void Controller::stop()
{
    running = false;
    if(workerThread.joinable()) workerThread.join();
    server.stop();
}

void Controller::workerLoop() 
{
    RobotTelemetry incoming;
    const std::chrono::milliseconds cycleDuration(10);
    
    while (running) 
    {
        auto start_time = std::chrono::steady_clock::now();

        // 1. Receive robot data
        if (server.receiveData(incoming)) {

            RobotCommand cmdToSend;
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                _lastTelemetry = incoming;

                // Auto-initialization
                if (!isInitialPoseSet) {
                    for(int i=0; i<7; i++) currentCommand.joint_target[i] = incoming.jointPositions[i];
                    isInitialPoseSet = true;
                    std::cout << "[Controller] Got initial pose" << std::endl;
                }

                cmdToSend = currentCommand;
            } // lock released here

            // 2. Send the last defined command
            server.sendData(cmdToSend);
        } 
        else 
        {
            // If connection get lost, stop the loop
            std::cerr << "[Controller] Connection lost." << std::endl;
            running = false;
        }

        // Sleep to maintain frequency rate
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        auto sleep_time = cycleDuration - elapsed_time;

        if(sleep_time > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_time);
        }
        else
        {
            // std::cerr << "Warning: Control loop duration = " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() << " > 10ms!" << std::endl;
        }
    }
}


// -------------------------------------------------------------------
//                          SETTERS
// -------------------------------------------------------------------

void Controller::setControlMode(ControlType mode) 
{
    std::lock_guard<std::mutex> lock(dataMutex);
    _currentControlMode = mode;
    currentCommand.mode = mode;
}

void Controller::setJointStiffness(const std::array<double,7>& stiff)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    for (int i=0; i<7; i++) 
    {   _jointStiffness[i] = stiff[i];
        currentCommand.jointStiffness[i] = stiff[i];
    }

    return;
}

void Controller::setJointDamping(const std::array<double,7>& damp)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    for (int i=0; i<7; i++) 
    {
        _jointDamping[i] = damp[i];
        currentCommand.jointDamping[i] = damp[i];
    }
    
    return;
}

void Controller::setCartesianStiffness(const std::array<double,6>& stiff)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    for (int i=0; i<6; i++) 
    {   _cartesianStiffness[i] = stiff[i];
        currentCommand.cartStiffness[i] = stiff[i];
    }
    
    return;
}

void Controller::setCartesianDamping(const std::array<double,6>& damp)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    for (int i = 0; i<6; i++)
    {
        double safeValue = std::clamp(damp[i], 0.0, 1.0);

        if (safeValue != damp[i])
        {
            std::cout << "WARNING: Damping value " << i << " saturated to "  << safeValue << std::endl;
        }

        _cartesianDamping[i] = safeValue;
        currentCommand.cartDamping[i] = safeValue;
    }
    
    return;
}

void Controller::moveJointsPtp(const std::array<double,7>& q_des, double jointPtpVel)
{
    std::lock_guard<std::mutex> lock(dataMutex);

    // Clamp joint positions into safe range
    std::array<double,7> q = q_des;
    q[0] = std::clamp(q[0], -2.96, 2.96);
    q[1] = std::clamp(q[1], -2.09, 2.09);
    q[2] = std::clamp(q[2], -2.96, 2.96);
    q[3] = std::clamp(q[3], -2.09, 2.09);
    q[4] = std::clamp(q[4], -2.96, 2.96);
    q[5] = std::clamp(q[5], -2.09, 2.09);
    q[6] = std::clamp(q[6], -3.05, 3.05);

    // Apply joint values
    for(int i = 0; i < 7; i++)
    {
        currentCommand.joint_target[i] = q[i];
        _jointTargetPtp[i] = q[i];
    }
    
    // Set velocity
    currentCommand.ptp_velocity = jointPtpVel;
    _jointPtpVelocity = jointPtpVel;

    // Ensure that robot control mode is set to JointPtp
    _currentControlMode = ControlType::JOINT_PTP;
    currentCommand.mode = ControlType::JOINT_PTP;

    return;
}

void Controller::moveCartesianPtp(const std::array<double,6>& p_des, double cartPtpVel)
{
    std::lock_guard<std::mutex> lock(dataMutex);

    // Apply pose values
    for(int i = 0; i < 6; i++)
    {
        currentCommand.cartTarget[i] = p_des[i];
        _cartesianTargetPtp[i] = p_des[i];
    }
    
    // Set velocity
    currentCommand.cartVelocity = cartPtpVel;
    _cartesianPtpVelocity = cartPtpVel;

    // Ensure that robot control mode is set to JointPtp
    _currentControlMode = ControlType::CARTESIAN_PTP;
    currentCommand.mode = ControlType::CARTESIAN_PTP;

    return;
}

// void Controller::movePtpWithJointImpedance(double (&q_des)[7], double jointPtpVel, double (&stiff)[7], double (&damp)[7])
// {
//     std::lock_guard<std::mutex> lock(dataMutex);

//     // Apply joint values and impedance parameters
//     for(int i = 0; i < 7; i++)
//     {
//         currentCommand.joint_target[i] = q_des[i];
//         currentCommand.jointStiffness[i] = stiff[i];
//         currentCommand.jointDamping[i] = damp[i];
//         _jointTargetPtp[i] = q_des[i];
//         _jointDamping[i] = damp[i];
//         _jointStiffness[i] = stiff[i];
//     }
    
//     // Set velocity
//     currentCommand.ptp_velocity = jointPtpVel;
//     _jointPtpVelocity = jointPtpVel;

//     // Ensure that robot control mode is set to JointPtp
//     _currentControlMode = ControlType::PTP_AND_JOINT_IMPEDANCE;
//     currentCommand.mode = ControlType::PTP_AND_JOINT_IMPEDANCE;

//     return;
// }

void Controller::movePtpWithImpedance(
    ControlType motionType,
    ControlType impedanceType,
    double* target,
    double velocity,
    double* stiff,
    double* damp)
{
    std::lock_guard<std::mutex> lock(dataMutex);

    // Determina il ControlType combinato da inviare a Java
    ControlType combined;
    if (motionType == ControlType::JOINT_PTP && impedanceType == ControlType::JOINT_IMPEDANCE)
        combined = ControlType::PTP_AND_JOINT_IMPEDANCE;
    else if (motionType == ControlType::JOINT_PTP && impedanceType == ControlType::CARTESIAN_IMPEDANCE)
        combined = ControlType::PTP_AND_CART_IMPEDANCE;
    else if (motionType == ControlType::CARTESIAN_PTP && impedanceType == ControlType::JOINT_IMPEDANCE)
        combined = ControlType::CART_PTP_AND_JOINT_IMP;
    else  // CARTESIAN_PTP + CARTESIAN_IMPEDANCE
        combined = ControlType::CART_PTP_AND_CART_IMP;

    currentCommand.mode = combined;
    _currentControlMode = combined;
    currentCommand.ptp_velocity = velocity;
    currentCommand.cartVelocity = velocity;

    // Target: joint (7) o cartesiano (6)
    if (motionType == ControlType::JOINT_PTP) {
        for (int i = 0; i < 7; i++) currentCommand.joint_target[i] = target[i];
    } else {
        for (int i = 0; i < 6; i++) currentCommand.cartTarget[i] = target[i];
    }

    // Joint impedance params
    if (impedanceType == ControlType::JOINT_IMPEDANCE) {
        double* s = stiff ? stiff : _default_joint_stiff;
        double* d = damp  ? damp  : _default_joint_damp;
        for (int i = 0; i < 7; i++) {
            currentCommand.jointStiffness[i] = s[i];
            currentCommand.jointDamping[i]   = d[i];
        }
    }
    // Cartesian impedance params
    else {
        double* s = stiff ? stiff : _default_cart_stiff;
        double* d = damp  ? damp  : _default_cart_damp;
        for (int i = 0; i < 6; i++) {
            currentCommand.cartStiffness[i] = s[i];
            currentCommand.cartDamping[i]   = d[i];
        }
    }
}

// -------------------------------------------------------------------
//                          GETTERS
// -------------------------------------------------------------------
std::array<double,7> Controller::getCurrentJointPositions() const
{
    std::lock_guard<std::mutex> lock(dataMutex);
    std::array<double,7> out;
    for (int i = 0; i < 7; i++) out[i] = _lastTelemetry.jointPositions[i];
    return out;
}

std::array<double,6> Controller::getCurrentCartesianPose() const
{
    std::lock_guard<std::mutex> lock(dataMutex);
    std::array<double,6> out;
    for (int i = 0; i < 6; i++) out[i] = _lastTelemetry.cartesianPose[i];
    return out;
}

std::array<double,7> Controller::getExternalTorques() const
{
    std::lock_guard<std::mutex> lock(dataMutex);
    std::array<double,7> out;
    for (int i = 0; i < 7; i++) out[i] = _lastTelemetry.externalTorques[i];
    return out;
}

std::array<double,7> Controller::getMeasuredTorques() const
{
    std::lock_guard<std::mutex> lock(dataMutex);
    std::array<double,7> out;
    for (int i = 0; i < 7; i++) out[i] = _lastTelemetry.measuredTorques[i];
    return out;
}

std::array<double,6> Controller::getExternalWrench() const
{
    std::lock_guard<std::mutex> lock(dataMutex);
    std::array<double,6> out;
    for (int i = 0; i < 6; i++) out[i] = _lastTelemetry.externalWrench[i];
    return out;
}

ControlType Controller::getControlMode() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(dataMutex));
    return _currentControlMode;
}

std::array<double,7> Controller::getJointStiffness() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(dataMutex));
    return _jointStiffness;
}

std::array<double,7> Controller::getJointDamping() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(dataMutex));
    return _jointDamping;
}

std::array<double,6> Controller::getCartesianStiffness() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(dataMutex));
    return _cartesianStiffness;
}

std::array<double,6> Controller::getCartesianDamping() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(dataMutex));
    return _cartesianDamping;
}