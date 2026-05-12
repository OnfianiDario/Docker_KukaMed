#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "RobotData.hpp"
#include "TcpServer.hpp"
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <array>

// #pragma pack(push,1)    // Ensure alignement of data with Java

class Controller
{
    public:
        Controller(int port = 5005);
        ~Controller();

        bool start();
        void stop();
        
        // Getters
        ControlType getControlMode() const; 
        std::array<double,7> getCurrentJointPositions() const;
        std::array<double,6> getCurrentCartesianPose() const;
        std::array<double,7> getExternalTorques() const;
        std::array<double,7> getMeasuredTorques() const;
        std::array<double,6> getExternalWrench() const;
        std::array<double,7> getJointStiffness() const;
        std::array<double,7> getJointDamping() const;
        std::array<double,6> getCartesianStiffness() const;
        std::array<double,6> getCartesianDamping() const;
        
        
        // Setters
        void setControlMode(ControlType mode);
        void setJointStiffness(const std::array<double,7>& stiff);
        void setJointDamping(const std::array<double,7>& damp);
        void setCartesianStiffness(const std::array<double,6>& stiff);
        void setCartesianDamping(const std::array<double,6>& damp);
        void moveJointsPtp(const std::array<double,7>& q_des, double jointPtpVel = 0.1);
        void moveCartesianPtp(const std::array<double,6>& p_des, double cartPtpVel);
        // void movePtpWithJointImpedance(double (&q_des)[7], double jointPtpVel = 0.1, double (&stiff)[7] = _default_joint_stiff, double (&damp)[7] = _default_joint_damp);
        void movePtpWithImpedance(
            ControlType motionType,             // JOINT_PTP o CARTESIAN_PTP
            ControlType impedanceType,          // JOINT_IMPEDANCE o CARTESIAN_IMPEDANCE
            double* target,                     // 7 valori (joint) o 6 (cartesian)
            double velocity = 0.1,
            double* stiff = nullptr,            // null = usa default
            double* damp  = nullptr
        );

        // Helpers
        std::string modeToString(ControlType modes);

    private:
        void workerLoop();

        // Socket    
        TcpServer server;
        int port;

        // Threading
        std::thread workerThread;
        std::atomic<bool> running;
        mutable std::mutex dataMutex;

        // State
        RobotTelemetry _lastTelemetry;
        RobotCommand currentCommand;
        bool isInitialPoseSet;

        // Robot Control
        ControlType _currentControlMode;
        std::array<double,7> _jointStiffness;
        std::array<double,7> _jointDamping;
        std::array<double,6> _cartesianStiffness;
        std::array<double,6> _cartesianDamping;
        std::array<double,7> _jointTargetPtp;
        std::array<double,6> _cartesianTargetPtp;
        double _jointPtpVelocity;
        double _cartesianPtpVelocity;

        static double _default_joint_stiff[7];
        static double _default_joint_damp[7];
        static double _default_cart_stiff[6];
        static double _default_cart_damp[6];        
};

// #pragma pack(pop)

#endif //CONTROLLER_HPP