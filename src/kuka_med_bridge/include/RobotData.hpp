#ifndef ROBOT_DATA_HPP
#define ROBOT_DATA_HPP

#include <cstdint>
#include <string.h>

/**
 * @brief Available control modes for the robot.
 * This enum defines the different control modes that the robot can operate in.
 * They must correspond exactly to the enum values defined in the Java code within Kuka Controller
 */

enum class ControlType : int32_t
{
    JOINT_PTP = 0,
    CARTESIAN_PTP = 1,
    JOINT_IMPEDANCE = 2,
    CARTESIAN_IMPEDANCE = 3,
    PTP_AND_JOINT_IMPEDANCE = 4,
    PTP_AND_CART_IMPEDANCE = 5,
    CART_PTP_AND_JOINT_IMP = 6,
    CART_PTP_AND_CART_IMP = 7,
    _UNUSED_8 = 8,
    _UNUSED_9 = 9,
    BLOCK = 10
};

/**
 * @brief Data structure of the command sent to the robot.
 * Use double (8 bytes) and int32_t (4 bytes) to esure alignment and size consistency between C++ and Java.
 */

 #pragma pack(push, 1)              // Enforce 1-byte alignement (no padding)
 struct RobotCommand
 {
    ControlType mode;               // 4 bytes
    double joint_target[7];         // 56 bytes
    double ptp_velocity;            // 8 bytes
    double jointStiffness[7];       // 56 bytes
    double jointDamping[7];         // 56 bytes
    double cartStiffness[6];        // 48 bytes
    double cartDamping[6];          // 48 bytes
    double cartTarget[6];           // 48 bytes
    double cartVelocity;            // 8 bytes
 };
 #pragma pack(pop)

 /**
  * @brief Data structure of the message received from the robot.
  */

  struct RobotTelemetry
  {
    double jointPositions[7];       // 56 bytes
    double measuredTorques[7];      // 56 bytes
    double externalTorques[7];      // 56 bytes
    double externalWrench[6];       // 48 bytes
    double cartesianPose[6];        // 48 bytes
  };
  
#endif // ROBOT_DATA_HPP