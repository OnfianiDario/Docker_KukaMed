#include <iostream>
#include <thread>
#include <chrono>
#include "TcpServer.hpp"
#include "Controller.hpp"
#include <cmath> 
#include <array>

int main() 
{
    // 1. Create controller
    Controller robot;

    // 2. Start and check whether everything is ok
    if (!robot.start()) 
    {
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (true) 
    {
        std::array<double, 6> p = robot.getCurrentCartesianPose();
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed >= 0.5)
            break;
    }

    std::cout << "Kuka Med ready and connected" << std::endl;
    
    // Wait some time
    std::cout << "Waiting for Robot Data" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Read first cartesian pose
    std::array<double, 6> currentPose = robot.getCurrentCartesianPose();

    std::cout << "Current cartesian pose:" << std::endl;
    for (int i = 0; i < 6; i++)
        std::cout << currentPose[i] << std::endl;


    // ****************** ESEMPIO 1 **********************************
    // Joint Impedance con stiffness 0 — modalità gravity compensation
    // Premi INVIO per stampare la posa attuale dei giunti

    // double stiff[7] = {0, 0, 0, 0, 0, 0, 0};
    // double damp[7]  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

    // robot.setControlMode(ControlType::JOINT_IMPEDANCE);
    // robot.setJointStiffness(stiff);
    // robot.setJointDamping(damp);

    // std::cout << "Robot in gravity compensation. Premi INVIO per stampare la posa." << std::endl;
    // std::cout << "Premi CTRL+C per uscire." << std::endl;

    // while (true)
    // {
    //     std::cin.get();

    //     std::array<double, 7> q = robot.getCurrentJointPositions();
    //     std::cout << "\n--- Posa attuale [rad] ---" << std::endl;
    //     for (int i = 0; i < 7; i++)
    //     {
    //         std::cout << "  q[" << i << "] = " << q[i] << std::endl;
    //     }
    //     std::cout << std::endl;
    // }

    // // ****************** CONTROLLO CARTESIANO PTP **********************************
    
    // std::array<double,6> p = robot.getCurrentCartesianPose();
    // double p_des[6] = {-400, -550, 630, -100, -42, 168};
    // // robot.moveCartesianPtp(p_des, 0.1);

    // while (true)
    // {
    //     std::cin.get();

    //     std::array<double, 7> q = robot.getCurrentJointPositions();
    //     std::cout << "\n--- Posa attuale [rad] ---" << std::endl;
    //     for (int i = 0; i < 7; i++)
    //     {
    //         std::cout << "  q[" << i << "] = " << q[i] << std::endl;
    //     }
    //     std::cout << std::endl;
    // }
       

    // // ***************** MOVE PTP WITH IMPEDANCE **************************************++
    // ── MOVIMENTO 1: Joint PTP + Joint Impedance ──────────────────
        std::cout << "\n[1] Joint PTP + Joint Impedance. Premi INVIO per avviare..." << std::endl;
        std::cin.get();

        std::array<double,7> q     = {0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0};
        std::array<double,7> stiff = {500, 500, 500, 300, 300, 200, 100};
        std::array<double,7> damp  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

        robot.movePtpWithImpedance(
            ControlType::JOINT_PTP, ControlType::JOINT_IMPEDANCE, q.data(), 0.1, stiff.data(), damp.data());

        std::cout << "Movimento in corso... Premi INVIO quando il robot si è fermato." << std::endl;
        std::cin.get();

        // MOVIMENTO 2 : Joint PTP + Cartesian Impedance
        std::cout << "\n[1] Joint PTP + Joint Impedance. Premi INVIO per avviare..." << std::endl;
        std::cin.get();

        q[1] = 0.0;
        std::array<double,6> c_stiff = {50, 50, 50, 10, 10, 10};
        std::array<double,6> c_damp  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

        robot.movePtpWithImpedance(
            ControlType::JOINT_PTP, ControlType::CARTESIAN_IMPEDANCE, q.data(), 0.1, c_stiff.data(), c_damp.data());

        std::cout << "Movimento in corso... Premi INVIO quando il robot si è fermato." << std::endl;
        std::cin.get();

        

        // ── MOVIMENTO 3: Cartesian PTP + Cartesian Impedance ──────────
        std::cout << "\n[3] Cartesian PTP + Cartesian Impedance. Premi INVIO per avviare..." << std::endl;
        std::cin.get();

        std::array<double,6> p0 = robot.getCurrentCartesianPose();
        std::array<double,6> p      = {p0[0], p0[1], p0[2] + 50.0, p0[3], p0[4], p0[5]}; // +50mm su Z
        std::array<double,6> cStiff = {2000, 2000, 2000, 200, 200, 200};
        std::array<double,6> cDamp  = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

        robot.movePtpWithImpedance(
            ControlType::CARTESIAN_PTP, ControlType::CARTESIAN_IMPEDANCE, p.data(), 0.1, cStiff.data(), cDamp.data());

        std::cout << "Movimento in corso... Premi INVIO per fermare il robot e uscire." << std::endl;
    std::cin.get();

    robot.stop();
    return 0;
}