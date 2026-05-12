#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    typedef int SOCKET;
    #define INVALID_SOCKET -1   
    #define SOCKET_ERROR -1
#endif

#include "RobotData.hpp"


class TcpServer
{
    public:
        TcpServer();
        ~TcpServer();

        bool start(int port);
        void stop();

        // Functions to send and receive binary data
        bool receiveData(RobotTelemetry& data);
        bool sendData(const RobotCommand& data);

        bool isConnected() const { return clientSocket != INVALID_SOCKET;}

    private:
        SOCKET listenSocket;
        SOCKET clientSocket;

        void cleanup();
};

#endif // TCP_SERVER_HPP