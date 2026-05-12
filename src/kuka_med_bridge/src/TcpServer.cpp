#include "TcpServer.hpp"
#include <iostream>

// ---------------------------------------------------------------------
// CONSTRUCTOR - initializes sockets and sets them to invalid state
// ---------------------------------------------------------------------
TcpServer::TcpServer() : listenSocket(INVALID_SOCKET), clientSocket(INVALID_SOCKET)
{
    #ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);       // Socket initialization for Windows
    #endif
}
// ---------------------------------------------------------------------
// DESTRUCTOR - ensures proper cleanup of sockets
// ---------------------------------------------------------------------
TcpServer::~TcpServer()
{
    stop();                                     // Ensure sockets are closed on destruction
    #ifdef _WIN32
        WSACleanup();                           // Cleanup Winsock on Windows
    #endif
}

// ---------------------------------------------------------------------
// START - initializes and starts the TCP server
// ---------------------------------------------------------------------
bool TcpServer::start(int port)
{
    // Create a TCP socket
    listenSocket = socket(AF_INET, SOCK_STREAM, 0);

    // Check whether socket creation was successful
    if (listenSocket == INVALID_SOCKET)
    {
        std::cerr << "Failed to create socket." << std::endl;
        return false;                               
    }

    // Set up the server address structure
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;              // Use IPv4
    serverAddr.sin_addr.s_addr = INADDR_ANY;      // Listen on all interfaces
    serverAddr.sin_port = htons(port);            // Set port number
    
    // Bind the socket to the specified port
    if (bind(listenSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
    {
        std::cerr << "Failed to bind socket." << std::endl;
        return false;                               
    }

    // Start listening for incoming connections
    listen(listenSocket, 1);                      
    std::cout << "Server started, waiting for connections on port " << port << "..." << std::endl;

    // Accept a single client connection (blocking call)
    clientSocket = accept(listenSocket, nullptr, nullptr);  
    if (clientSocket != INVALID_SOCKET)
    {
        std::cout << "Kuka connected successfully!" << std::endl;
        return true;                               
    }
    else
    {
        std::cerr << "Failed to accept Kuka connection." << std::endl;
        return false;                               
    }
}

// ---------------------------------------------------------------------
// STOP - stops and cleans up the TCP server
// ---------------------------------------------------------------------
void TcpServer::stop()
{
    if (clientSocket != INVALID_SOCKET)
    {
        #ifdef _WIN32
            closesocket(clientSocket);              // Close client socket on Windows
        #else
            close(clientSocket);                    // Close client socket on Unix-like systems
        #endif
        clientSocket = INVALID_SOCKET;             // Reset client socket state

    // Do the same also for listen socket
    if (listenSocket != INVALID_SOCKET)
    {
        #ifdef _WIN32
            closesocket(listenSocket);
        #else
            close(listenSocket);
        #endif
        listenSocket = INVALID_SOCKET;
    }
    }
}

// ---------------------------------------------------------------------
// RECEIVEDATA - receives binary data from the client and fills the provided structure
// ---------------------------------------------------------------------
bool TcpServer::receiveData(RobotTelemetry& data)
{
    char* buf = (char*)&data;
    int total = 0;
    int needed = sizeof(RobotTelemetry);

    while (total < needed) {
        int bytesRead = recv(clientSocket, buf + total, needed - total, 0);
        if (bytesRead <= 0) return false;
        total += bytesRead;
    }
    return true;
}

// ---------------------------------------------------------------------
// SENDDATA - sends binary data to the client from the provided structure
// ---------------------------------------------------------------------
bool TcpServer::sendData(const RobotCommand& data)
{
    const char* buf = (const char*)&data;
    int total = 0;
    int needed = sizeof(RobotCommand);

    while (total < needed) {
        int bytesSent = send(clientSocket, buf + total, needed - total, 0);
        if (bytesSent <= 0) return false;
        total += bytesSent;
    }
    return true;
}
