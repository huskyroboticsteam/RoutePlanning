#include "Server.h"
#include <vector>
#include <ctime>
#include <iostream>

#ifdef _WIN32
	#include <windows.h>
	#include <WS2tcpip.h>
	#pragma comment (lib, "ws2_32.lib")
#else
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h> 
	#include <errno.h>
	#include <unistd.h>
#endif


const unsigned char WATCHDOG_ID = 0xF0;
const std::string endpoint = "MainRover"; 
sockaddr_in server;
SOCKET out;
SOCKET in;

RoverPathfinding::Server::Server()
{
#ifdef _WIN32
	// Windows-Specific - Startup Winsock
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int ws0k = WSAStartup(version, &data);
	if (ws0k != 0)
	{
		std::cout << "Can't start Winsock!" << ws0k;
		return;
	}
#endif

	// Socket creation
	out = socket(AF_INET, SOCK_DGRAM, 0);
	in = socket(AF_INET, SOCK_DGRAM, 0);

	// Bind socket to ip address and port
	sockaddr_in serverHint;
	serverHint.sin_addr.S_un.S_addr = ADDR_ANY;
	serverHint.sin_family = AF_INET;
	serverHint.sin_port = htons(54000);

	if (bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR)
	{
		std::cout << "Can't bind socket! " << WSAGetLastError();
	}

	sockaddr_in client;
	ZeroMemory(&client, sizeof(client));
	int clientLength = sizeof(client);

	char buf[256];
	ZeroMemory(buf, 256);

	inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);



	while (true) 
	{
		ZeroMemory(buf, 256);

		// Wait for message
		int bytesIn = recvfrom(in, buf, 256, 0, (sockaddr*)&client, &clientLength);
		if (bytesIn == SOCKET_ERROR) 
		{
#ifdef WIN32
			std::cout << "Error receiving from client" << WSAGetLastError();
			continue;
#else
			std::cout << "That didn't work! " << strerror(errno);
			continue;
#endif
		}

		// Display message and client 
		char clientIp[256];
		ZeroMemory(clientIp, 256);

		inet_ntop(AF_INET, &client.sin_addr, clientIp, 256);

		std::cout << "Message recieved from" << clientIp << " : " << buf;
	}
}

bool RoverPathfinding::Server::send_action(std::vector<unsigned char> dataBody, unsigned char id) // same data and id format as in Scarlet
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled
	for (unsigned i = 0; i < dataBody.size(); i++) {
		packet.push_back(dataBody[i]); // represents speed/position to be sent
	}

	// packet.data() first four bytes are the time stamp, fifth is the id, and the rest is the data
	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*)&server, sizeof(server));
	if (sendOk == SOCKET_ERROR)
	{
#ifdef _WIN32
		std::cout << "That didn't work! " << WSAGetLastError();
		return false;
#else
		std::cout << "That didn't work! " << strerror(errno);
		return false;
#endif
	}
	else {
		return true;
	}
}

bool RoverPathfinding::Server::send_action(unsigned char id) // same id format as in Scarlet
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled

	// packet.data() first four bytes are the time stamp, fifth is the id, and sixth is the data
	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*)&server, sizeof(server));
	if (sendOk == SOCKET_ERROR)
	{
#ifdef _WIN32
		std::cout << "That didn't work! " << WSAGetLastError();
		return false;
#else
		std::cout << "That didn't work! " << strerror(errno);
		return false;
#endif
	}
	else {
		return true;
	}
}

void RoverPathfinding::Server::send_watchdog() {
	boolean hasSent = RoverPathfinding::Server::send_action(WATCHDOG_ID);

	while (!hasSent) {
		// Sleep for 100 milliseconds and then try again
#ifdef _WIN32 // Windows
		Sleep(100); // in milliseconds
#else // Unix
		usleep(100 * 1000); // in microseconds
#endif

		RoverPathfinding::Server::send_action(WATCHDOG_ID);
	}
}

void RoverPathfinding::Server::stop()
{
	closesocket(out);
	WSACleanup();
}

std::vector<unsigned char> RoverPathfinding::Server::current_time()
{
	std::time_t time = std::time(0);
	std::vector<unsigned char> timebytes;

	timebytes.push_back((time >> 24) & 0xff);
	timebytes.push_back((time >> 16) & 0xff);
	timebytes.push_back((time >> 8) & 0xff);
	timebytes.push_back(time & 0xff);

	return timebytes;
}
