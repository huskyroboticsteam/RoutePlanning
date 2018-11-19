#include "Server.h"
#include <vector>
#include <ctime>
#include <iostream>
#include <WS2tcpip.h>

#pragma comment (lib, "ws2_32.lib")

const std::string endpoint = "MainRover"; 
sockaddr_in server;
SOCKET out;

RoverPathfinding::Server::Server()
{
	// Startup Winsock
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int ws0k = WSAStartup(version, &data);
	if (ws0k != 0)
	{
		std::cout << "Can't start Winsock!" << ws0k;
		return;
	}

	// Create a hint structure for the server
	server.sin_family = AF_INET;
	server.sin_port = htons(54000);
	inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

	// Socket creation
	out = socket(AF_INET, SOCK_DGRAM, 0);
}

bool RoverPathfinding::Server::send_action(unsigned char data, unsigned char id)
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled
	packet.push_back(data); // represents speed/position to be sent

	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*)&server, sizeof(server));
	if (sendOk == SOCKET_ERROR)
	{
		std::cout << "That didn't work! " << WSAGetLastError();
		return false;
	}
	else {
		return true;
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