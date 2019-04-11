#include "Server.hpp"


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
	#include <string.h>
	#define SOCKET int
	#define SOCKET_ERROR -1
#endif


const unsigned char WATCHDOG_ID = 0xF0;
const std::string endpoint = "MainRover"; 
unsigned char buf[256];
sockaddr_in server;
SOCKET out;
SOCKET in;

RP::Server::Server()
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

	//Example of how to set up sendto address:
	server.sin_family = AF_INET;
	server.sin_port = htons(54100);
	inet_aton("10.19.216.238", &(server.sin_addr));
	memset(&(server.sin_zero), '\0', 8);
	
	// Bind socket to ip address and port
	sockaddr_in serverHint;
	serverHint.sin_addr.s_addr = INADDR_ANY;
	//inet_pton(AF_INET, "10.19.161.242", &serverHint.sin_addr.s_addr);
	serverHint.sin_family = AF_INET;
	serverHint.sin_port = htons(54050);
	
	if (bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR)
	{
#ifdef _WIN32
		std::cout << "Can't bind socket! " << WSAGetLastError() << std::endl;
#else
		std::cout << "Can't bind socket! " << strerror(errno) << std::endl;
#endif
	}
}

//TODO: make this a shared_ptr or a vector (we don't want to return a pointer to a static array)
unsigned char* RP::Server::go() {
	sockaddr_in client;
#ifdef _WIN32
	ZeroMemory(&client, sizeof(client));
#else
	memset(&client, 0, sizeof(client));
#endif
	unsigned int clientLength = sizeof(client);
#ifdef _WIN32
	ZeroMemory(buf, 256);
#else
	memset(buf,0, 256);
#endif
	
#ifdef _WIN32
	ZeroMemory(buf, 256);
#else
	memset(buf, 0, 256);
#endif

	// Wait for message
	int bytesIn = recvfrom(in, buf, 256, 0, (sockaddr*)&client, &clientLength);
	if (bytesIn == SOCKET_ERROR) 
	{
#ifdef _WIN32
		std::cout << "Error receiving from client" << WSAGetLastError();
	
#else
		std::cout << "That didn't work! " << strerror(errno);
		
#endif
	}

	// Display message and client 
	char clientIp[256];
#ifdef _WIN32
	ZeroMemory(clientIp, 256);
#else
	memset(clientIp, 0, 256);
#endif
	inet_ntop(AF_INET, &client.sin_addr, clientIp, 256);

	std::cout << "Message received from " << clientIp << " : " << buf << std::endl;
		
	return buf; 
}

bool RP::Server::send_action(std::vector<unsigned char> dataBody, unsigned char id) // same data and id format as in Scarlet
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

bool RP::Server::send_action(unsigned char dataBody, unsigned char id) // same data and id format as in Scarlet
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled
	packet.push_back(dataBody); // represents speed/position to be sent

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

bool RP::Server::send_action(unsigned char id) // same id format as in Scarlet
{
	//std::cout << "started send_action" << std::endl;
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled

	//std::cout << "trying to send" << std::endl;
	// packet.data() first four bytes are the time stamp, fifth is the id, and sixth is the data
	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*) &server, sizeof(server));
	//std::cout << "sent" << std::endl;
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

void RP::Server::send_watchdog() {
	//std::cout << "Entered watchdog thread" << std::endl;
	bool hasSent;
	while (true) {
		//std::cout << "trying to send" << std::endl;
		hasSent = RP::Server::send_action(WATCHDOG_ID);
		//std::cout << "tried to send: " << hasSent << std::endl;
		while (!hasSent) {
			hasSent = RP::Server::send_action(WATCHDOG_ID);
		}
		// Sleep for 100 milliseconds and then try again
#ifdef _WIN32 // Windows
		Sleep(100); // in milliseconds
#else // Unix
		usleep(100 * 1000); // in microseconds
#endif
//		std::cout << "Finished loop" << std::endl;
	}
}

void RP::Server::stop()
{
	close(out);
#ifdef _WIN32
	WSACleanup();
#endif
}

std::vector<unsigned char> RP::Server::current_time()
{
	std::time_t time = std::time(0);
	std::vector<unsigned char> timebytes;

	timebytes.push_back((time >> 24) & 0xff);
	timebytes.push_back((time >> 16) & 0xff);
	timebytes.push_back((time >> 8) & 0xff);
	timebytes.push_back(time & 0xff);

	return timebytes;
}
