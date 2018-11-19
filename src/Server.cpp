#include "Server.h"
#include <vector>
#include <ctime>

const std::string endpoint = "MainRover"; 

RoverPathfinding::Server::Server()
{

}

void RoverPathfinding::Server::send(unsigned char data, unsigned char id)
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled
	packet.push_back(data); // represents speed/position to be sent

	reinterpret_cast<char*>(packet.data());

	// TODO send data
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