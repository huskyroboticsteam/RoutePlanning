#include <vector>
#include <string>
#include <cctype>
#include <cmath>
#include <map>

const std::string endpoint = "MainRover"; 

public:

RoverPathfinding::Server::Server(std::string address)
{
	
}

void RoverPathfinding::Server::send(unsigned char data, unsigned char id)
{
	std::vector<unsigned char> packet = current_time(); // start packet with time stamp
	packet.push_back(id); // represents component to be controlled
	packet.push_back(data); // represents speed/position to be sent

	// TODO: send data
}

private:

// obtains current time since epoch in the form of seconds converted to bytes
std::vector<unsigned char> RoverPathfinding::Server::current_time()
{
	std::time_t time = std::time(0);
	std::vector<unsigned char> timeBytes;

	timeBytes.push_back((time >> 24) & 0xFF);
	timeBytes.push_back((time >> 16) & 0xFF);
	timeBytes.push_back((time >> 8) & 0xFF);
	timeBytes.push_back(time & 0xFF);

	return timeBytes;
}