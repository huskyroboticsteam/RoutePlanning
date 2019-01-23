#include "Server.cpp"

int main() {
	RoverPathfinding::Server server1 = RoverPathfinding::Server();
	server1.send_action(0x01);
	return 0;
}