/*
Server class that handles networking; e.g. setting up a server and connections,
sending/receiving and encoding/decoding data packets, etc.
*/

#ifndef ROVERPATHFINDING_SERVER_H
#define ROVERPATHFINDING_SERVER_H

#include "Controller.h"

namespace RoverPathfinding
{
class Server
{
  public:
    Server();
    // TODO add parameters that encapsulate the action
	bool send_action(std::vector<unsigned char> data, unsigned char id);    // Sends action to client with data body, returns whether action was successful or not
	bool send_action(unsigned char id);    // Sends action to client without data body, returns whether action was successful or not
	void send_watchdog(); // Sends watchdog so this client isn't kicked out
	void stop(); // Stops socket and cleans up

  private:
	std::vector<unsigned char> current_time();	// Stores unix timestamp in 4 bytes
    // RoverPathfinding::Controller controller;
};
} // namespace RoverPathfinding

#endif
