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
	void send(unsigned char data, unsigned char id);    // Sends action to client
    // TODO is this event driven? if it is, we need to add event trigger here and event handler in Controller.
  private:
	std::vector<unsigned char> current_time();	// Stores unix timestamp in 4 bytes
    // RoverPathfinding::Controller controller;
};
} // namespace RoverPathfinding

#endif