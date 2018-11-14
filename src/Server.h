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
    bool send_action_response();    // Encode the best action as bytes and send to the client. Return true if succeeded and false if not.
    // TODO is this event driven? if it is, we need to add event trigger here and event handler in Controller.
  private:

    RoverPathfinding::Controller controller;
};
} // namespace RoverPathfinding

#endif