/*
Controller class for getting the input from obstacle detection and sensors
to be used for computations in Map, and passing the output to the server
(i.e. suggested direction, speed, etc.) to be fed back to the drivetrain.
Basically this class handles everything: model/logic (Map), networking
(Server), and view.
*/

#ifndef ROVERPATHFINDING_CONTROLLER_H
#define ROVERPATHFINDING_CONTROLLER_H

#include "Map.h"
#include "Server.h"

namespace RoverPathfinding
{
class Controller
{
  public:
    Controller(RoverPathfinding::Map map, RoverPathfinding::Server serv);
  private:
    // TODO might take some parameters
    void update();                    // Update the current world state.

    RoverPathfinding::Map map;    // The agent that handles the logic of finding a path         
    RoverPathfinding::Server server;  // The server that handles networking and triggers
};
} // namespace RoverPathfinding

#endif