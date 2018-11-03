/*
Controller class for decoding input from obstacle detection and sensors
to be used for computations in Map and Agent, and encoding the output
(i.e. suggested direction, speed, etc.) to be fed back to the drivetrain.
*/

#ifndef ROVERPATHFINDING_CONTROLLER_H
#define ROVERPATHFINDING_CONTROLLER_H

#include "Map.h"
#include "Agent.h"

namespace RoverPathfinding
{
class Controller
{
  private:
    RoverPathfinding::Map map;
    RoverPathfinding::Agent agent;
};
} // namespace RoverPathfinding

#endif