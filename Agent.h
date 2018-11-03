#ifndef ROVERPATHFINDING_AGENT_H
#define ROVERPATHFINDING_AGENT_H

#include "Map.h"

namespace RoverPathfinding
{
class Agent
{
  public:
    void bind(RoverPathfinding::Map map);           // bind this agent to a map instance.
    std::pair<float, float> compute_goal();         // find the shortest path to the goal and return a target direction vector.
    std::pair<float, float> compute_search();       // search for the tennis ball once the goal is reached. Return a target direction vector.
  private:
};
} // namespace RoverPathfinding

#endif