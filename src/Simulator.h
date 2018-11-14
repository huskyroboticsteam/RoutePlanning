#ifndef ROVERPATHFINDING_SIMULATOR_H
#define ROVERPATHFINDING_SIMULATOR_H

#include <vector>
#include <string>
#include "Map.h"

#define MAX_GRID_RESOLUTION 100 // max number of grid cells on each side of the map

namespace RoverPathfinding
{
struct simulator_config
{
  float vision_angle;     // Angle of FOV in degrees
  float vision_dist;     // Distance of vision in meters
};

struct line
{
  point p;
  point q;
};

class Simulator
{

public:
  Simulator(const std::string &grid_path, const Map &map, float init_bearing, simulator_config conf);
  Simulator(const Map &map) : Simulator("test_map.txt", map, 0.f, simulator_config{45, 10}){};
  void load_map(const std::string &path);

private:
  void update_agent();
  bool out_of_view(point pt);

  Map map;
  point cur_pos;
  float cur_brng;    // Bearing in degrees counterclockwise from the positive x-axis
  point target_pos;
  simulator_config config;
  std::vector<line> obstacles;
};
} // namespace RoverPathfinding

#endif