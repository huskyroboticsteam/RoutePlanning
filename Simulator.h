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
  float vision_angle;     // Total angle of vision in degrees
  float vision_depth;     // Distance of vision in meters
};

class Simulator
{
  typedef unsigned int grid_size_type;

public:
  Simulator(const std::string &grid_path, const Map &map, float init_bearing, simulator_config conf);
  Simulator(const Map &map) : Simulator("test_map.txt", agt, 0, simulator_config{45, 10}){};
  void load_map(const std::string &path);
  std::string as_str();

private:
  void update_agent();

  bool grid[MAX_GRID_RESOLUTION][MAX_GRID_RESOLUTION];
  grid_size_type rows;
  grid_size_type cols;
  Map map;
  point cur_pos;
  point target_pos;
  simulator_config config;
  float bearing;    // Bearing in degrees counterclockwise from the positive x-axis
};
} // namespace RoverPathfinding

#endif