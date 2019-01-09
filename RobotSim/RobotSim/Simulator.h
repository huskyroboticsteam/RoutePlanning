#ifndef ROVERPATHFINDING_SIMULATOR_H
#define ROVERPATHFINDING_SIMULATOR_H

#include <list>
#include <string>
#include "../../src/utils.h"
#include "../../src/Map.h"

#define MAX_GRID_RESOLUTION 100 // max number of grid cells on each side of the map

namespace RoverPathfinding
{
struct simulator_config
{
  float vision_angle; //Angle of FOV in degrees
  float vision_dist;  //Distance of vision in meters
};

struct sim_obstacle
{
  point p;
  point q;
  std::list<RoverPathfinding::point> endpoints;
};

class Simulator
{
public:
  Simulator(const std::string &map_path, float init_bearing, simulator_config conf);
  Simulator();
  void load_map(const std::string &path);
  std::list<line> visible_obstacles() { return view_obstacles; };
  void update_agent();
  //getters
  float get_bearing();
  point get_pos();
  //setters
  void set_bearing(float brng);
  void set_pos(float x, float y);

private:
  std::vector<point> intersection_with_arc(const point &p, const point &q, const point &lower_point, const point &upper_point);
  bool out_of_view(const point &pt);

  Map map;
  point cur_pos;
  float bearing;  //Bearing in degrees counterclockwise from the positive x-axis
  float lower_vis; //Bearing subtracted by half of vision_angle (lower bound of FoV) for caching
  float upper_vis; //Bearing added by half of vision_angle (upper bound of FoV)
  point target_pos;
  simulator_config config;
  std::list<sim_obstacle> sim_obstacles;
  std::list<line> view_obstacles;
};
} // namespace RoverPathfinding

#endif