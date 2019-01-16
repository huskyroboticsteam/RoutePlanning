#ifndef ROVERPATHFINDING_SIMULATOR_H
#define ROVERPATHFINDING_SIMULATOR_H

#include <list>
#include <string>
#include "Map.hpp"
#include "obstacle.hpp"
#include "agent.hpp"

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

class Simulator : public sf::Drawable
{
public:
  Simulator(const std::list<Obstacle>& obstacleList, const Agent& agent, simulator_config conf, float map_scale);
  const std::list<line>& visible_obstacles() { return view_obstacles; };
  void update_agent();

private:
  std::vector<point> intersection_with_arc(const point &p, const point &q, const point &lower_point, const point &upper_point);
  bool within_view(const point &pt);
  float scale; // scale is only used when calling draw
  Map map;
  const Agent& agent;

  // for computations; not actual attributes of the system
  point cur_pos; //updated every cal to update_agent()
  float lower_vis; //Bearing subtracted by half of vision_angle (lower bound of FoV) for caching
  float upper_vis; //Bearing added by half of vision_angle (upper bound of FoV)
  float vision_dist_sq;
  point fov_lower;
  point fov_upper;

  point target_pos;
  simulator_config config;
  const std::list<Obstacle>& sim_obstacles;
  std::list<line> view_obstacles;

  virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const;
};
} // namespace RoverPathfinding

#endif