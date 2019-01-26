#ifndef ROVERPATHFINDING_Map_H
#define ROVERPATHFINDING_Map_H

#include "utils.hpp" // TODO include this outside as this shouldn't be in here
#include <vector>
#include <utility>

namespace RP
{

struct node
{
  using edge = std::pair<int, float>; // The index of a neighboring node in nodes and its distance
  int prev;
  float dist_to;
  point coord;
  std::vector<edge> connection;
};

struct obstacle
{
  bool marked; // indicates whether this obstacle has already been accounted for in build_graph()
  point coord1;
  point coord2;
  std::pair<int, int> &side_safety_nodes;
  int center_safety_node;
};
class Map
{
public:
  Map() { nodes.resize(2); }
  void add_obstacle(point coord1, point coord2); //Adds an obstacle to the map. Obstacle is specified with 2 points
  point compute_goal();                          // find the shortest path to the goal and return a target direction vector.
  point compute_search();                        // search for the tennis ball once the goal is reached. Return a target direction vector.
  std::vector<point> shortest_path_to(float cur_lat, float cur_lng,
                                      float tar_lat, float tar_lng); //Returns a std::vector of lat/lng pairs that specifies the shortest path to the target destination (for debugging)
  std::vector<point> a_star_algorithm(float cur_lat, float cur_lng,
									  float tar_lat, float tar_lng); //Returns a vector of lat/lng pairs for the shortest path to the target location, determined by A* algorithm
private:
  line add_length_to_line_segment(point p, point q, float length); //Returns a pair of points that are "length" away from the ends of segment pq
  void add_edge(int n1, int n2);                                   //Adds an edge to the graph
  int create_node(point coord);                                    //Creates a node. Returns index in nodes of the created node
  std::vector<node> build_graph(point cur, point tar);             //Builds the graph using the obstacles so that the shortest path gets calculated
  std::vector<node> nodes;                                         //The nodes to the graph
  std::vector<obstacle> obstacles;                                 //The obstacles
};
} // namespace RP

#endif
