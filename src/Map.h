#ifndef ROVERPATHFINDING_Map_H
#define ROVERPATHFINDING_Map_H

#include "utils.h" // TODO include this outside as this shouldn't be in here
#include <vector>
#include <utility>

namespace RoverPathfinding
{
namespace
{
struct node
{
  int prev;
  float dist_to;
  point coord;
  std::vector<std::pair<int, float>> connection;
};

struct obstacle
{
  bool marked; // indicates whether this obstacle has already been accounted for in build_graph()
  point coord1;
  point coord2;
  std::pair<int, int> side_safety_nodes;
  int center_safety_node;
};
} // namespace
class Map
{
public:
  Map() { nodes.resize(2); }
  void add_obstacle(point coord1, point coord2); //Adds an obstacle to the map. Obstacle is specified with 2 points
  point compute_goal();                          // find the shortest path to the goal and return a target direction vector.
  point compute_search();                        // search for the tennis ball once the goal is reached. Return a target direction vector.
  std::vector<point> shortest_path_to(float cur_lat, float cur_lng,
                                      float tar_lat, float tar_lng); //Returns a std::vector of lat/lng pairs that specifies the shortest path to the target destination (for debugging)
private:
  std::pair<point, point> add_length_to_line_segment(point p, point q, float length); //Returns a pair of points that are "length" away from the ends of segment pq
  void add_edge(int n1, int n2);                                                      //Adds an edge to the graph
  int create_node(point coord);                                                       //Creates a node. Returns index in nodes of the created node
  std::vector<node> build_graph(point cur, point tar);                                //Builds the graph using the obstacles so that the shortest path gets calculated
  std::vector<node> nodes;                                                            //The nodes to the graph
  std::vector<obstacle> obstacles;                                                    //The obstacles
};
} // namespace RoverPathfinding

#endif