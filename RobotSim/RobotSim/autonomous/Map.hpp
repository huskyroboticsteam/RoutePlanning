#ifndef ROVERPATHFINDING_Map_H
#define ROVERPATHFINDING_Map_H

#include <vector>
#include <utility>
#include <memory>
#include "utils.hpp" // TODO include this outside as this shouldn't be in here
#include "timer.hpp"
#include "memorizer.hpp"

namespace RP
{
struct edge
{
  int parent;
  int child;
  float len;
};

typedef std::shared_ptr<edge> eptr;

struct node
{
  int prev;
  float dist_to;
  point coord;
  std::vector<eptr> connection;
};

class Map
{
public:
  Map(const point& cur_pos, const point& target, const Memorizer& mem, float bot_width=0.f);
  const std::vector<node>& d_nodes = nodes;
  point compute_next_point();                    // return coordinate for next point to go to
  point compute_search();                        // search for the tennis ball once the goal is reached. Return a target direction vector.
  std::vector<point> shortest_path_to(); //Returns a std::vector of meter coordinates that specifies the shortest path to the target destination (for debugging)
  // void reassign_target(const point& target) { tar = target; }

  void breakpoint() { debugging = true; }
  const float bot_width;
  const point& cur;
  const point& tar;
  const Memorizer& memorizer;
  
private:
  line add_length_to_line_segment(point p, point q, float length); //Returns a pair of points that are "length" away from the ends of segment pq
  RP::eptr add_edge(int parent, int child);                                   //Adds an edge to the graph; returns parent=>child edge pointer
  void remove_edge(int parent, int child);
  int create_node(point coord);                                    //Creates a node. Returns index in nodes of the created node
  std::vector<node> build_graph(point cur, point tar, float tol);             //Builds the graph using the obstacles so that the shortest path gets calculated
  std::vector<node> nodes;                                         //The nodes to the graph
  Timer timer; // for debugging
  inline point& nd_coord(int node) { return nodes[node].coord; }
  int get_closest_obstacle(eptr edge, float path_width, const std::vector<line>& obstacles); // return index of closest obstacle that intersects with edge. If doesn't exist return -1
  point ind_to_coord(int i);
  void prune_path(std::vector<int>& path, float tol);

  bool debugging = false;
}; 
} // namespace RP

#endif