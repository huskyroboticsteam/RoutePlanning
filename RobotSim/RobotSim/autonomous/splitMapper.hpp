#ifndef ROVERPATHFINDING_SPLITMAPPER_H
#define ROVERPATHFINDING_SPLITMAPPER_H

#include <vector>
#include <utility>
#include <memory>
#include "utils.hpp" // TODO include this outside as this shouldn't be in here
#include "timer.hpp"
#include "mapper.hpp"

namespace RP
{
class SplitMapper : public Mapper
{
public:
  SplitMapper(const point &cur_pos, const point &target, const std::vector<line> &all_obstacles, float tolr = 0.f);

  virtual void set_pos(point pos) override;
  virtual void set_tar(point tar) override;
  virtual void set_tol(float tol) override;
  virtual void new_obstacles(const std::vector<line>& obstacles) override;
  // graph methods used for pather
  virtual void compute_graph() override;
  virtual bool path_good(int node1, int node2, float tol) const override;

private:
  bool need_rebuild;
  // TODO make this override
  void rebuild_graph(float tol); //Builds the graph using the obstacles so that the shortest path gets calculated
  Timer timer;                                                     // for debugging
  inline const point &nd_coord(int node) const { return mygraph.nodes[node].coord; }
  // return index of closest obstacle that intersects with edge. If doesn't exist return -1
  int get_closest_obstacle(eptr edge, float path_width, const std::vector<line> &obstacles) const;

  bool debugging = false;
};
} // namespace RP

#endif