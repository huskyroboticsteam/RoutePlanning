// handles all the pathing logic. has member Mapper that creates graph and possibly members (or methods)
// that handle obstalce merging and pathfinding within graph

// a good idea is to switch between Continuous and Discrete mapping based on the number and density of
// obstacles
#ifndef RP_PATHER_HPP
#define RP_PATHER_HPP

#include "splitMapper.hpp"
#include "quadMapper.hpp"
#include "memorizer.hpp"

namespace RP
{
class Pather
{
  public:
    Pather(point origin, point target, point max_point, float tol);
    const point& cur_point = cur;
    const point& tar_point = tar;
    std::vector<point> compute_path();
    point compute_next_point();
    const std::vector<line>& mem_obstacles() const;
    void add_obstacles(const std::vector<line> &obstacles);
    const graph& d_graph() const;
    void set_pos(const point& pos);
    void set_tar(const point& tar);

    pqtree debug_qtree_root() const { return fineMapper.qtree_root; }

  private:
    Memorizer memorizer;
    SplitMapper roughMapper;
    QuadMapper fineMapper;
    Mapper& curMapper;
    point cur;
    point tar;
    float tol;
    point max_pt;

    void prune_path(std::vector<int> &path, float tol);
};
} // namespace RP

#endif