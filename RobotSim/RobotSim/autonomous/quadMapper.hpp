// a more computationally expensive, but potentially more robust in more maze-y situations, alternative to Map
// also this could actually be less expensive than map for really cramped maps
/*
TODOs
1) split Map into two files: one is the Mapper, which converts obstacle coordinates to a graph; the other is
a pather, which finds a path within the graph. Also possibly isolate the obstacle merging part
2) implement inheritance. rename Map to SplitMap or ContinuousMap or something, and then make this and SplitMap inherit
from a new abtract Map.cpp
*/

/*
Implementation notes:
Use the QuadTree data structure, using the following splitting algorithm:

    func make_quadtree(AABB):
        if area of AABB < epsilon OR no obstacles within AABB:  # note there is some tolerance value here
            return

        foreach quadrant within AABB:
            make_quadtree(quadrant)

The center of each leaf node is a node in the final graph.

Things I haven't figured out:
- how to efficiently connect start/end node with a quadtree node (should be a search algorithm)
- how to connect node with neighbors

*/
#ifndef RP_DISCRETEMAPPER_HPP
#define RP_DISCRETEMAPPER_HPP

#include <memory>
#include "mapper.hpp"
#include "utils.hpp"

namespace RP
{
struct QTreeNode;
typedef std::shared_ptr<QTreeNode> pqtree;
struct QTreeNode
{
    QTreeNode(float minx, float miny, float maxx, float maxy, int d) : depth(d), min_x(minx), min_y(miny),
    max_x(maxx), max_y(maxy), is_leaf(true), is_blocked(false), topleft(pqtree(nullptr)), topright(pqtree(nullptr)),
    botleft(pqtree(nullptr)), botright(pqtree(nullptr))
    {
      sides[0] = line{point{min_x, min_y}, point{max_x, min_y}};
      sides[1] = line{point{max_x, min_y}, point{max_x, max_y}};
      sides[2] = line{point{max_x, max_y}, point{min_x, max_y}};
      sides[3] = line{point{min_x, max_y}, point{min_x, min_y}};
    }
    int depth;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
    bool is_leaf;
    bool is_blocked;  // this only matters if obstacle is leaf
    pqtree topleft;
    pqtree topright;
    pqtree botleft;
    pqtree botright;
    line sides[4];
};
pqtree make_qtnode(float minx, float miny, float maxx, float maxy, int depth);

class QuadMapper : public Mapper
{
  public:
    const pqtree& qtree_root = root;
    QuadMapper(const point &cur_pos, const point &target, const std::vector<line> &all_obstacles,
    float field_width, float field_height, int max_depth = 6, float tolerance = 0.f);

    virtual void set_pos(point pos) override;
    virtual void set_tar(point tar) override;
    virtual void set_tol(float tol) override;
    virtual void new_obstacles(const std::vector<line> &obstacles) override;
    // graph methods used for pather
    virtual graph get_graph() override;
    virtual bool path_good(int node1, int node2, float tol) const override;

    void set_max_depth(float md);
  private:
    int max_depth;
    pqtree root;

    // returns true if obstacle is inside node (i.e. intersects with the square area)
    bool obs_in_node(const line& obs, pqtree tnode);
};
} // namespace RP
#endif
