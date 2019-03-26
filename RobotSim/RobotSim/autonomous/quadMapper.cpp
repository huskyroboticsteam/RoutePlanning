#include "quadMapper.hpp"
#include <queue>

RP::pqtree RP::make_qtnode(float minx, float miny, float maxx, float maxy, int lvl)
{
    return pqtree(new QTreeNode(minx, miny, maxx, maxy, lvl));
}

RP::QuadMapper::QuadMapper(const point &cur_pos, const point &target, const std::vector<line> &all_obstacles,
                           float field_width, float field_height, int max_d, float tolerance) :
                           Mapper(cur_pos, target, tolerance), max_depth(max_d)
{
    root = make_qtnode(0, 0, field_width, field_height, 1);
}

void RP::QuadMapper::set_pos(point c)
{
    // TODO finish set_pos
    cur = c;
}

void RP::QuadMapper::set_tar(point t)
{
    // TODO finish set_tar
    tar = t;
}

void RP::QuadMapper::set_tol(float t)
{
    // TODO finish set_tol
    tol = t;
}

void RP::QuadMapper::new_obstacles(const std::vector<line> &obstacles)
{
    std::queue<pqtree> q;
    for (const line &obs : obstacles)
    {
        q.push(root);
        while (!q.empty())
        {
            pqtree nd = q.front();
            q.pop();

            if (obs_in_node(obs, nd))
            {
                // if depth limit reahed, don't split anymore
                if (nd->depth >= max_depth)
                {
                    nd->is_blocked = true;
                }
                else
                {
                    if (nd->is_leaf)
                    {
                        nd->is_leaf = false;

                        // split node
                        float mid_x = (nd->min_x + nd->max_x) / 2;
                        float mid_y = (nd->min_y + nd->max_y) / 2;
                        int next_depth = nd->depth + 1;
                        nd->botleft = make_qtnode(nd->min_x, nd->min_y,
                                                  mid_x, mid_y, next_depth);
                        nd->botright = make_qtnode(mid_x, nd->min_y,
                                                   nd->max_x, mid_y, next_depth);
                        nd->topleft = make_qtnode(nd->min_x, mid_y,
                                                  mid_x, nd->max_y, next_depth);
                        nd->topright = make_qtnode(mid_x, mid_y, nd->max_x,
                                                   nd->max_y, next_depth);
                    }
                    q.push(nd->botleft);
                    q.push(nd->botright);
                    q.push(nd->topleft);
                    q.push(nd->topright);
                }
            }
        }
    }
}

bool RP::QuadMapper::obs_in_node(const line &obs, pqtree tnode)
{
    point placeholder;
    return seg_intersects_rect(obs, tnode->sides, placeholder);
}

RP::graph RP::QuadMapper::get_graph()
{
    printf("get graph not implemented for QuadMapper\n");
    return graph{};
}

bool RP::QuadMapper::path_good(int node1, int node2, float tol) const
{
    return false;
}
