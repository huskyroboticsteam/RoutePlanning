#include "quadMapper.hpp"
#include <queue>
#include <cassert>

// get code from https://geidav.wordpress.com/2017/12/02/advanced-octrees-4-finding-neighbor-nodes/
// kinda know how it works but not didn't exactly go into it -gary
RP::pqtree RP::QTreeNode::get_neighbor_ge(Direction dir)
{
    pqtree nd;
    switch (dir)
    {
    case UP:
        if (!parent)
            return pqtree(nullptr);

        if (parent->botleft.get() == this)
            return parent->topleft;

        if (parent->botright.get() == this)
            return parent->topright;

        nd = parent->get_neighbor_ge(dir);
        if (!nd || nd->is_leaf)
            return nd;

        // this must be a top child
        return parent->topleft.get() == this ? nd->botleft : nd->botright;
    case DOWN:
        if (!parent)
            return pqtree(nullptr);

        if (parent->topleft.get() == this)
            return parent->botleft;

        if (parent->topright.get() == this)
            return parent->botright;

        nd = parent->get_neighbor_ge(dir);
        if (!nd || nd->is_leaf)
            return nd;

        return parent->botleft.get() == this ? nd->topleft : nd->topright;
    case LEFT:
        if (!parent)
            return pqtree(nullptr);

        if (parent->topright.get() == this)
            return parent->topleft;

        if (parent->botright.get() == this)
            return parent->botleft;

        nd = parent->get_neighbor_ge(dir);
        if (!nd || nd->is_leaf)
            return nd;

        return parent->topleft.get() == this ? nd->topright : nd->botright;
    case RIGHT:
        if (!parent)
            return pqtree(nullptr);

        if (parent->topleft.get() == this)
            return parent->topright;

        if (parent->botleft.get() == this)
            return parent->botright;

        nd = parent->get_neighbor_ge(dir);
        if (!nd || nd->is_leaf)
            return nd;

        return parent->topright.get() == this ? nd->topleft : nd->botleft;
    default:
        printf("WARNING: unrecognized direction argument in QTreeNode::get_neighbor_ge()\n");
        return nullptr;
    }
}

RP::pqtree RP::QuadMapper::create_qtnode(float minx, float miny, float maxx, float maxy, int lvl)
{
    pqtree created = pqtree(new QTreeNode(minx, miny, maxx, maxy, lvl, qtnodes.size()));
    qtnodes.push_back(created);
    return created;
}

RP::QuadMapper::QuadMapper(const point &cur_pos, const point &target, const std::vector<line> &allobst,
                           float fwidth, float fheight, int max_d, float tolerance) : Mapper(cur_pos, target, tolerance, allobst),
                                                                                      max_depth(max_d), field_width(fwidth), field_height(fheight), new_tnode_added(true)
{
    root = create_qtnode(0, 0, field_width, field_height, 1);
}

void RP::QuadMapper::set_pos(point c)
{
    cur = c;
    cur_changed = true;
}

void RP::QuadMapper::set_tar(point t)
{
    tar = t;
    tar_changed = true;
}

void RP::QuadMapper::set_tol(float t)
{
    tol = t;
    qtnodes.clear();
    root = create_qtnode(0, 0, field_width, field_height, 1);
    new_obstacles(all_obstacles);
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
                        new_tnode_added = true;
                        // split node
                        float mid_x = (nd->min_x + nd->max_x) / 2;
                        float mid_y = (nd->min_y + nd->max_y) / 2;
                        int next_depth = nd->depth + 1;
                        pqtree bl = create_qtnode(nd->min_x, nd->min_y, mid_x, mid_y, next_depth);
                        nd->botleft = bl;
                        nd->botleft->parent = nd;
                        nd->botright = create_qtnode(mid_x, nd->min_y,
                                                     nd->max_x, mid_y, next_depth);
                        nd->botright->parent = nd;
                        nd->topleft = create_qtnode(nd->min_x, mid_y,
                                                    mid_x, nd->max_y, next_depth);
                        nd->topleft->parent = nd;
                        nd->topright = create_qtnode(mid_x, mid_y, nd->max_x,
                                                     nd->max_y, next_depth);
                        nd->topright->parent = nd;
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

RP::pqtree RP::QuadMapper::get_enclosing_node(point coord) const
{
    if (coord.x > root->max_x || coord.x < root->min_x || coord.y > root->max_y ||
        coord.y < root->min_y)
    {
        printf("WARNING: coord not enclosed in root node in QuadMapper.\n");
        return pqtree(nullptr);
    }

    pqtree cn = root;
    while (!cn->is_leaf)
    {
        if (cn->is_blocked)
        {
            printf("WARNING: coord enclosed in blocked node in QuadMapper.\n");
            return pqtree(nullptr);
        }
        if (coord.x < cn->center_coord.x)
        {
            if (coord.y < cn->center_coord.y)
                cn = cn->botleft;
            else
                cn = cn->topleft;
        }
        else
        {
            if (coord.y < cn->center_coord.y)
                cn = cn->botright;
            else
                cn = cn->topright;
        }
    }
    return cn;
}

void RP::QuadMapper::make_path_graph()
{
    mygraph.clear();
    mygraph.create_node(cur);
    mygraph.nodes[0].dist_to = 0.f;
    mygraph.create_node(tar);

    // copy quadtree nodes to graph
    // optimize by skipping this step if necessary
    size_t nqt = qtnodes.size();
    for (auto it = qtnodes.begin(); it != qtnodes.end(); it++)
        mygraph.create_node((*it)->center_coord);

    bool *visited = new bool[nqt]();
    for (size_t i = 0; i < nqt; i++)
    {
        if (!qtnodes[i]->is_leaf || qtnodes[i]->is_blocked)
            continue;
        for (Direction d : {UP, DOWN, LEFT, RIGHT})
        {
            pqtree n = qtnodes[i]->get_neighbor_ge(d);
            if (!n || !n->is_leaf || n->is_blocked)
                continue;
            if (!visited[n->id] || n->depth < qtnodes[i]->depth)
            {
                // +2 for offset created by cur and tar
                mygraph.add_edge(i + 2, n->id + 2);
            }
        }
        visited[i] = true;
    }

    pqtree cur_node = get_enclosing_node(mygraph.nodes[0].coord);
    pqtree tar_node = get_enclosing_node(mygraph.nodes[1].coord);
    if (!cur_node || !tar_node)
        return;
    mygraph.add_edge(0, cur_node->id + 2);
    mygraph.add_edge(1, tar_node->id + 2);

    delete[] visited;
}

RP::pqtree RP::QuadMapper::get_qtree_root() const
{
    return root;
}

bool RP::QuadMapper::obs_in_node(const line &obs, pqtree tnode)
{
    point placeholder;
    return seg_intersects_rect(obs, tnode->sides, placeholder);
}

void RP::QuadMapper::compute_graph()
{
    // if more performance needed, modify path graph as new obstacles are added
    // instead of remaking it
    if (new_tnode_added)
    {
        make_path_graph();
        new_tnode_added = false;
    }

    if (cur_changed)
    {
        cur_changed = false;
        mygraph.nodes[0].connection.clear();
        pqtree cur_node = get_enclosing_node(mygraph.nodes[0].coord);
        if (cur_node)
            mygraph.add_edge(0, cur_node->id + 2);
    }

    if (tar_changed)
    {
        tar_changed = false;
        mygraph.nodes[1].connection.clear();
        pqtree tar_node = get_enclosing_node(mygraph.nodes[1].coord);
        if (tar_node)
            mygraph.add_edge(1, tar_node->id + 2);
    }
}

bool RP::QuadMapper::path_good(int node1, int node2, float tol) const
{
    line path_seg{mygraph.nodes[node1].coord, mygraph.nodes[node2].coord};
    point dont_care;
    std::queue<pqtree> q;
    q.push(root);
    while (!q.empty())
    {
        pqtree nd = q.front();
        q.pop();
        if (seg_intersects_rect(path_seg, nd->sides, dont_care))
        {
            if (nd->is_blocked)
                return false;
            if (!nd->is_leaf)
            {
                q.push(nd->topleft);
                q.push(nd->topright);
                q.push(nd->botleft);
                q.push(nd->botright);
            }
        }
    }
    return true;
}
