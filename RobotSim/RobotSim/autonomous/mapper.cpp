#include "mapper.hpp"
#include <cassert>
#include <cmath>

RP::Mapper::Mapper(point origin, point target, float tolerance) : cur(origin), tar(target), tol(tolerance)
{

}

RP::eptr RP::graph::add_edge(int parent, int child)
{
    float dist = sqrt(dist_sq(nodes[parent].coord, nodes[child].coord));

    eptr p_to_c = eptr(new edge{parent, child, dist});
    nodes[parent].connection.push_back(p_to_c);

    eptr c_to_p = eptr(new edge{child, parent, dist});
    nodes[child].connection.push_back(c_to_p);

    return p_to_c;
}

void RP::graph::remove_edge(int parent, int child)
{
    assert(parent >= 0 && child >= 0);
    auto &conn = nodes.at(parent).connection;
    bool found = false;
    for (int i = 0; i < conn.size(); i++)
    {
        if (conn.at(i)->child == child)
        {
            conn.erase(conn.begin() + i);
            found = true;
            break;
        }
    }
    if (!found)
        printf("WARNING: edge not removed. Parent: %d, child %d\n", parent, child);
    found = false;
    auto &conn2 = nodes.at(child).connection;
    for (int i = 0; i < conn2.size(); i++)
    {
        if (conn2.at(i)->child == parent)
        {
            conn2.erase(conn2.begin() + i);
            found = true;
            break;
        }
    }
    if (!found)
        printf("WARNING: edge not removed (reverse). Parent: %d, child %d\n", parent, child);
}

int RP::graph::create_node(point coord)
{
    node n;
    n.prev = -1;
    n.dist_to = INFINITY;
    n.coord = coord;
    nodes.push_back(n);
    return (nodes.size() - 1);
}

void RP::graph::clear()
{
    nodes.clear();
}
