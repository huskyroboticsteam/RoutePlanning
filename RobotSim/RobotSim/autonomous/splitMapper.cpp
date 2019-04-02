#include <queue>
#include <cmath>
#include <algorithm>
#include <list>
#include <cassert>
#include "splitMapper.hpp"
#include "memorizer.hpp"

RP::SplitMapper::SplitMapper(const point &orig, const point &tget, const std::vector<line> &allobst, float tolr) :
Mapper(orig, tget, tolr, allobst), need_rebuild(false)
{
}

// RP::point RP::SplitMapper::compute_next_point()
// {
//     return compute_path().front();
// }

void RP::SplitMapper::set_pos(point pos)
{
    if (!same_point(cur, pos, 1e-4))
    {
        need_rebuild = true;
        cur = pos;
    }
}

void RP::SplitMapper::set_tar(point t)
{
    if (!same_point(t, tar, 1e-4))
    {
        need_rebuild = true;
        tar = t;
    }
}

void RP::SplitMapper::set_tol(float t)
{
    if (abs(tol - t) > 1e-4)
    {
        need_rebuild = true;
        tol = t;
    }
}

void RP::SplitMapper::new_obstacles(const std::vector<line> &new_obst)
{
    need_rebuild = true;
}

RP::graph RP::SplitMapper::get_graph()
{
    if (need_rebuild)
        rebuild_graph(tol);

    return mygraph;
}

RP::line RP::SplitMapper::add_length_to_line_segment(point p, point q, float length)
{
    std::pair<float, float> pq = std::make_pair(q.x - p.x, q.y - p.y); //vector
    float pq_len = sqrt(pq.first * pq.first + pq.second * pq.second);
    pq.first = length * pq.first / pq_len;
    pq.second = length * pq.second / pq_len;

    point p1 = point{q.x + pq.first, q.y + pq.second};
    point p2 = point{p.x - pq.first, p.y - pq.second};
    return line{p1, p2};
}

void RP::SplitMapper::rebuild_graph(float side_tolerance)
{
    //TODO(sasha): make R a constant - the following few lines are just a hack
    //             to get R to be in lat/lng units
    //<hack>
    // shouldn't need this since we're passing meters
    // #define R_METERS 0.5f
    //     auto offset = RP::lat_long_offset(cur.x, cur.y, 0.0f, R_METERS);
    //     auto diff = std::make_pair(offset.x - cur.x, offset.y - cur.y);
    //     float SIDE_TOLERANCE = sqrt(diff.first * diff.first + diff.second * diff.second);
    // #undef R_METERS
    //</hack>

    mygraph.clear();
    mygraph.create_node(cur);
    mygraph.nodes[0].dist_to = 0.0f;
    mygraph.create_node(tar);
    eptr init_edge = mygraph.add_edge(0, 1);
    if (all_obstacles.empty())
    {
        return;
    }

    // if obstacle has been visited from one side
    std::vector<bool> visited(all_obstacles.size(), false);
    std::queue<eptr> unprocessed_edges;
    // add first edge
    unprocessed_edges.push(init_edge);

    // pad for curr and parent
    // for each safety node, find the obstacle closest to it
    while (!unprocessed_edges.empty())
    {
        const eptr curr_edge = unprocessed_edges.front();
        unprocessed_edges.pop();
        int closest_index = get_closest_obstacle(curr_edge, tol, all_obstacles);

        if (closest_index != -1)
        {
            const line &closest = all_obstacles.at(closest_index);
            mygraph.remove_edge(curr_edge->parent, curr_edge->child);
            // printf("removing %f, %f - %f, %f\n", nodes.at(curr_edge->parent).coord.x, nodes.at(curr_edge->parent).coord.y,
            // nodes.at(curr_edge->child).coord.x, nodes.at(curr_edge->child).coord.y);
            if (!visited[closest_index])
            {
                // make copies of endponts
                point end1 = closest.p;
                point end2 = closest.q;
                line closer = add_length_to_line_segment(end1, end2, side_tolerance);
                line farther = add_length_to_line_segment(end1, end2, side_tolerance);

                point par = nd_coord(curr_edge->parent);
                point chd = nd_coord(curr_edge->child);
                int opar = orientation(closer.p, closer.q, par);
                int ochd = orientation(farther.p, farther.q, chd);

                if (opar == 0 || ochd == 0 || opar == ochd)
                {
                    closer = get_moved_line(closer, side_tolerance, true);
                    farther = get_moved_line(farther, side_tolerance, false);
                }
                else
                {
                    closer = get_moved_line(closer, side_tolerance, opar == COUNTERCLOCKWISE);
                    farther = get_moved_line(farther, side_tolerance, ochd == COUNTERCLOCKWISE);
                }

                // create safety nodes
                int branch1closer = mygraph.create_node(closer.p);
                int branch1farther = mygraph.create_node(farther.p);
                unprocessed_edges.push(mygraph.add_edge(curr_edge->parent, branch1closer));
                unprocessed_edges.push(mygraph.add_edge(branch1closer, branch1farther));
                unprocessed_edges.push(mygraph.add_edge(branch1farther, curr_edge->child));

                int branch2closer = mygraph.create_node(closer.q);
                int branch2farther = mygraph.create_node(farther.q);
                unprocessed_edges.push(mygraph.add_edge(curr_edge->parent, branch2closer));
                unprocessed_edges.push(mygraph.add_edge(branch2closer, branch2farther));
                unprocessed_edges.push(mygraph.add_edge(branch2farther, curr_edge->child));

                // special case for "vertical" obstacles
                unprocessed_edges.push(mygraph.add_edge(branch1closer, branch2closer));
                unprocessed_edges.push(mygraph.add_edge(branch1farther, branch2farther));

                // set endpoints associated with obstacle as visited
                visited.at(closest_index) = true;
            }
            else
            {
                /*
                printf("visited (%f, %f) - (%f, %f)\n", obstacles[closest_index].p.x, 
                    obstacles[closest_index].p.y, obstacles[closest_index].q.x,
                obstacles[closest_index].q.y);
                */
            }
        }
    }
}

int RP::SplitMapper::get_closest_obstacle(eptr edge, float path_width, const std::vector<line> &obstacles) const
{
    float min_dist = INFINITY;
    int closest_index = -1;
    for (int i = 0; i < obstacles.size(); i++)
    {
        const auto &obst = obstacles.at(i);
        point inters;
        if (seg_intersects_width(nd_coord(edge->parent),
                                 nd_coord(edge->child), obst.p, obst.q, path_width, inters))
        {
            float dist = dist_sq(inters, nd_coord(edge->parent));
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = i;
            }
        }
    }
    return closest_index;
}

bool RP::SplitMapper::path_good(int node1, int node2, float tol) const
{
    auto e = eptr(new edge{node1, node2});
    return get_closest_obstacle(e, tol, all_obstacles) == -1;
}
