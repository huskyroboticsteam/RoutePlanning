#include <queue>
#include <cmath>
#include <algorithm>
#include <list>
#include <cassert>
#include "Map.hpp"

RP::Map::Map(const point &cpos, const point &tget) : cur(cpos), tar(tget)
{
}

void RP::Map::add_obstacle(point coord1, point coord2)
{
    // obstacle o{};
    obstacles.push_back(obstacle{false, coord1, coord2});
}

RP::point RP::Map::compute_next_point() 
{
    return shortest_path_to().front();
}

RP::line RP::Map::add_length_to_line_segment(point p, point q, float length)
{
    std::pair<float, float> pq = std::make_pair(q.x - p.x, q.y - p.y); //vector
    float pq_len = sqrt(pq.first * pq.first + pq.second * pq.second);
    pq.first = length * pq.first / pq_len;
    pq.second = length * pq.second / pq_len;

    point p1 = point{q.x + pq.first, q.y + pq.second};
    point p2 = point{p.x - pq.first, p.y - pq.second};
    return line{p1, p2};
}

void RP::Map::add_edge(int n1, int n2)
{
    float dist = std::sqrt(dist_sq(nodes[n1].coord, nodes[n2].coord));

    auto n1_to_n2 = std::make_pair(n2, dist);
    nodes[n1].connection.push_back(n1_to_n2);

    auto n2_to_n1 = std::make_pair(n1, dist);
    nodes[n2].connection.push_back(n2_to_n1);
}

int RP::Map::create_node(point coord)
{
    node n;
    n.prev = -1;
    n.dist_to = INFINITY;
    n.coord = coord;
    nodes.push_back(n);
    return (nodes.size() - 1);
}

void move_line_toward_point(RP::line& side_points, RP::point cur, float dist);

std::vector<RP::node> RP::Map::build_graph(point cur, point tar)
{
    //TODO(sasha): make R a constant - the following few lines are just a hack
    //             to get R to be in lat/lng units
    //<hack>
    #define SIDE_TOLERANCE 1.7f
    #define CENTER_TOLERANCE 1.5f
    // shouldn't need this since we're passing meters
// #define R_METERS 0.5f
//     auto offset = RP::lat_long_offset(cur.x, cur.y, 0.0f, R_METERS);
//     auto diff = std::make_pair(offset.x - cur.x, offset.y - cur.y);
//     float SIDE_TOLERANCE = sqrt(diff.first * diff.first + diff.second * diff.second);
// #undef R_METERS
    //</hack>
    obstacles.clear();
    obstacles.reserve(mem_obstacles.size());
    for (const auto &o : mem_obstacles)
        obstacles.emplace_back(obstacle{false, o.coord1, o.coord2});

    // printf("%d\n", obstacles.size());
    // for (auto &n : nodes)
    // {
    //     n.dist_to = INFINITY;
    //     n.connection.clear();
    // }
    nodes.clear();
    create_node(cur);
    nodes[0].dist_to = 0.0f;
    create_node(tar);
    if (obstacles.empty())
    {
        add_edge(0, 1); // probably start (origin or current?) and target nodes
        return (nodes);
    }
    
    std::vector<bool> visited(obstacles.size() * 2, false); // visited[i] and [i+1] stores whether obstacles[i/2].p or q has been visited, respectively
    std::queue<int> unprocessed_nodes;
    unprocessed_nodes.push(0);
    // for each safety node, find the obstacle closest to it
    while (!unprocessed_nodes.empty())
    {
        // printf("stuck\n");
        int curr_node = unprocessed_nodes.front();
        // printf("curr_node: %f, %f\n", nodes.at(curr_node).coord.x, nodes.at(curr_node).coord.y);
        assert(curr_node >= 0);
        unprocessed_nodes.pop();
        bool destination_blocked = false;
        int closest_obst;
        float min_dist = INFINITY;
        // find closest intersection
        for (int i = 0; i < obstacles.size(); i++)
        {
            auto &obst = obstacles[i];
            if (RP::segments_intersect(nodes[curr_node].coord, obst.coord1, tar, obst.coord2))
            {
                // printf("destination blocked by %f, %f\n", nodes[curr_node].coord.x, nodes[curr_node].coord.y);
                destination_blocked = true;
                point inters = intersection(nodes[curr_node].coord, tar, obst.coord1, obst.coord2);
                float dist = RP::dist_sq(nodes[curr_node].coord, inters);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_obst = i;
                }
            }
        }
        if (destination_blocked)
        {
            auto &obst = obstacles[closest_obst];
            int n1 = -1, n2 = -1;
            // if (!obst.marked)
            // {
                // obst.marked = true;
                // really move around the side points
                auto side_points = add_length_to_line_segment(obst.coord1, obst.coord2, SIDE_TOLERANCE);
                move_line_toward_point(side_points, cur, 1.2f);

                bool create_n1 = true;
                bool create_n2 = true;

                //TODO(sasha): if this is too slow, use a partitioning scheme to only check against
                //             nodes in the vicinity
                // for (int safety = 2; safety < nodes.size(); safety++)
                // {
                //     // if n1/n2 too closest to another endpoint, set n1/n2 to that
                //     // endpoint instead
                //     node &n = nodes[safety];
                //     if (RP::within_radius(n.coord, side_points.p, SIDE_TOLERANCE))
                //     {
                //         printf("within radius\n");
                //         create_n1 = false;
                //         n1 = safety;
                //     }
                //     if (RP::within_radius(n.coord, side_points.q, SIDE_TOLERANCE))
                //     {
                //         printf("within radius\n");
                //         create_n2 = false;
                //         n2 = safety;
                //     }
                // }
                if (!visited.at(closest_obst * 2)) {
                    n1 = create_node(side_points.p);
                    visited.at(closest_obst * 2) = true;
                    obst.side_safety_nodes.first = n1;
                }

                if (!visited[closest_obst * 2 + 1]) {
                    n2 = create_node(side_points.q);
                    visited.at(closest_obst * 2 + 1) = true;
                    obst.side_safety_nodes.second = n2;
                }

                // make robot move to somewhere near the center of the obstacle first,
                // and then move it to one of the side safety nodes
                point center_coord = RP::center_point_with_radius(nodes[curr_node].coord, side_points.p, side_points.q, CENTER_TOLERANCE);
                // printf("CENTER COORD: %f, %f\n", center_coord.x, center_coord.y);
                obst.center_safety_node = create_node(center_coord);
                add_edge(curr_node, obst.center_safety_node);
                if (n1 >= 0)
                    add_edge(n1, obst.center_safety_node);
                if (n2 >= 0)
                    add_edge(n2, obst.center_safety_node);
            // }
            // else
            // {
            //     n1 = obst.side_safety_nodes.first;
            //     n2 = obst.side_safety_nodes.second;
            // }
            if (n1 >= 0) {
                add_edge(curr_node, n1);
                unprocessed_nodes.push(n1);
            }
            if (n2 >= 0) {
                add_edge(curr_node, n2);
                unprocessed_nodes.push(n2);
            }
        }
        else
        {
            add_edge(curr_node, 1);
        }
    }
    return (nodes);
}

// return true if original and challenger are sufficiently different
// so that they should be merged
bool same_obstacle(RP::obstacle original, RP::obstacle challenger)
{
    if (RP::same_point(original.coord1, challenger.coord1))
        return RP::same_point(original.coord2, challenger.coord2);
    else
        return RP::same_point(original.coord1, challenger.coord2) &&
               RP::same_point(original.coord2, challenger.coord1);
}

void RP::Map::update(const std::list<obstacle> &new_obstacles)
{
    obstacle merged;
    // bool debug = timer.elapsed() > 1;
    bool debug = true;
    if (debug) {
        // printf("Mem Count: %d\n", mem_obstacles.size());
        // for (const obstacle& o : mem_obstacles) printf("mem (%f, %f), (%f, %f)\n", o.coord1.x, o.coord1.y, o.coord2.x, o.coord2.y);
        // printf("New Count: %d\n", new_obstacles.size());
        // for (const obstacle& o : new_obstacles) printf("new (%f, %f), (%f, %f)\n", o.coord1.x, o.coord1.y, o.coord2.x, o.coord2.y);
    }
    for (const obstacle &newobs : new_obstacles)
    {
        // printf("(%f, %f), (%f, %f)\n", newobs.coord1.x, newobs.coord1.y, newobs.coord2.x, newobs.coord2.y);
        merged = newobs;
        bool should_add = true;
        int nmerged = 0;
        for (auto it_mobs = mem_obstacles.begin(); it_mobs != mem_obstacles.end();)
        {
            bool can_merge = false;
            // if intersect/overlap
            // merge obstacles, remove vobs and don't add newobs, and add merged obstacles
            obstacle temp = merge(merged, *it_mobs, can_merge);
            if (debug) {
                // printf("merged(): m(%f, %f) (%f, %f) and n(%f, %f) (%f, %f) -> t(%f, %f), (%f, %f)\n",
                //             it_mobs->coord1.x, it_mobs->coord1.y, it_mobs->coord2.x, it_mobs->coord2.y,
                //             newobs.coord1.x, newobs.coord1.y, newobs.coord2.x, newobs.coord2.y,
                //             temp.coord1.x, temp.coord1.y, temp.coord2.x, temp.coord2.y);
            }
            if (can_merge)
            {
                if (!same_obstacle(temp, *it_mobs))
                {
                    merged = temp;
                    it_mobs = mem_obstacles.erase(it_mobs);
                    nmerged++;
                    if (debug) printf("merged\n");
                }
                else
                {
                    should_add = false;
                    // printf("Same obstacles t(%f, %f) - (%f, %f) and m(%f, %f) - (%f, %f). Don't merge.\n", temp.coord1.x, temp.coord1.y,
                    //     temp.coord2.x, temp.coord2.y,
                    //     it_mobs->coord1.x, it_mobs->coord1.y, it_mobs->coord2.x, it_mobs->coord2.y);
                    break;
                }
            }
            else
            {
                it_mobs++;
            }
        }
        if (should_add)
        {
            // if (debug) printf("added\n");
            mem_obstacles.emplace_back(merged);
        }
        if (debug) timer.reset();
    }
}

RP::obstacle RP::merge(const obstacle &o, const obstacle &p, bool &can_merge)
{
    bool colinear = orientation(o.coord1, o.coord2, p.coord1) == 0 &&
                    orientation(o.coord1, o.coord2, p.coord2) == 0;
    if (!colinear)
    {
        can_merge = false;
        // printf("Not colinear\n");
        return o;
    }
    can_merge = true;
    point points[] = {o.coord1, o.coord2, p.coord1, p.coord2};
    std::sort(points, points + 4, [](const point &p1, const point &p2) { 
        if (p1.x != p2.x)
            return fabs(p1.x) > fabs(p2.x);
        else
            return fabs(p1.y) > fabs(p2.y);
    });
    if (fabs(points[3].x - points[0].x) - 1e-5 > fabs(o.coord2.x - o.coord1.x) + fabs(p.coord2.x - p.coord1.x)
        || fabs(points[3].y - points[0].y) - 1e-5 > fabs(o.coord2.y - o.coord1.y) + fabs(p.coord2.y - p.coord1.y))
    {
        can_merge = false;
        // printf("Colinear but not overlapping\n");
        return o;
    }
    // printf("Merging\n");
    // for (auto p : points)
    // {
    //     printf("(%f, %f), ", p.x, p.y);
    // }
    // printf("\n");
    return obstacle{false, points[0], points[3]};
}

//TODO(sasha): Find heuristics and upgrade to A*
std::vector<RP::point> RP::Map::shortest_path_to()
{
    std::vector<node> nodes = build_graph(cur, tar);
#if 0
    for(int i = 0; i < nodes.size(); i++)
    {
	std::cout << "Node " << i << " at (" << nodes[i].coord.first << ", " << nodes[i].coord.second << ") connected to: " << std::endl << '\t';
	for(auto con : nodes[i].connection)
	    std::cout << con.first << ' ';
	std::cout << std::endl;
    }
#endif
    auto cmp = [nodes](int l, int r) { return nodes[l].dist_to < nodes[r].dist_to; };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q(cmp);
    q.push(0);
    while (!q.empty())
    {
        int n = q.top();
        q.pop();

        for (auto &edge : nodes[n].connection)
        {
            float dist = nodes[n].dist_to + edge.second;
            if (dist < nodes[edge.first].dist_to)
            {
                nodes[edge.first].prev = n;
                nodes[edge.first].dist_to = dist;
                q.push(edge.first);
            }
        }
    }

    std::vector<point> result;
    int i = 1;
    while (i != 0)
    {
        if (i == -1) {
            printf("WARNING: no path found!\n");
            break;
        }
        node &n = nodes[i];
        // printf("node: %f, %f\n", n.coord.x, n.coord.y);
        result.push_back(n.coord);
        i = n.prev;
    }
    std::reverse(result.begin(), result.end());
    return (result);
}

void move_line_toward_point(RP::line& side_points, RP::point pt, float d)
{
    int orient = RP::orientation(side_points.p, side_points.q, pt);
    if (orient == 0) return;
    RP::point along{side_points.q.x - side_points.p.x, side_points.q.y - side_points.p.y};
    RP::point dir;
    if (orient == 1)
    {
        dir.x = along.y;
        dir.y = -along.x;
    }
    else
    {
        dir.x = -along.y;
        dir.y = along.x;
    }
    float norm = sqrt(dir.x * dir.x + dir.y * dir.y);
    dir.x *= d / norm;
    dir.y *= d / norm;
    side_points.p.x += dir.x; side_points.p.y += dir.y;
    side_points.q.x += dir.x; side_points.q.y += dir.y;
}
