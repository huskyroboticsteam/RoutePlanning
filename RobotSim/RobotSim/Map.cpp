#include <queue>
#include <cmath>
#include <algorithm>
#include <list>
#include <cassert>
#include <numeric>
#include "Map.hpp"

RP::Map::Map(const point &cpos, const point &tget, float bw) : cur(cpos), tar(tget), bot_width(bw)
{
}

void RP::Map::add_obstacle(point coord1, point coord2)
{
    // obstacle o{};
    obstacles.push_back(obstacle{coord1, coord2});
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

RP::eptr RP::Map::add_edge(int parent, int child)
{
    float dist = std::sqrt(dist_sq(nodes[parent].coord, nodes[child].coord));

    eptr p_to_c = eptr(new edge{parent, child, dist});
    nodes[parent].connection.push_back(p_to_c);

    eptr c_to_p = eptr(new edge{child, parent, dist});
    nodes[child].connection.push_back(c_to_p);

    return p_to_c;
}

void RP::Map::remove_edge(int parent, int child)
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

int RP::Map::create_node(point coord)
{
    node n;
    n.prev = -1;
    n.dist_to = INFINITY;
    n.coord = coord;
    nodes.push_back(n);
    return (nodes.size() - 1);
}

std::vector<RP::node> RP::Map::build_graph(point cur, point tar, float side_tolerance)
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
    obstacles.clear();
    obstacles.reserve(mem_obstacles.size());
    for (const auto &o : mem_obstacles)
        obstacles.emplace_back(obstacle{o.coord1, o.coord2});

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
    eptr init_edge = add_edge(0, 1);
    if (obstacles.empty())
    {
        return (nodes);
    }

    // if obstacle has been visited from one side
    std::vector<bool> visited(obstacles.size(), false);
    std::queue<eptr> unprocessed_edges;
    // add first edge
    unprocessed_edges.push(init_edge);

    // pad for curr and parent
    // for each safety node, find the obstacle closest to it
    while (!unprocessed_edges.empty())
    {
        // TODO handle visited, intersections in edge chains (i.e. remove node and maybe edge)
        const eptr curr_edge = unprocessed_edges.front();
        unprocessed_edges.pop();
        int closest_index = get_closest_obstacle(curr_edge, bot_width);

        if (closest_index != -1)
        {
            obstacle &closest = obstacles.at(closest_index);
            remove_edge(curr_edge->parent, curr_edge->child);
            // printf("removing %f, %f - %f, %f\n", nodes.at(curr_edge->parent).coord.x, nodes.at(curr_edge->parent).coord.y,
            // nodes.at(curr_edge->child).coord.x, nodes.at(curr_edge->child).coord.y);
            if (!visited[closest_index])
            {
                // make copies of endponts
                point end1 = closest.coord1;
                point end2 = closest.coord2;
                line closer = add_length_to_line_segment(end1, end2, side_tolerance);
                line farther = add_length_to_line_segment(end1, end2, side_tolerance);

                point par = nd_coord(curr_edge->parent);
                point chd = nd_coord(curr_edge->child);
                int opar = orientation(closer.p, closer.q, par);
                int ochd = orientation(farther.p, farther.q, chd);

                if (opar == 0 || ochd == 0)
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
                int branch1closer = create_node(closer.p);
                int branch1farther = create_node(farther.p);
                unprocessed_edges.push(add_edge(curr_edge->parent, branch1closer));
                unprocessed_edges.push(add_edge(branch1closer, branch1farther));
                unprocessed_edges.push(add_edge(branch1farther, curr_edge->child));

                int branch2closer = create_node(closer.q);
                int branch2farther = create_node(farther.q);
                unprocessed_edges.push(add_edge(curr_edge->parent, branch2closer));
                unprocessed_edges.push(add_edge(branch2closer, branch2farther));
                unprocessed_edges.push(add_edge(branch2farther, curr_edge->child));

                // special case for "vertical" obstacles
                unprocessed_edges.push(add_edge(branch1closer, branch2closer));
                unprocessed_edges.push(add_edge(branch1farther, branch2farther));

                // set endpoints associated with obstacle as visited
                visited.at(closest_index) = true;
            }
            else
            {
                /*
                printf("visited (%f, %f) - (%f, %f)\n", obstacles[closest_index].coord1.x, 
                    obstacles[closest_index].coord1.y, obstacles[closest_index].coord2.x,
                obstacles[closest_index].coord2.y);
                */
            }
        }
    }
    return nodes;
}

int RP::Map::get_closest_obstacle(eptr edge, float path_width)
{
    float min_dist = INFINITY;
    int closest_index = -1;
    for (int i = 0; i < obstacles.size(); i++)
    {
        const auto &obst = obstacles.at(i);
        point inters;
        if (seg_intersects_width(nd_coord(edge->parent),
                                 nd_coord(edge->child), obst.coord1, obst.coord2, path_width, inters))
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
    if (debug)
    {
        // printf("Mem Count: %d\n", mem_obstacles.size());
        // for (const obstacle& o : mem_obstacles) printf("mem (%f, %f), (%f, %f)\n", o.coord1.x, o.coord1.y, o.coord2.x, o.coord2.y);
        // printf("New Count: %d\n", new_obstacles.size());
        // for (const obstacle& o : new_obstacles) printf("new (%f, %f), (%f, %f)\n", o.coord1.x, o.coord1.y, o.coord2.x, o.coord2.y);
    }
    /*
    iterate over each new obstacle, try to merge it with existing obstacles
    if they can merge (meaning, if they are colinear and overlapping), remove the old 
    obstacles and add the new merged obstacle
    */
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
            if (can_merge)
            {
                // printf("attempted merge: m(%f, %f) (%f, %f) and n(%f, %f) (%f, %f) -> t(%f, %f), (%f, %f)\n",
                //                 it_mobs->coord1.x, it_mobs->coord1.y, it_mobs->coord2.x, it_mobs->coord2.y,
                //                 newobs.coord1.x, newobs.coord1.y, newobs.coord2.x, newobs.coord2.y,
                //                 temp.coord1.x, temp.coord1.y, temp.coord2.x, temp.coord2.y);
                if (!same_obstacle(temp, *it_mobs))
                {
                    merged = temp;
                    it_mobs = mem_obstacles.erase(it_mobs);
                    nmerged++;
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
            // if (debug)
            // printf("added\n");
            mem_obstacles.emplace_back(merged);
        }
        if (debug)
            timer.reset();
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
        if (fabs(p1.x - p2.x) > 1e-3)
            return fabs(p1.x) > fabs(p2.x);
        else
            return fabs(p1.y) > fabs(p2.y);
    });
    if (fabs(points[3].x - points[0].x) - 1e-2 > fabs(o.coord2.x - o.coord1.x) + fabs(p.coord2.x - p.coord1.x) || fabs(points[3].y - points[0].y) - 1e-2 > fabs(o.coord2.y - o.coord1.y) + fabs(p.coord2.y - p.coord1.y))
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
    return obstacle{points[0], points[3]};
}

void assertGraph(std::vector<RP::node> nodes)
{
    // debug
    for (int parent = 0; parent < nodes.size(); parent++)
    {
        for (RP::eptr conn : nodes[parent].connection)
        {
            bool found = false;
            for (RP::eptr connback : nodes[conn->child].connection)
            {
                if (connback->child == parent)
                {
                    found = true;
                    break;
                }
            }
            assert(found);
        }
    }
}

void RP::Map::prune_path(std::vector<int> &path, float tol)
{
    int i = 1;
    path.insert(path.begin(), 0);

    while (i < path.size() - 1)
    {
        // construct temporary edge
        auto e = eptr(new edge{path[i - 1], path[i + 1]});
        if (get_closest_obstacle(e, tol) == -1)
        {
            path.erase(path.begin() + i);
        }
        else
        {
            i++;
        }
    }
    path.erase(path.begin());
}

// RP::point RP::Map::ind_to_coord(int i) { return nodes[i].coord; };

//TODO(sasha): Find heuristics and upgrade to A*
std::vector<RP::point> RP::Map::shortest_path_to()
{
    float tolerances[]{2.f, 1.5f, 1.f, 0.5f, 0.f};
    // size_t tol_len = arrlen(tolerances);
    for (int tol_ind = 0; tol_ind < 5; tol_ind++)
    {
        float tol = tolerances[tol_ind];
        // printf("%f\n", tol);
        std::vector<node> nodes = build_graph(cur, tar, tol);

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
            
            for (const eptr &e : nodes[n].connection)
            {
                float dist = nodes[n].dist_to + e->len;
                if (dist < nodes[e->child].dist_to)
                {
                    nodes[e->child].prev = n;
                    nodes[e->child].dist_to = dist;
                    q.push(e->child);
                }
            }
        }

        std::vector<int> pathIndices;
        int i = 1;
        bool pathFound = true;
        while (i != 0)
        {
            if (i == -1)
            {
                // printf("No path to target found. Resorting to node closest to target\n");
                pathFound = false;
                break;
            }
            node &n = nodes[i];
            // printf("node: %f, %f\n", n.coord.x, n.coord.y);
            pathIndices.push_back(i);
            i = n.prev;
        }
        // path not found. resort to node closest to target
        if (!pathFound)
        {
            const node &tar = nodes[1];
            std::vector<size_t> indices(nodes.size());
            iota(indices.begin(), indices.end(), 0);
            // sort by closest to tar
            std::sort(indices.begin() + 2, indices.end(),
                      [nodes, tar](size_t i, size_t j) {
                          return dist_sq(nodes[i].coord, tar.coord) < dist_sq(nodes[j].coord, tar.coord);
                      });

            assertGraph(nodes);
            int i = -1;
            for (auto it = indices.begin() + 2; it != indices.end(); it++)
            {
                // if (*it == 3)
                //     printf("biatch");
                pathIndices.clear();
                i = *it;
                while (i != 0)
                {
                    if (i < 0)
                        break;
                    node &n = nodes[i];
                    // printf("node: %f, %f\n", n.coord.x, n.coord.y);
                    pathIndices.push_back(i);
                    i = n.prev;
                }
                if (i >= 0)
                    break;
            }

            if (i < 0)
            {
                // printf("No path found for tolerance %f. Decreasing tolerance.\n", tol);
                continue;
            }
        }
        std::reverse(pathIndices.begin(), pathIndices.end());
        prune_path(pathIndices, tol + 1.f);
        std::vector<point> result;
        
        for (int ind : pathIndices)
            result.push_back(nodes[ind].coord);
        
        return (result);
    }
    printf("WARNING: completely trapped.\n");
    return std::vector<point>();
}
