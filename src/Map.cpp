#include <queue>
#include <cmath>
#include <algorithm>    
#include "Map.h"



RoverPathfinding::point convertFloatsToPoint(float first, float second)
{
	RoverPathfinding::point p = { first, second };
	return p;
}

RoverPathfinding::point convertPairToPoint(std::pair<float, float> pair)
{
	RoverPathfinding::point p = { pair.first, pair.second };
	return p;
}

void RoverPathfinding::Map::add_obstacle(point coord1, point coord2)
{
	obstacle o;
	o.marked = false;
	o.coord1 = coord1;
	o.coord2 = coord2;
	obstacles.push_back(o);
}

void RoverPathfinding::Map::add_obstacle(std::pair<float, float> coord1, std::pair<float, float> coord2) {
	add_obstacle(convertPairToPoint(coord1), convertPairToPoint(coord2));
}

// Adds extra distance between p and q, returns a pair of points in the line
std::pair<RoverPathfinding::point, RoverPathfinding::point> RoverPathfinding::Map::add_length_to_line_segment(point p, point q, float length)
{
    // boom_bam();
	point pq = convertFloatsToPoint(q.first - p.first, q.second - p.second); //vector (delta x, delta y)
    float pq_len = sqrt(pq.first * pq.first + pq.second * pq.second); // Length of vector(pythag)
    pq.first = length * pq.first / pq_len;
    pq.second = length * pq.second / pq_len;

	point p1 = convertFloatsToPoint(q.first + pq.first, q.second + pq.second);
	point p2 = convertFloatsToPoint(p.first - pq.first, p.second - pq.second);
    return (std::make_pair(p1, p2));
}

void RoverPathfinding::Map::add_edge(int n1, int n2)
{
    float dist = std::sqrt(dist_sq(nodes[n1].coord, nodes[n2].coord));

    auto n1_to_n2 = std::make_pair(n2, dist);
    nodes[n1].connection.push_back(n1_to_n2);

    auto n2_to_n1 = std::make_pair(n1, dist);
    nodes[n2].connection.push_back(n2_to_n1);
}

int RoverPathfinding::Map::create_node(point coord)
{
    node n;
    n.prev = -1;
    n.dist_to = INFINITY;
    n.coord = coord;
    nodes.push_back(n);
    return (nodes.size() - 1);
}

std::vector<RoverPathfinding::node> RoverPathfinding::Map::build_graph(point cur, point tar)
{
    //TODO(sasha): make R a constant - the following few lines are just a hack
    //             to get R to be in lat/lng units
    //<hack>
#define R_METERS 0.5f
    auto offset = RoverPathfinding::lat_long_offset(cur.first, cur.second, 0.0f, R_METERS);
    auto diff = std::make_pair(offset.first - cur.first, offset.second - cur.second);
    float TOLERANCE = sqrt(diff.first * diff.first + diff.second * diff.second);
#undef R_METERS
    //</hack>

    node start;
    for (auto &n : nodes)
    {
        n.dist_to = INFINITY;
        n.connection.clear();
    }

    nodes[0].prev = -1;
    nodes[0].dist_to = 0.0f;
    nodes[0].coord = cur;

    nodes[1].coord = tar;

    if (obstacles.empty())
    {
        add_edge(0, 1); // probably start (origin or current?) and target nodes
        return (nodes);
    }

    std::queue<int> unprocessed_nodes;
    unprocessed_nodes.push(0);
    // for each safety node, find the obstacle closest to it
    while (!unprocessed_nodes.empty())
    {
        int curr_node = unprocessed_nodes.front();
        unprocessed_nodes.pop();
        bool destination_blocked = false;
        int closest_obst;
        float min_dist = INFINITY;
        for (int i = 0; i < obstacles.size(); i++)
        {
            auto &obst = obstacles[i];
            if (RoverPathfinding::segments_intersect(nodes[curr_node].coord, obst.coord1, tar, obst.coord2))
            {
                destination_blocked = true;
                point inters = intersection(nodes[curr_node].coord, tar, obst.coord1, obst.coord2);
                float dist = RoverPathfinding::dist_sq(nodes[curr_node].coord, inters);
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
            if (!obst.marked)
            {
                obst.marked = true;
                // TODO we might need to handle edge cases where we cannot circumvent the obstacle because we might be out of bounds
                auto obstacle_side_pts = add_length_to_line_segment(obst.coord1, obst.coord2, TOLERANCE);  // Add tolerance 

                bool create_n1 = true;
                bool create_n2 = true;

                //TODO(sasha): if this is too slow, use a partitioning scheme to only check against
                //             nodes in the vicinity
                for (int safety = 2; safety < nodes.size(); safety++) // possibly to check whether nodes are safe or not
                {
                    node &n = nodes[safety];
                    if (RoverPathfinding::within_radius(n.coord, obstacle_side_pts.first, TOLERANCE))
                    {
                        create_n1 = false;
                        n1 = safety;
                    }
                    if (RoverPathfinding::within_radius(n.coord, obstacle_side_pts.second, TOLERANCE))
                    {
                        create_n2 = false;
                        n2 = safety;
                    }
                }

                if (create_n1)
                    n1 = create_node(obstacle_side_pts.first);

                if (create_n2)
                    n2 = create_node(obstacle_side_pts.second);

                obst.side_safety_nodes = std::make_pair(n1, n2);

                point center_coord = RoverPathfinding::center_point_with_radius(nodes[curr_node].coord, obstacle_side_pts.first, obstacle_side_pts.second, TOLERANCE);
                obst.center_safety_node = create_node(center_coord);
                add_edge(curr_node, obst.center_safety_node);
                add_edge(n1, obst.center_safety_node);
                add_edge(n2, obst.center_safety_node);
            }
            else
            {
                n1 = obst.side_safety_nodes.first;
                n2 = obst.side_safety_nodes.second;
            }
            if (n1 >= 0)
                add_edge(curr_node, n1);
            if (n2 >= 0)
                add_edge(curr_node, n2);

            unprocessed_nodes.push(n1);
            unprocessed_nodes.push(n2);
        }
        else
        {
            add_edge(curr_node, 1);
        }
    }
    return (nodes);
}

//TODO(sasha): Find heuristics and upgrade to A*
std::vector<RoverPathfinding::point> RoverPathfinding::Map::shortest_path_to(float cur_lat, float cur_lng,
                                                                           float tar_lat, float tar_lng)
{
    auto cur = convertFloatsToPoint(cur_lat, cur_lng);
    auto tar = convertFloatsToPoint(tar_lat, tar_lng);
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
        node &n = nodes[i];
        result.push_back(n.coord);
        i = n.prev;
    }
    std::reverse(result.begin(), result.end());
    return (result);
}
