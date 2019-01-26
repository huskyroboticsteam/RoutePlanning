#include <queue>
#include <cmath>
#include <algorithm>    
#include "Map.hpp"
#include "utils.hpp"

void RP::Map::add_obstacle(point coord1, point coord2)
{
    // obstacle o{};
    std::pair<int, int> pair = std::make_pair(-1, -1);
    obstacles.push_back(obstacle{false, coord1, coord2, pair});
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

std::vector<RP::node> RP::Map::build_graph(point cur, point tar)
{
    //TODO(sasha): make R a constant - the following few lines are just a hack
    //             to get R to be in lat/lng units
    //<hack>
#define R_METERS 0.5f
    auto offset = RP::lat_long_offset(cur.x, cur.y, 0.0f, R_METERS);
    auto diff = std::make_pair(offset.x - cur.x, offset.y - cur.y);
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
            if (RP::segments_intersect(nodes[curr_node].coord, obst.coord1, tar, obst.coord2))
            {
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
            if (!obst.marked)
            {
                obst.marked = true;
                // TODO we might need to handle edge cases where we cannot circumvent the obstacle because we might be   of bounds
                auto obstacle_side_pts = add_length_to_line_segment(obst.coord1, obst.coord2, TOLERANCE);  // Add tolerance 

                bool create_n1 = true;
                bool create_n2 = true;

                //TODO(sasha): if this is too slow, use a partitioning scheme to only check against
                //             nodes in the vicinity
                for (int safety = 2; safety < nodes.size(); safety++) // possibly to check whether nodes are safe or not
                {
                    node &n = nodes[safety];
                    if (RP::within_radius(n.coord, obstacle_side_pts.p, TOLERANCE))
                    {
                        create_n1 = false;
                        n1 = safety;
                    }
                    if (RP::within_radius(n.coord, obstacle_side_pts.q, TOLERANCE))
                    {
                        create_n2 = false;
                        n2 = safety;
                    }
                }

                if (create_n1)
                    n1 = create_node(obstacle_side_pts.p);

                if (create_n2)
                    n2 = create_node(obstacle_side_pts.q);

                obst.side_safety_nodes.first = n1;
                obst.side_safety_nodes.second = n2;

                point center_coord = RP::center_point_with_radius(nodes[curr_node].coord, obstacle_side_pts.p, obstacle_side_pts.q, TOLERANCE);
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
std::vector<RP::point> RP::Map::shortest_path_to(float cur_lat, float cur_lng,
                                                                           float tar_lat, float tar_lng)
{
    auto cur = point{cur_lat, cur_lng};
    auto tar = point{tar_lat, tar_lng};
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

std::vector<RP::point> RP::Map::a_star_algorithm(float cur_lat, float cur_lng, 
																			float tar_lat, float tar_lng)
{
	auto cur = point{ cur_lat, cur_lng };
	auto tar = point{ tar_lat, tar_lng };
	std::vector<node> nodes = build_graph(cur, tar);

	// Assigns a heuristic value to all nodes and puts them in a new vector
	using heuristicPair = std::pair<node, std::pair<float, float>>;
	heuristicPair start = { nodes[0], std::make_pair(0, std::sqrt(dist_sq(nodes[0].coord, tar))) };
	std::vector<heuristicPair*> heuristicNodes;
	for (node node : nodes) {
		heuristicNodes.push_back(&std::make_pair(node, std::make_pair(INFINITY, std::sqrt(dist_sq(node.coord, tar)))));
	}
	heuristicNodes[0] = &start;

	std::vector<heuristicPair*> closedNodes;
	// Create and fill a priorityqueue of indexes, ordered on heuristic value
	auto cmp = [&](heuristicPair* l, heuristicPair* r) { 
		return l->second.first + l->second.second < r->second.first + r->second.second; 
	};
	std::priority_queue<heuristicPair*, std::vector<heuristicPair*>, decltype(cmp)> openNodes(cmp);
	openNodes.push(&start);

	// While we're not at tar, keep evaluating paths
	while (!openNodes.empty() && (openNodes.top()->first.coord != tar)) { 
		heuristicPair current = *openNodes.top();
		openNodes.pop();
		closedNodes.push_back(&current);

		// For each neighbor of current
		for (auto &index : current.first.connection) {
			heuristicPair neighbor = *heuristicNodes[index.first];
			float distFromStart = index.second + current.second.first;
			// If this is a more efficient path
			if (neighbor.second.first > distFromStart) {
				// If neighbor is in closedNodes, remove it
				int closedIndex = std::find(closedNodes.begin(), closedNodes.end(), &neighbor) - closedNodes.begin;
				if (closedIndex < closedNodes.size) {
					closedNodes.erase(closedNodes.begin + closedIndex);
				}
				// Add neighbor to openNodes and set its previous to the current node
				neighbor.second.first = distFromStart;
				openNodes.push(&neighbor);
				neighbor.first.prev = std::find(heuristicNodes.begin(), heuristicNodes.end(), &current) - heuristicNodes.begin;
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