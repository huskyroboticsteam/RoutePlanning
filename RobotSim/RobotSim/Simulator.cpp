#include <vector>
#include <iostream>
#include <fstream>
#include <cctype>
#include <cmath>
#include <assert.h>
#include "Simulator.hpp"
#include "grid.hpp"
#include "Map.hpp"

#define DEBUG_MSG 1

void debugmsg(const char*);

RoverPathfinding::Simulator::Simulator(const std::list<Obstacle> &obstacleList, const Agent &agt,
                                       simulator_config conf, float map_scale) : sim_obstacles(obstacleList),
                                                                                 agent(agt), config(conf), scale(map_scale),
                                                                                 vision_dist_sq(std::pow(conf.vision_dist, 2))
{
    std::cout << vision_dist_sq;
}

void RoverPathfinding::Simulator::update_agent()
{
    cur_pos = point{agent.getX(), agent.getY()};
    float bearing = agent.getInternalRotation();

    point cur_pos{agent.getX(), agent.getY()};
    upper_vis = normalize_angle(deg_to_rad(bearing + config.vision_angle / 2));
    lower_vis = normalize_angle(deg_to_rad(bearing - config.vision_angle / 2));
    // compute farthest point at lower and upper fov bounds once
    fov_lower = polar_to_cartesian(cur_pos, config.vision_dist, lower_vis);
    fov_upper = polar_to_cartesian(cur_pos, config.vision_dist, upper_vis);
    if (sim_obstacles.empty())
    {
        return;
    }

    std::vector<point> intersects;
    std::list<sim_obstacle> cropped_obst;

    // crop obstacles
    for (auto it = sim_obstacles.begin(); it != sim_obstacles.end(); it++)
    {
        point p{it->x1, it->y1};
        point q{it->x2, it->y2};
        //possible optimization here for large map:
        //remove obstacles if they seem too far (i.e. much farther than vis_dist)
        bool p_within_view = within_view(p);
        bool q_within_view = within_view(q);
        if (p_within_view && q_within_view)
        {
            cropped_obst.push_back(sim_obstacle{p, q});
            // debugmsg("BOTH endpoints within view");
        }
        else if (p_within_view || q_within_view)
        {
            point fixed_pt = p_within_view ? p : q;
            std::vector<point> pts = intersection_with_arc(p, q, fov_lower, fov_upper);
            assert(pts.size() == 1);
            cropped_obst.push_back(sim_obstacle{fixed_pt, pts.at(0)});
            // debugmsg("ONE endpoints within view");
        }
        else
        {
            // neither is within view. check for double intersection with arc
            std::vector<point> pts = intersection_with_arc(p, q, fov_lower, fov_upper);
            if (pts.size() == 2)
            {
                cropped_obst.push_back(sim_obstacle{pts.at(0), pts.at(1)});
            }
            assert(pts.size() == 0);
            // debugmsg("NO endpoint within view");
        }
    }

    /*
    Logic:
    for each obstacle, and for each of its endpoints, find the closest intersection between line{cur_pos, endpoint} and 
    any other obstacle. If that intersection is closer to cur_pos than endpoint is, remove this endpoint from obstacle.
    Else, add that intersection point to the obstacle it landed on.
    */
    for (auto it = cropped_obst.begin(); it != cropped_obst.end(); it++)
    {
        // iterate over two endpoints
        point pts[2] = {it->p, it->q};
        for (point p : pts)
        {
            // find intersection
            float closest_dist = INFINITY;
            point closest;
            sim_obstacle closest_obstacle;
            for (auto so : cropped_obst)
            {
                point s = intersection(cur_pos, p, so.p, so.q);
                float dist = dist_sq(cur_pos, s);
                if (!same_point(s, p, 0.05) && (dist < closest_dist))
                {
                    closest_dist = dist;
                    closest = s;
                    closest_obstacle = so;
                }
            }

            // not blocked
            if (closest_dist >= dist_sq(cur_pos, p))
            {
                it->endpoints.push_back(p);
            }
            // blocked. Add projection point onto closest obstacle
            else if (closest_dist != INFINITY)
            {
                closest_obstacle.endpoints.push_back(closest);
            }
        }
    }
    //note: side rays already accounted for in "crop obstacles"
    //collect and add obstacles
    view_obstacles.clear();
    for (sim_obstacle obs : cropped_obst)
    {
        assert(obs.endpoints.size() % 2 == 0);
        for (auto it = obs.endpoints.begin(); it != obs.endpoints.end(); it++)
        {
            view_obstacles.push_back(line{*it, *(++it)});
        }
    }
}

void debugmsg(const char *line)
{
#if DEBUG_MSG
    std::cout << line << std::endl;
#endif
}

// Find the intersection of a line with the field of view arc (https://stackoverflow.com/a/30012445)
std::vector<RoverPathfinding::point>
RoverPathfinding::Simulator::intersection_with_arc(const point &p1, const point &p2,
                                                   const point &lower_point, const point &upper_point)
{
    std::vector<point> ret;
    // check the two extreme rays
    point s1 = intersection(p1, p2, cur_pos, lower_point);
    point s2 = intersection(p1, p2, cur_pos, upper_point);
    if (s1.x != INFINITY)
        ret.push_back(s1);
    if (s2.x != INFINITY)
        ret.push_back(s2);

    if (ret.size() == 2)
        return ret;

    float a = p1.y - p1.y;
    float b = p2.x - p1.x;
    float c = p2.x * p1.y - p1.x * p2.y;
    float p = std::atan(b / a);
    float q = std::acos(-(a * cur_pos.x + b * cur_pos.y + c) / (config.vision_dist * std::sqrt(a * a + b * b)));

    float t1 = normalize_angle(p + q); // first possible angle
    float t2 = normalize_angle(p - q); // second possible angle
    point pt1 = polar_to_cartesian(cur_pos, config.vision_dist, t1);
    point pt2 = polar_to_cartesian(cur_pos, config.vision_dist, t2);
    float within_1 = within_angle(t1, lower_vis, upper_vis);
    float within_2 = within_angle(t2, lower_vis, upper_vis);

    if (within_1)
        ret.push_back(pt1);
    if (within_2)
        ret.push_back(pt2);
    return ret;
}

bool RoverPathfinding::Simulator::within_view(const point &pt)
{
    // too far?
    if (dist_sq(cur_pos, pt) > vision_dist_sq)
    {
        debugmsg("too far");
        return false;
    }
    float angle = std::atan2(-(pt.y - cur_pos.y), -(pt.x - cur_pos.x));
    // angle out of view
    if (!within_angle(angle, lower_vis, upper_vis))
    {
        debugmsg("outside angle");
        return false;
    }

    for (auto it = sim_obstacles.begin(); it != sim_obstacles.end(); it++)
    {
        point p{it->x1, it->y1};
        point q{it->x2, it->y2};
        if (intersection(p, q, cur_pos, pt).x != INFINITY && (!same_point(p, pt, 0.05) && !same_point(q, pt, 0.05))) { // intersection is not self
            debugmsg("blocked");
            return false;
        }
    }
    debugmsg("in view");
    return true;
}

sf::VertexArray get_vertex_line(RoverPathfinding::point p, RoverPathfinding::point q, sf::Color c, float scale = 1.f)
{
    sf::VertexArray line;
    line.setPrimitiveType(sf::Lines);
    line.resize(2);
    line[0] = sf::Vertex(sf::Vector2f((p.x + 1) * scale, (p.y + 1) * scale));
    line[0].color = c;
    line[1] = sf::Vertex(sf::Vector2f((q.x + 1) * scale, (q.y + 1) * scale));
    line[1].color = c;
    return line;
}

void RoverPathfinding::Simulator::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(get_vertex_line(cur_pos, fov_lower, sf::Color::Blue, scale));
    target.draw(get_vertex_line(cur_pos, fov_upper, sf::Color::Blue, scale));
    for (auto obst : view_obstacles)
    {
        target.draw(get_vertex_line(obst.p, obst.q, sf::Color::Green, scale), states);
    }
}
