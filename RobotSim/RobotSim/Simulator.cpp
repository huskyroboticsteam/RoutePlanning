#include <vector>
#include <iostream>
#include <fstream>
#include <cctype>
#include <cmath>
#include <assert.h>
#include "Simulator.hpp"
#include "grid.hpp"
#include "Map.hpp"

#define DEBUG_MSG 0

void debugmsg(const char *);

RoverPathfinding::Simulator::Simulator(const std::list<Obstacle> &obstacleList, const Agent &agt,
                                       simulator_config conf, float map_scale, float windowH) : raw_obstacles(obstacleList),
                                                                                                agent(agt), config(conf), scale(map_scale),
                                                                                                window_height(windowH),
                                                                                                vision_dist_sq(std::pow(conf.vision_dist, 2))
{
    // std::cout << vision_dist_sq;
}

void RoverPathfinding::Simulator::update_agent()
{
    all_obstacles.clear();
    int id = 0;
    for (auto const &obst : raw_obstacles)
    {
        sim_obstacle to_add{point{obst.x1, obst.y1}, point{obst.x2, obst.y2}};
        all_obstacles.push_back(to_add);
    }
    cur_pos = point{agent.getX(), agent.getY()};
    float bearing = agent.getInternalRotation();

    point cur_pos{agent.getX(), agent.getY()};
    upper_vis = normalize_angle(deg_to_rad(bearing + config.vision_angle / 2));
    lower_vis = normalize_angle(deg_to_rad(bearing - config.vision_angle / 2));
    // compute farthest point at lower and upper fov bounds once
    fov_lower = polar_to_cartesian(cur_pos, config.vision_dist, lower_vis);
    fov_upper = polar_to_cartesian(cur_pos, config.vision_dist, upper_vis);
    if (all_obstacles.empty())
    {
        return;
    }

    std::list<sim_obstacle> cropped_obst;
    // crop obstacles
    for (auto it = all_obstacles.begin(); it != all_obstacles.end(); it++)
    {
        point p = it->p;
        point q = it->q;
        //possible optimization here for large map:
        //remove obstacles if they seem too far (i.e. much farther than vis_dist)
        bool p_within_view = within_view(p);
        bool q_within_view = within_view(q);
        if (p_within_view && q_within_view)
        {
            debugmsg("BOTH endpoints within view");
            sim_obstacle so = {p, q};
            so.id = it->id;
            cropped_obst.push_back(so);
        }
        else if (p_within_view || q_within_view)
        {
            debugmsg("ONE endpoints within view");
            point fixed_pt = p_within_view ? p : q;
            std::vector<point> pts = intersection_with_arc(p, q, fov_lower, fov_upper);
            debugmsg("intersecting with arc");
            // assert(pts.size() == 1);
            if (pts.size() == 1)
            {
                sim_obstacle so = {fixed_pt, pts.at(0)};
                so.id = it->id;
                cropped_obst.push_back(so);
            }
        }
        else
        {
            debugmsg("NO endpoint within view");
            // neither is within view. check for double intersection with arc
            int sz = all_obstacles.size();
            std::vector<point> pts = intersection_with_arc(p, q, fov_lower, fov_upper);
            if (pts.size() == 2)
            {
                debugmsg("intersecting with arc");
                sim_obstacle so {pts.at(0), pts.at(1)};
                so.id = it->id;
                cropped_obst.push_back(so);
            }
            else
            {
                if (pts.size() != 0)
                {
                    intersection_with_arc(p, q, fov_lower, fov_upper);
                }
            }
        }
    }
    // std::cout << cropped_obst.size() << std::endl;
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
            for (auto jt = all_obstacles.begin(); jt != all_obstacles.end(); jt++)
            {
                if (it->id == jt->id) // same one
                    continue;
                point s = intersection(cur_pos, p, jt->p, jt->q);
                if (s.x == INFINITY)
                    continue;
                if (same_point(s, p, 1e-5))
                {
                    // TODO this is a shared vertex. decide if this should be added
                    // simply see the line intesection of jt and point{cur_pos, q}
                    // where q is the other side point (from p). If the intersect
                    // is in the same direction as q relative to cur_pos, then
                    // this p should be discarded
                    point q = pts[p == pts[0]];
                    point inter = intersection(jt->p, jt->q, cur_pos, q);
                    if (inter.x == INFINITY || same_dir(cur_pos, inter, q))
                    {
                        goto sidepoint_end; // skip this sidepoint
                    }
                }
                float dist = dist_sq(cur_pos, s);
                if (same_dir(cur_pos, p, s) && dist < closest_dist)
                {
                    closest_dist = dist;
                    closest = s;
                    closest_obstacle = *jt;
                }
            }

            // not blocked
            if (closest_dist >= dist_sq(cur_pos, p))
            {
                it->endpoints.push_back(p);
                if (closest_dist != INFINITY && closest_dist <= config.vision_dist)
                    closest_obstacle.endpoints.push_back(closest); // add projection
            }
        sidepoint_end:
        {
        }
        }
    }
    //note: side rays already accounted for in "crop obstacles"
    //collect and add obstacles
    view_obstacles.clear();
    for (sim_obstacle obs : cropped_obst)
    {
        assert(obs.endpoints.size() % 2 == 0);
        obs.endpoints.sort([](const point &p, const point &q) { return p.x > q.x; });
        for (auto it = obs.endpoints.begin(); it != obs.endpoints.end(); it++)
        {
            point a = *it;
            if (++it == obs.endpoints.end())
                break;
            view_obstacles.push_back(line{a, *it});
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
    point s1 = segments_intersection(p1, p2, cur_pos, lower_point);
    point s2 = segments_intersection(p1, p2, cur_pos, upper_point);
    if (s1.x != INFINITY)
        ret.push_back(s1);
    if (s2.x != INFINITY)
        ret.push_back(s2);

    if (ret.size() == 2)
        return ret;

    float a = p2.y - p1.y;
    float b = -(p2.x - p1.x);
    float c = p2.x * p1.y - p1.x * p2.y;
    float p = std::atan2(b, a);
    float cosval = -(a * cur_pos.x + b * cur_pos.y + c) / (config.vision_dist * std::sqrt(a * a + b * b));
    if (cosval >= 1 || cosval < -1)
        return ret; // no intersection/tangent
    float q = std::acos(cosval);

    float t1 = normalize_angle(p + q); // first possible angle
    float t2 = normalize_angle(p - q); // second possible angle
    point pt1 = polar_to_cartesian(cur_pos, config.vision_dist, t1);
    point pt2 = polar_to_cartesian(cur_pos, config.vision_dist, t2);
    float within_1 = within_angle(t1, lower_vis, upper_vis);
    float within_2 = within_angle(t2, lower_vis, upper_vis);

    float seg_len = dist_sq(p1, p2);
    if (within_1 && within_segment(p1, p2, pt1))
        ret.push_back(pt1);
    if (within_2 && within_segment(p1, p2, pt2))
        ret.push_back(pt2);
    return ret;
}

bool RoverPathfinding::Simulator::within_view(const point &pt)
{
    // too far?
    if (dist_sq(cur_pos, pt) > vision_dist_sq)
    {
        return false;
    }
    float angle = normalize_angle(std::atan2(pt.y - cur_pos.y, pt.x - cur_pos.x));
    // angle out of view
    if (!within_angle(angle, lower_vis, upper_vis))
    {
        return false;
    }

    for (auto it = all_obstacles.begin(); it != all_obstacles.end(); it++)
    {
        const point p = it->p;
        const point &q = it->q;
        if (segments_intersection(p, q, cur_pos, pt).x != INFINITY && (!same_point(p, pt, 1e-4) && !same_point(q, pt, 1e-4)))
        { // intersection is not self
            return false;
        }
    }
    return true;
}

sf::VertexArray get_vertex_line(RoverPathfinding::point p, RoverPathfinding::point q, sf::Color c, float scale, float window_height)
{
    sf::VertexArray line;
    line.setPrimitiveType(sf::Lines);
    line.resize(2);
    line[0] = sf::Vertex(sf::Vector2f((p.x + 1) * scale, (window_height - p.y) * scale));
    line[0].color = c;
    line[1] = sf::Vertex(sf::Vector2f((q.x + 1) * scale, (window_height - q.y) * scale));
    line[1].color = c;
    return line;
}

void RoverPathfinding::Simulator::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(get_vertex_line(cur_pos, fov_lower, sf::Color::Blue, scale, window_height));
    target.draw(get_vertex_line(cur_pos, fov_upper, sf::Color::Blue, scale, window_height));
    for (auto obst : view_obstacles)
    {
        target.draw(get_vertex_line(obst.p, obst.q, sf::Color::Green, scale, window_height), states);
    }
}
