#include <vector>
#include <iostream>
#include <fstream>
#include <cctype>
#include <cmath>
#include <assert.h>
#include "Simulator.h"
#include "../../src/Map.h"

#define ASSERT_ON 1

RoverPathfinding::Simulator::Simulator(const std::string &map_path, float init_bearing,
                                       simulator_config conf) : bearing(init_bearing), config(conf)
{
    load_map(map_path);
    cur_pos = point{0, 0};
}

RoverPathfinding::Simulator::Simulator() : Simulator("./obstacles.txt", 0.f, simulator_config{45.f, 10.f}) {}

void RoverPathfinding::Simulator::load_map(const std::string &path)
{
    sim_obstacles.clear();
    std::ifstream infile(path);
    // if (!infile.good())
    // {
    //     std::cerr << "Invalid map file: " << path << std::endl;
    //     std::cout << " eof()=" << infile.eof();
    //     std::cout << " fail()=" << infile.fail();
    //     std::cout << " bad()=" << infile.bad();
    // }
    float px, py, qx, qy;
    target_pos = point{px, py};
    while (infile >> px >> py >> qx >> qy)
    {
        sim_obstacles.push_back(RoverPathfinding::sim_obstacle{point{px, py}, point{qx, qy}});
    }
    infile.close();
}

void RoverPathfinding::Simulator::update_agent()
{
    upper_vis = normalize_angle(deg_to_rad(bearing + config.vision_angle / 2));
    lower_vis = normalize_angle(deg_to_rad(bearing - config.vision_angle / 2));
    // compute farthest point at lower and upper fov bounds once
    point fov_lower = polar_to_cartesian(cur_pos, config.vision_dist, lower_vis);
    point fov_upper = polar_to_cartesian(cur_pos, config.vision_dist, upper_vis);

    std::vector<point> intersects;
    std::list<sim_obstacle> cropped_obst;

    // crop obstacles
    for (auto it = sim_obstacles.begin(); it != sim_obstacles.end(); it++)
    {
        //possible optimization here for large map:
        //remove obstacles if they seem too far (i.e. much farther than vis_dist)
        bool p_within_view = !out_of_view(it->p);
        bool q_within_view = !out_of_view(it->q);
        if (p_within_view && q_within_view)
        {
            cropped_obst.push_back(*it);
        }
        else if (p_within_view || q_within_view)
        {
            point fixed_pt = p_within_view ? it->p : it->q;
            std::vector<point> pts = intersection_with_arc(it->p, it->q, fov_lower, fov_upper);
#if ASSERT_ON
            assert(pts.size() == 1);
#endif
            cropped_obst.push_back(sim_obstacle{fixed_pt, pts.at(0)});
        }
        else
        {
            // neither is within view. check for double intersection with arc
            std::vector<point> pts = intersection_with_arc(it->p, it->q, fov_lower, fov_upper);
            if (pts.size() == 2)
            {
                cropped_obst.push_back(sim_obstacle{pts.at(0), pts.at(1)});
            }
#if ASSERT_ON
            assert(pts.size() == 0);
#endif
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
                if (!same_point(s, p, 0.05) && dist < closest_dist)
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
                if (closest_dist != INFINITY)
                {
                    closest_obstacle.endpoints.push_back(closest);
                }
            }
        }
    }
    //note: side rays already accounted for in "crop obstacles"
    //collect and add obstacles
    view_obstacles.clear();
    for (sim_obstacle obs : cropped_obst)
    {
#if ASSERT_ON
        assert(obs.endpoints.size() % 2 == 0);
#endif
        for (auto it = obs.endpoints.begin(); it != obs.endpoints.end(); it++)
        {
            view_obstacles.push_back(line{*it, *(++it)});
        }
    }
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

bool RoverPathfinding::Simulator::out_of_view(const point &pt)
{
    // too far?
    if (dist_sq(cur_pos, pt) > config.vision_dist)
    {
        return false;
    }
    float angle = std::atan((pt.y - cur_pos.y) / (pt.x - cur_pos.x));
    // angle out of view
    if (!within_angle(angle, lower_vis, upper_vis))
    {
        return false;
    }

    for (auto it = sim_obstacles.begin(); it != sim_obstacles.end(); it++)
    {
        if (intersection(it->p, it->q, cur_pos, pt).x != INFINITY && (it->p != pt && it->q != pt)) // intersection is not self
            return false;
    }
    return true;
}

//getters
float RoverPathfinding::Simulator::get_bearing() { return bearing; }
RoverPathfinding::point RoverPathfinding::Simulator::get_pos() { return cur_pos; }
//setters
void RoverPathfinding::Simulator::set_bearing(float brng) { bearing = brng; }
void RoverPathfinding::Simulator::set_pos(float x, float y) {
    cur_pos.x = x;
    cur_pos.y = y;
}
