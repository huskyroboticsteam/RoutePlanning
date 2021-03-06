#ifndef ROVERPATHFINDING_UTILS_H
#define ROVERPATHFINDING_UTILS_H

#include <utility>
#include <vector>
#include <SFML/Graphics.hpp>
// #include "Map.hpp"

#define THEME 0 // 0 is normal, 1 is HACKERMODE

#define TARGET_COLOR sf::Color:Red;
#if THEME
#define PATH_COLOR sf::Color::Yellow 
#define OBST_COLOR sf::Color(82, 165, 46)
#define VISIBLE_OBST_COLOR sf::Color(178, 255, 145)
#define SEEN_OBST_COLOR sf::Color::Green
#define VIEW_SHAPE_COLOR sf::Color::White
#else
#define PATH_COLOR sf::Color(115, 92, 196);
#define OBST_COLOR sf::Color::Black
#define VISIBLE_OBST_COLOR sf::Color::Green
#define SEEN_OBST_COLOR sf::Color(58, 201, 0)
#define VIEW_SHAPE_COLOR sf::Color::Blue
#endif

//#define float double // YOU ARE GOING TO HELL
 
typedef unsigned int uint;
namespace RP
{
struct point
{
    float x;
    float y;
    bool operator==(const point& p) const;
    bool operator!=(const point& p) const;
};
struct line
{
  point p;
  point q;
};

// normalize angle (in radians) to between 0 and 2PI
float normalize_angle(float rad);
float normalize_angle_deg(float deg);
float deg_to_rad(float deg);
float rad_to_deg(float rad);

//Returns the point of intersection between lines AB and CD
point intersection(point A, point B, point C, point D);

//Unused. Tells whether a line segment intersects with a circle of radius R with center at point "circle"
bool segment_intersects_circle(point start, point end, point circle, float R);

//Takes three points. Returns 0 if p, q, and r are colinear, 1 if pq, qr, and rp are clockwise, 2 if pq, qr, and rp are counterclockwise
int orientation(point p, point q, point r);

//Tells if three points are on a line segment
bool on_segment(point p, point q, point r);

//Returns true if segment p1q1 intersects p2q2
bool segments_intersect(point p1, point p2, point q1, point q2);

// intersection() but constrained by segment length
point segments_intersection(point a, point b, point c, point d);

//check if point a is on segment pq
bool within_segment(point p, point q, point a);

//Returns a point that is in the middle of pq and is R in the direction of cur
point center_point_with_radius(point cur, point p, point q, float R);

//Returns the square of the distance between two points
float dist_sq(point p1, point p2);

bool same_dir(point a, point b, point c);

//Returns whether p1 and p2 are within R of each other
bool within_radius(point p1, point p2, float R);

//Offsets a point with coordinates lat1, lon1, dist meters with bearing brng (0.0 is N)
point lat_long_offset(float lat1, float lon1, float brng, float dist);

//Given pt in lat-long units, normalize it to be in cartesian coordinates (meters/km undecided) with origin provided
point lat_long_to_meters(point pt, point origin); 

std::vector<RP::point> generate_spiral();
//Returns true if ang is within range of [lower, upper] (going counterclocwise).
bool within_angle(float ang, float lower, float upper);      
//Polar to cartesian relative to the given origin                   
point polar_to_cartesian(point origin, float r, float theta); 
//Find angle of p relative to origin, where positive x-axis is 0 radians.                  
float relative_angle(point origin, point p);                                    
bool same_point(const point &p, const point &q, float = 1e-6);
bool closeEnough(float a, float b, float tol=1e-6);
bool angleCloseEnough(float deg1, float deg2, float degtol=0.5f);

//Find if o and p are parallel AND overlap. If they do, can merge is set to true and
// return a merged obstacle; else, can_merge is set to false
                       

} // namespace RP

sf::VertexArray get_vertex_line(RP::point p, RP::point q, sf::Color c, float scale, float window_height);

#endif
