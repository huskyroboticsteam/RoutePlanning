#ifndef ROVERPATHFINDING_UTILS_H
#define ROVERPATHFINDING_UTILS_H

#include <utility>
#include <vector>

namespace RoverPathfinding
{
struct point
{
    float x;
    float y;
};
float normalize_angle(float rad); // normalize angle (in radians) to between 0 and 2PI
float deg_to_rad(float deg);
float rad_to_deg(float rad);
point intersection(point A, point B, point C, point D);                        //Returns the point of intersection between lines AB and CD
bool segment_intersects_circle(point start, point end, point circle, float R); //Unused. Tells whether a line segment intersects with a circle of radius R with center at point "circle"
int orientation(point p, point q, point r);                                    //Takes three points. //Returns 0 if p, q, and r are colinear, 1 if pq, qr, and rp are clockwise, 2 if pq, qr, and rp are counterclockwise
bool on_segment(point p, point q, point r);                                    //Tells if three points are on a line segment
bool segments_intersect(point p1, point p2, point q1, point q2);               //Returns true if segment p1q1 intersects p2q2
point center_point_with_radius(point cur, point p, point q, float R);          //Returns a point that is in the middle of pq and is R in the direction of cur
float dist_sq(point p1, point p2);                                             //Returns the square of the distance between two points
bool within_radius(point p1, point p2, float R);                               //Returns whether p1 and p2 are within R of each other
point lat_long_offset(float lat1, float lon1, float brng, float dist); //Offsets a point with coordinates lat1, lon1, dist meters with bearing brng (0.0 is N)
point lat_long_to_meters(point pt, point origin); //Given pt in lat-long units, normalize it to be in cartesian coordinates (meters/km undecided) with origin provided
std::vector<RoverPathfinding::point> generate_spiral();
} // namespace RoverPathfinding

#endif