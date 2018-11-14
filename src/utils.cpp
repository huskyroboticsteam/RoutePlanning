#include <algorithm>
#include <cmath>
#include "utils.h"

#define PI 3.14159265359

float RoverPathfinding::deg_to_rad(float deg)
{
    return (deg * PI / 180.0f);
}

float RoverPathfinding::rad_to_deg(float rad)
{
    return (rad * 180.0f / PI);
}

float RoverPathfinding::normalize_angle(float rad)
{
    rad = std::fmod(rad, 2 * PI);
    if (rad < 0)
        rad += 2 * PI;
    return rad;
}

void boom_bam() {};

RoverPathfinding::point RoverPathfinding::intersection(point A, point B, point C, point D)
{
    // Line AB represented as a1x + b1y = c1
    float a1 = B.second - A.second;
    float b1 = A.first - B.first;
    float c1 = a1 * (A.first) + b1 * (A.second);

    // Line CD represented as a2x + b2y = c2
    float a2 = D.second - C.second;
    float b2 = C.first - D.first;
    float c2 = a2 * (C.first) + b2 * (C.second);

    float determinant = a1 * b2 - a2 * b1;

    // The lines are parallel. This is simplified
    // by returning a pair of FLT_MAX
    if (-1e-7 <= determinant && determinant <= 1e-7)
        return (std::make_pair(INFINITY, INFINITY));

    float x = (b2 * c1 - b1 * c2) / determinant;
    float y = (a1 * c2 - a2 * c1) / determinant;
    return (std::make_pair(x, y));
}

//start, end, circle, and R are in lat/long coordinates
bool RoverPathfinding::segment_intersects_circle(point start,
                                                      point end,
                                                      point circle,
                                                      float R)
{

    auto direction = std::make_pair(end.first - start.first,
                                    end.second - start.second);
    auto center_to_start = std::make_pair(start.first - circle.first,
                                          start.second - circle.second);
    float a = direction.first * direction.first + direction.second * direction.second;
    float b = 2 * (center_to_start.first * direction.first + center_to_start.second * direction.second);
    float c = (center_to_start.first * center_to_start.first + center_to_start.second * center_to_start.second) + R * R;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return (false);
    }

    discriminant = sqrt(discriminant);
    float t1 = (-b + discriminant) / (2 * a);
    float t2 = (-b - discriminant) / (2 * a);

    return ((0 <= t1 && t1 <= 1.0f) || (0 <= t2 && t2 <= 1.0f));
}

#define COLINEAR 0
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 2
//Returns 0 if p, q, and r are colinear.
//Returns 1 if pq, qr, and rp are clockwise
//Returns 2 if pq, qr, and rp are counterclockwise
int RoverPathfinding::orientation(point p, point q, point r)
{
    float v = (q.second - p.second) * (r.first - q.first) -
              (q.first - p.first) * (r.second - q.second);
    if (-1e-7 <= v && v <= 1e-7)
        return COLINEAR;

    return ((v > 0.0f) ? CLOCKWISE : COUNTERCLOCKWISE);
}

//Tells if r is on segment pq
bool RoverPathfinding::on_segment(point p, point q, point r)
{
    return (q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) &&
            q.second <= std::max(p.second, r.second) && q.second >= std::min(p.second, r.second));
}

//Probably returns whether p1q1 and p2q2 intersect
bool RoverPathfinding::segments_intersect(point p1, point p2, point q1, point q2)
{
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return (true);

    if (o1 == 0 && on_segment(p1, p2, q1))
        return (true);

    if (o2 == 0 && on_segment(p1, q2, q1))
        return (true);

    if (o3 == 0 && on_segment(p2, p1, q2))
        return (true);

    if (o4 == 0 && on_segment(p2, q1, q2))
        return (true);

    return (false);
}

//Returns a point in the center of segment pq and then moves it R towards cur
RoverPathfinding::point RoverPathfinding::center_point_with_radius(RoverPathfinding::point cur, RoverPathfinding::point p, RoverPathfinding::point q, float R)
{
    point vec = std::make_pair(-p.second + q.second, p.first - q.first);
    float len = sqrt((vec.first * vec.first) + (vec.second * vec.second));
    vec.first = vec.first * R / len;
    vec.second = vec.second * R / len;
    point result = std::make_pair((p.first + q.first) / 2.0f, (p.second + q.second) / 2.0f);
    int o = orientation(cur, p, q);
    if (o == CLOCKWISE)
    {
        result.first += vec.first;
        result.second += vec.second;
    }
    else //o is COUNTERCLOCKWISE
    {
        result.first -= vec.first;
        result.second -= vec.second;
    }
    return (result);
}

float RoverPathfinding::dist_sq(point p1, point p2)
{
    return ((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

bool RoverPathfinding::within_radius(point p1, point p2, float R)
{
    return (dist_sq(p1, p2) <= R * R);
}

#define R_EARTH 6371.0088 // in km

// given lat, long, bearing (in degrees), and distance (in km), returns a new point
RoverPathfinding::point RoverPathfinding::lat_long_offset(float lat1, float lon1, float brng, float dist)
{
    dist /= 1000.0f;

    lat1 = deg_to_rad(lat1);
    lon1 = deg_to_rad(lon1);
    brng = deg_to_rad(brng);
    float lat2 = asin(sin(lat1) * cos(dist / R_EARTH) + cos(lat1) * sin(dist / R_EARTH) * cos(brng));
    float lon2 = lon1 + atan2(sin(brng) * sin(dist / R_EARTH) * cos(lat1), cos(dist / R_EARTH) - sin(lat1) * sin(lat2));
    lat2 = rad_to_deg(lat2);
    lon2 = rad_to_deg(lon2);
    return (std::make_pair(lat2, lon2));
}

// TODO and modify header
RoverPathfinding::point RoverPathfinding::lat_long_to_meters(RoverPathfinding::point pt, RoverPathfinding::point origin) 
{

}

// generates 100 points in spiral formation around origin and returns in vector 
std::vector<RoverPathfinding::point> RoverPathfinding::generate_spiral()
{
	int scaleFactor = 10;
	std::vector<RoverPathfinding::point> spiralPoints;

	for (int i = 0; i < 100; ++i) {
		float x = scaleFactor * i * cos(i + (PI));
		float y = scaleFactor * i * sin(i + (PI));
		spiralPoints.push_back(std::make_pair(x, y));
#if 0
		std::cout << i << ": (" << px << ", " << py << ")" << '\n';
#endif
	}

	return spiralPoints;

}