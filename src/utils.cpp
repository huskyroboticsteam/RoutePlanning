#include <algorithm>
#include <cmath>
#include "utils.hpp"

#define PI 3.14159265359

#define CONV_FACTOR_LNG 8.627
#define DEGREES_METER_LNG 0.0001
#define CONV_FACTOR_LAT 111319.9

bool RP::point::operator==(const point &p) const
{
    return x == p.x && y == p.y;
}

bool RP::point::operator!=(const point &p) const
{
    return !(*this == p);
}

float RP::deg_to_rad(float deg)
{
    return (deg * PI / 180.0f);
}

float RP::rad_to_deg(float rad)
{
    return (rad * 180.0f / PI);
}

float RP::normalize_angle(float rad)
{
    rad = std::fmod(rad, 2 * PI);
    if (rad < 0)
        rad += 2 * PI;
    return rad;
}

RP::point RP::intersection(point A, point B, point C, point D)
{
    // Line AB represented as a1x + b1y = c1
    float a1 = B.y - A.y;
    float b1 = A.x - B.x;
    float c1 = a1 * (A.x) + b1 * (A.y);

    // Line CD represented as a2x + b2y = c2
    float a2 = D.y - C.y;
    float b2 = C.x - D.x;
    float c2 = a2 * (C.x) + b2 * (C.y);

    float determinant = a1 * b2 - a2 * b1;

    // The lines are parallel. This is simplified
    // by returning a pair of FLT_MAX
    if (-1e-7 <= determinant && determinant <= 1e-7)
        return (point{INFINITY, INFINITY});

    float x = (b2 * c1 - b1 * c2) / determinant;
    float y = (a1 * c2 - a2 * c1) / determinant;
    return (point{x, y});
}

//start, end, circle, and R are in lat/long coordinates
bool RP::segment_intersects_circle(point start,
                                                 point end,
                                                 point circle,
                                                 float R)
{

    point direction{end.x - start.x, end.y - start.y};
    point center_to_start{start.x - circle.x, start.y - circle.y};
    float a = direction.x * direction.x + direction.y * direction.y;
    float b = 2 * (center_to_start.x * direction.x + center_to_start.y * direction.y);
    float c = (center_to_start.x * center_to_start.x + center_to_start.y * center_to_start.y) + R * R;

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
int RP::orientation(point p, point q, point r)
{
    float v = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (-1e-7 <= v && v <= 1e-7)
        return COLINEAR;

    return ((v > 0.0f) ? CLOCKWISE : COUNTERCLOCKWISE);
}

//Tells if r is on segment pq
bool RP::on_segment(point p, point q, point r)
{
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}

//Probably returns whether p1q1 and p2q2 intersect
bool RP::segments_intersect(point p1, point p2, point q1, point q2)
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
RP::point RP::center_point_with_radius(RP::point cur, RP::point p, RP::point q, float R)
{
    point vec{-p.y + q.y, p.x - q.x};
    float len = sqrt((vec.x * vec.x) + (vec.y * vec.y));
    vec.x = vec.x * R / len;
    vec.y = vec.y * R / len;
    point result = point{(p.x + q.x) / 2.0f, (p.y + q.y) / 2.0f};
    int o = orientation(cur, p, q);
    if (o == CLOCKWISE)
    {
        result.x += vec.x;
        result.y += vec.y;
    }
    else //o is COUNTERCLOCKWISE
    {
        result.x -= vec.x;
        result.y -= vec.y;
    }
    return (result);
}

float RP::dist_sq(point p1, point p2)
{
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

bool RP::within_radius(point p1, point p2, float R)
{
    return (dist_sq(p1, p2) <= R * R);
}

#define R_EARTH 6371.0088 // in km

// given lat, long, bearing (in degrees), and distance (in km), returns a new point
RP::point RP::lat_long_offset(float lat1, float lon1, float brng, float dist)
{
    dist /= 1000.0f;

    lat1 = deg_to_rad(lat1);
    lon1 = deg_to_rad(lon1);
    brng = deg_to_rad(brng);
    float lat2 = asin(sin(lat1) * cos(dist / R_EARTH) + cos(lat1) * sin(dist / R_EARTH) * cos(brng));
    float lon2 = lon1 + atan2(sin(brng) * sin(dist / R_EARTH) * cos(lat1), cos(dist / R_EARTH) - sin(lat1) * sin(lat2));
    lat2 = rad_to_deg(lat2);
    lon2 = rad_to_deg(lon2);
    return (point{lat2, lon2});
}

// Given two xy coordinates in degrees, returns the distance between them in meters.
RP::point RP::lat_long_to_meters(RP::point pt, RP::point origin)
{
    return point{(pt.x - origin.x) * 87029, (pt.y - origin.y) * 111111};
}

// generates maxpoints points in spiral formation in relation to the offset with armDist (m) between arms and returns in vector
std::deque<RP::point> RP::generate_spiral(float armDist, int maxPoints, float xOffset, float yOffset)
{
    std::deque<point> spiralPoints;

    for (int i = 0; i < maxPoints; ++i)
    {
		float angle = (PI / 5) * i;
		spiralPoints.push_back(convertToLatLng(xOffset, yOffset, 0, armDist, angle));
    }

    return spiralPoints;
}

// All angles should already be normalized and in radians
bool RP::within_angle(float ang, float lower, float upper)
{
    if (upper < lower)
    {
        upper += 2 * PI;
    }
    return ang >= lower && ang <= upper;
}

RP::point RP::polar_to_cartesian(point origin, float r, float theta)
{
    return RP::point{origin.x + r * std::cos(theta), origin.y + r * std::sin(theta)};
}

float RP::relative_angle(point o, point p)
{
    return std::atan((p.y - o.y) / (p.x - o.x));
}

bool RP::same_point(const point &p, const point &q, float tol)
{
    return std::sqrt(dist_sq(p, q)) <= tol;
}

RP::point RP::convertToLatLng(float lat, float lng, float dir, float dist, float angle) {
		float delta_x = dist * cos(angle + dir + M_PI/2);
		float delta_y = dist * sin(angle + dir + M_PI/2);
		//std::cout << "delta_x: " << delta_x << " delta_y: " << delta_y << "\n";
		float delta_lng = delta_x / CONV_FACTOR_LNG * DEGREES_METER_LNG;
		float delta_lat = delta_y / CONV_FACTOR_LAT;
		//std::cout << "delta_lat: " << delta_lat << " delta_lng: " << delta_lng << "\n";
		point p;
		p.x = delta_lat + lat;
		p.y = delta_lng + lng;
		return p;
}
