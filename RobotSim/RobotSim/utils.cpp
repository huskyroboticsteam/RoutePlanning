#include <algorithm>
#include <cmath>
#include "utils.hpp"

#define PI 3.14159265359
#define FLOAT_TOL -1e-4 // floating point error tolerance. Set to high value for now
                        // since we don't need too much precision

bool RoverPathfinding::point::operator==(const point &p) const
{
    return x == p.x && y == p.y;
}

bool RoverPathfinding::point::operator!=(const point &p) const
{
    return !(*this == p);
}

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

RoverPathfinding::point RoverPathfinding::intersection(point A, point B, point C, point D)
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
bool RoverPathfinding::segment_intersects_circle(point start,
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
int RoverPathfinding::orientation(point p, point q, point r)
{
    float v = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (-1e-7 <= v && v <= 1e-7)
        return COLINEAR;

    return ((v > 0.0f) ? CLOCKWISE : COUNTERCLOCKWISE);
}

//Tells if r is on segment pq
bool RoverPathfinding::on_segment(point p, point q, point r)
{
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
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

// returns the intersection between two points. returned point is at {inf, inf} if
// no intersection exists
RoverPathfinding::point RoverPathfinding::segments_intersection(point a, point b, point c, point d)
{
    point x = intersection(a, b, c, d);
    if (x.x == INFINITY)
        return x;
    if (!within_segment(a, b, x) || !within_segment(c, d, x))
        return point{INFINITY, INFINITY};
    return x;
}

//returns whether c is on ab assuming that abc is a line
bool RoverPathfinding::within_segment(point a, point b, point c)
{
    // dot product of ab and ac
    float dotprod = ((b.y - a.y) * (c.y - a.y) + (b.x - a.x) * (c.x - a.x));
    return dotprod >= FLOAT_TOL && dotprod <= dist_sq(a, b) + FLOAT_TOL;
}

//Returns a point in the center of segment pq and then moves it R towards cur
RoverPathfinding::point RoverPathfinding::center_point_with_radius(RoverPathfinding::point cur, RoverPathfinding::point p, RoverPathfinding::point q, float R)
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

float RoverPathfinding::dist_sq(point p1, point p2)
{
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
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
    return (point{lat2, lon2});
}

// Given two xy coordinates in degrees, returns the distance between them in meters.
RoverPathfinding::point RoverPathfinding::lat_long_to_meters(RoverPathfinding::point pt, RoverPathfinding::point origin)
{
    return point{(pt.x - origin.x) * 87029, (pt.y - origin.y) * 111111};
}

// generates 100 points in spiral formation around origin and returns in vector
std::vector<RoverPathfinding::point> RoverPathfinding::generate_spiral()
{
    int scaleFactor = 10;
    std::vector<point> spiralPoints;

    for (int i = 0; i < 100; ++i)
    {
        int x = round(scaleFactor * i * cos(i + (PI)));
        int y = round(scaleFactor * i * sin(i + (PI)));
        spiralPoints.push_back(point{(float)x, (float)y});
#if 0
		std::cout << i << ": (" << px << ", " << py << ")" << '\n';
#endif
    }

    return spiralPoints;
}

// All angles should already be normalized and in radians
bool RoverPathfinding::within_angle(float ang, float lower, float upper)
{
    if (upper < lower)
    {
        return ang >= lower || ang <= upper;
    }
    // printf("Angle: %.2f; Lower: %.2f; Upper: %.2f\n", ang, lower, upper);
    return ang >= lower && ang <= upper;
}

// return if ab and ac are in the same direction, assuming abc is a line
bool RoverPathfinding::same_dir(point a, point b, point c)
{
    return ((b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)) >= FLOAT_TOL;
}

RoverPathfinding::point RoverPathfinding::polar_to_cartesian(point origin, float r, float theta)
{
    return RoverPathfinding::point{origin.x + r * std::cos(theta), origin.y + r * std::sin(theta)};
}

float RoverPathfinding::relative_angle(point o, point p)
{
    return std::atan((p.y - o.y) / (p.x - o.x));
}

bool RoverPathfinding::same_point(const point &p, const point &q, float tol = 1e-6)
{
    return dist_sq(p, q) <= tol;
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
