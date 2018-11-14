#include <vector>
#include <string>
#include <fstream>
#include <cctype>
#include <cmath>
#include "Simulator.h"
#include "Map.h"

RoverPathfinding::Simulator::Simulator(const std::string &grid_path, const Map &map, float init_bearing,
                                       simulator_config conf) : map(map), cur_brng(init_bearing), config(conf)
{
    load_map(grid_path);
}

void RoverPathfinding::Simulator::load_map(const std::string &path)
{
    std::ifstream infile(path); // TODO error handling
    std::string line;
    rows = 0;
    bool origin_found = false;
    bool target_found = false;
    while (std::getline(infile, line))
    {
        if (rows + 1 > MAX_GRID_RESOLUTION || line.length() > MAX_GRID_RESOLUTION)
            throw std::range_error("Provided grid file exceeded maximum grid resolution.");
        for (grid_size_type col = 0; col != line.length(); col++)
        {
            grid[rows][col] = line.at(col) != '0'; // anything not 0 is an obstacle
            switch (std::tolower(line.at(col)))
            {
            case 'x':
                grid[rows][col] = true;
                break;
            case 'o':
                if (!origin_found)
                {
                    origin_found = true;
                    grid[rows][col] = false;
                    cur_pos = std::make_pair(rows, col);
                }
                break;
            case 't':
                if (!target_found)
                {
                    target_found = true;
                    grid[rows][col] = false;
                    target_pos = std::make_pair(rows, col);
                }
                break;
            default:
                grid[rows][col] = false;
                break;
            }
        }
        rows++;
    }
    if (!origin_found || !target_found)
        throw std::exception("No origin or target in map file.");
    cols = line.length(); // This assumes all rows have the same number of columns
}

void RoverPathfinding::Simulator::update_agent()
{
    
}

// Currently abandoned, marginally faster but a nightmare to implement FoV algorithm 
// void RoverPathfinding::Simulator::update_agent()
// {
// // Scan around for obstacles
// #define X_QUAD \
//     int[] { 1, -1, -1, 1 }
// #define Y_QUAD \
//     int[] { 1, 1, -1, -1 }
//     /*
//     pseudocode
//     1) define function that returns quadrant number (1-4) given x and y values
//     2) use tan() to find x,y values of angle limits. use function to get quadrant values
//     3)
//     for int q = start_quad; q <= end_quad; q++:
//         x_sign, y_sign = get_xy_signs(q)
//         // TODO x boundaries should also depend on quadrant to avoid double counting
//         for (int x = x_sign * d; x != 0; x -= x_sign * 1)
//             TODO y's starting and ending boundaries should depend on both
//             1) remaining len allowed for y (sqrt(d^2-x^2))
//             2) boundary of quadrant (e.g. 1st quadrant y>=0) as well as angle (e.g. x * tan(start)),
//             whichever is tighter
//     */

//     float s_ang = RoverPathfinding::normalize_angle(RoverPathfinding::deg_to_rad(cur_brng - config.vision_angle / 2));
//     float e_ang = RoverPathfinding::normalize_angle(RoverPathfinding::deg_to_rad(cur_brng + config.vision_angle / 2));
//     float s_quad = RoverPathfinding::which_quadrant(s_ang);
//     float e_quad = RoverPathfinding::which_quadrant(e_ang);
//     float d = config.vision_depth;
//     for (unsigned int q = s_quad; q <= e_quad; q++)
//     {
//         int x_sign = RoverPathfinding::x_sign(q);
//         int y_sign = RoverPathfinding::y_sign(q);
//         for (int x = x_sign * d; x != 0; x -= x_sign)
//         {
//             float y = y_sign * std::sqrt(d * d - x * x);
//         }
//     }

//     // TODO check orthogonal parts where x == 0 || y == 0
// }

// // Tiny helper function to tell if x is in range of the given quadrant
// bool x_in_quad(int x, unsigned int quad)
// {
//     x = std::abs(x);
//     if (quad % 2 == 0) // This prevents double-counting (hopefully)
//         return x >= 0;
//     return x > 0;
// }

std::string RoverPathfinding::Simulator::as_str()
{
    std::string result = "";
    // TODO distinguish seen cells with unseen ones
    for (grid_size_type r = 0; r < rows; r++)
    {
        for (grid_size_type c = 0; c < cols; c++)
        {
            if (r == cur_pos.first && c == cur_pos.second)
            {
                result.append("A");
            }
            else if (r == target_pos.first && c == target_pos.second)
            {
                result.append("T");
            }
            else
            {
                result.append(grid[r][c] ? "x" : "-");
            }
        }
        result.append("\n");
    }
    return result;
}
