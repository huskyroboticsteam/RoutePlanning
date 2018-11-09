#include <vector>
#include <string>
#include <fstream>
#include <cctype>
#include "Simulator.h"
#include "Agent.h"

RoverPathfinding::Simulator::Simulator(const std::string &grid_path, const Agent &agt, float init_bearing,
                                       simulator_config conf) : agent(agt), config(conf), bearing(init_bearing)
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
                    agent_pos = std::make_pair(rows, col);
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

std::string RoverPathfinding::Simulator::as_str()
{
    std::string result = "";
    // TODO distinguish seen cells with unseen ones
    for (grid_size_type r = 0; r < rows; r++)
    {
        for (grid_size_type c = 0; c < cols; c++)
        {
            if (r == agent_pos.first && c == agent_pos.second)
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
