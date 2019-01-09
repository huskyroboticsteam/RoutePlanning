#include <iostream>
#include "Simulator.h"

int main(void)
{
    // RoverPathfinding::Map map;
    // map.add_obstacle(RoverPathfinding::point{-4.0f, 5.0f}, RoverPathfinding::point{4.0f, 5.0f});
    // map.add_obstacle(RoverPathfinding::point{-4.0f, 6.0f}, RoverPathfinding::point{4.0f, 6.0f});
    // map.add_obstacle(RoverPathfinding::point{1.0f, 7.0f}, RoverPathfinding::point{3.0f, 10.0f});
    //    map.add_obstacle(std::make_pair(-1.0f, 7.0f), std::make_pair(-3.0f, 10.0f));

    // auto path = map.shortest_path_to(0, 0, 0, 10);
    // for (auto i : path)
        // std::cout << '(' << i.x << ", " << i.y << ')' << std::endl;
    // return (0);
    std::cout << "Testing started..." << std::endl;
    RoverPathfinding::Simulator sim;
    sim.update_agent();
    for (auto vo : sim.visible_obstacles())
    {
        std::cout << "(" << vo.p.x << ", " << vo.p.y << ") - (" << vo.q.x << ", " << vo.q.y << ")" << std::endl;
    }
    std::cout << "Done. Press any key to exit" << std::endl;
    getchar();
}
