#include <iostream>
#include "Map.h"
#include "Map.cpp"

int main(void)
{
    RoverPathfinding::Map map;
    //map.add_obstacle(std::make_pair(-4.0f, 5.0f), std::make_pair(4.0f, 5.0f));
    //map.add_obstacle(std::make_pair(-4.0f, 6.0f), std::make_pair(4.0f, 6.0f));
    //map.add_obstacle(std::make_pair(1.0f, 7.0f), std::make_pair(3.0f, 10.0f));
    //    m.AddObstacle(std::make_pair(-1.0f, 7.0f), std::make_pair(-3.0f, 10.0f));

    auto path = map.shortest_path_to(0, 0, 0, 10);
    for (auto i : path)
        std::cout << '(' << i.x << ", " << i.y << ')' << std::endl;
    return (0);
}
