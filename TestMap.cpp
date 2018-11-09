#include <iostream>
#include "Agent.h"
#include "Agent.cpp"

int main(void)
{
    RoverPathfinding::Agent agt;
    agt.add_obstacle(std::make_pair(-4.0f, 5.0f), std::make_pair(4.0f, 5.0f));
    agt.add_obstacle(std::make_pair(-4.0f, 6.0f), std::make_pair(4.0f, 6.0f));
    agt.add_obstacle(std::make_pair(1.0f, 7.0f), std::make_pair(3.0f, 10.0f));
//    m.AddObstacle(std::make_pair(-1.0f, 7.0f), std::make_pair(-3.0f, 10.0f));
    
    auto path = agt.shortest_path_to(0, 0, 0, 10);
    for(auto i : path)
	std::cout << '(' << i.first << ", " << i.second << ')' << std::endl;
    return(0);
}
