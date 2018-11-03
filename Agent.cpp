#include "Agent.h"

// generates 100 points in spiral formation around origin and returns in vector 
std::vector<RoverPathfinding::point> RoverPathFinding::Agent::generate_spiral() 
{
	int scaleFactor = 5;
	std::vector<RoverPathFinding::point> spiralPoints;

	for (int i = 0; i < 100; ++i) {
		int x = round(scaleFactor * i * cos(i + (PI)));
		int y = round(scaleFactor * i * sin(i + (PI)));
		spiralPoints.push_back(std::make_pair(x, y));
#if 0
		std::cout << i << ": (" << px << ", " << py << ")" << '\n';
#endif
	}

	return spiralPoints;

}