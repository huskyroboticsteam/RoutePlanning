#include <iostream>
#include <vector>
#include "Map.h"
#include "Server.h"

namespace RP 
{
	struct obstacle_vector {
		float distance;
		float angle;
	};
    class Controller
    {
        public:
            RP::Map map;
			void setDirection(float heading);
			bool setSpeed(float speed);
			void parsePacket(unsigned char packetID, unsigned char data[]);
			float curr_lat, curr_long, curr_dir;
			void addObstacles(std::vector<obstacle_vector> points);
			RP::Server* server;
    };
}

