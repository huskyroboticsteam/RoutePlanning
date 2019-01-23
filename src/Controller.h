#include <iostream>
#include <vector>
#include "Map.h"
#include "Server.h"
namespace RoverPathfinding 
{
    class Controller
    {
        public:
            RoverPathfinding::Map map;
			void setDirection(float heading);
			bool setSpeed(float speed);
			void parsePacket(unsigned char packetID, unsigned char data[]);
			RoverPathfinding::Server* server;
    };
}
