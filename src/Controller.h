#include <iostream>
#include <vector>
#include "Map.h"
#include "Server.h"

namespace RP 
{
    class Controller
    {
        public:
            RP::Map map;
			void setDirection(float heading);
			bool setSpeed(float speed);
			void parsePacket(unsigned char packetID, unsigned char data[]);
			float curr_lat, curr_long, curr_dir;
			RP::Server* server;
    };
}

