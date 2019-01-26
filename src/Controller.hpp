#include <iostream>
#include <vector>
#include "Map.hpp"
#include "Server.hpp"
namespace RP
{
    class Controller
    {
        public:
			RP::Map map;
			RP::Server server;
			bool setDirection(float heading);
			bool setSpeed(float speed);
			void parsePacket(unsigned char packetID, unsigned char data[]);
            void addObstacle(float dist1, float dir1, float dist2, float dir2);
			void foundTennisBall(float dist, float dir);
			
        private:
            point convertToLatLng(float dist, float dir);
			float curr_lat, curr_lng, curr_dir;
    };
}
