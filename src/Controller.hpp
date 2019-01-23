#include <iostream>
#include <vector>
#include "Map.hpp"
#include "Server.hpp"
namespace RoverPathfinding 
{
    class Controller
    {
        public:
            RoverPathfinding::Map map;
			RoverPathfinding::Server server;
			bool setDirection(float heading);
			bool setSpeed(float speed);
			void parsePacket(unsigned char packetID, unsigned char data[]);
            void addObstacle(float curr_lat, float curr_lng, float curr_dir, float dist1, float dir1, float dist2, float dir2);
        
        private:
            point convertToLatLng(float curr_lat, float curr_lng, float curr_dir, float dist, float dir);
    };
}
