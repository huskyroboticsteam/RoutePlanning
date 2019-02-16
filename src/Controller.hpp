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
			Controller(const point& cur_pos, std::vector<point> targetSites);
			bool setDirection(float heading);
			bool setSpeed(float speed);
			void update();
			void parsePacket(unsigned char packetID, unsigned char data[]);
            void addObstacle(float dist1, float dir1, float dist2, float dir2);
			void foundTennisBall(float dist, float dir);
        private:
            int state;
            RP::point convertToLatLng(float dist, float dir);
			std::vector<point> targetSites;
            std::vector<point> spiralPts;
			float curr_lat, curr_lng, curr_dir;
			RP::point target;
			RP::point nextPoint;
    };
}
