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
            point dst;
			point target;
			float curr_lat; 
            float curr_lng; 
            float curr_dir;
            bool in_spiral_radius();
            bool found_ball();
            RP::point convertToLatLng(float dist, float dir);
			std::vector<point> targetSites;
            std::vector<point> spiralPts;
    };
}
