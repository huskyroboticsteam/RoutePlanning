#include "Controller.hpp"
#include <iostream>
#include <cstring>
#include <math.h>
#include "Server.hpp"
#include "Map.hpp"

#define DATA_GPS 0xC0
#define DATA_MAG 0xC1

#define SET_SPEED 0x95
#define HEADING_CHANGE 0x97

#define TARGET_LAT 123.0
#define TARGET_LNG 321.0

int main() {
	std::vector<RP::point> targetSites(0);
	std::cout << "Enter coordinates: " << std::endl;
	float lat = 0;
	float lng = 0;
	RP::point p;
	std::cin >> p.x;
	std::cin >> p.y;
	while(p.x != -1) {
		targetSites.push_back(p);
		std::cin >> p.x;
		std::cin >> p.y;
		
	}
	for(int i = 0; i < targetSites.size(); i++) {
		std::cout << targetSites[i].x << ", " << targetSites[i].y << std::endl;
	}
	std::cout << "Enter Current Position: " << std::flush;
	std::cin >> p.x;
	std::cin >> p.y;
	RP::Controller controller(p, targetSites);
	
}


namespace RP {


	Controller::Controller(const point& cur_pos, std::vector<point> targetSites)
      : map(cur_pos, targetSites[0], std::list<RP::line>())	{
		this->targetSites = targetSites;
		
		curr_lat = cur_pos.x;
		curr_lng = cur_pos.y;
		server = Server();
	}
    // TODO: split packet into separate parts (timestamp, packetID, data)
    // TODO: Feed lat/long to map to find vector pointing to next destination
    // TODO: Feed x/y/z into ??? to find current orientation, then we can find how much to turn

    // using given packet data and server send a packet containing either a direction or motor power
	bool Controller::setDirection(float delta_heading) {
        std::vector<unsigned char> data(4);
        std::memcpy(&data[0], &delta_heading, 4);
		return server.send_action(delta_heading, HEADING_CHANGE); 
	}

	bool Controller::setSpeed(float speed) {
		std::vector<unsigned char> data(4);
		std::memcpy(&data[0], &speed, 4);
		return server.send_action(data, SET_SPEED);
	}
    void Controller::parsePacket(unsigned char packetID, unsigned char data[]) {
        // GPS has latitude and lng
        if (packetID == DATA_GPS) {
            float lat = 0.0;
            std::memcpy(&lat, data, sizeof(float));
            float lng = 0.0;
            std::memcpy(&lng, &data[sizeof(float)], sizeof(float));
			curr_lat = lat;
			curr_lng = lng;
			//std::vector<point> path = map.shortest_path_to(lat, lng, TARGET_LAT, TARGET_LNG);
			float delta_heading = atan2(nextPoint.y - lng, nextPoint.x - lat);
			//float heading = atan2(nextPoint.y - longitude, nextPoint.x - lat);
			setDirection(delta_heading);
			setSpeed(1.0); //TODO: figure out how setting speed and heading actually works
	    // Magnometer has x, y, z values
        } else if (packetID == DATA_MAG) {
            float x = 0.0, y = 0.0, z = 0.0;
            std::memcpy(&x, data, sizeof(float));
            std::memcpy(&y, &data[sizeof(float)], sizeof(float));
            std::memcpy(&z, &data[2 * sizeof(float)], sizeof(float));
        }
		
		nextPoint = map.shortest_path_to()[0];
		
    }

    void Controller::addObstacle(float dist1, float dir1, float dist2, float dir2) {
        RP::point latlng1 = convertToLatLng(dist1, dir1);
        RP::point latlng2 = convertToLatLng(dist2, dir2);
        map.add_obstacle(latlng1, latlng2);
    }

	
	void Controller::foundTennisBall(float dist, float dir) {
		target = convertToLatLng(dist - 1, dir);
		map.shortest_path_to();
	}
    // angle must be in radians, dist in meters
    // formula source: stackoverflow q 53182179 (convert lat/long to XY); I simply did the reverse math
    RP::point Controller::convertToLatLng(float dist, float angle) {
		return RP::convertToLatLng(curr_lat, curr_lng, curr_dir, dist, angle); 
    }

}

