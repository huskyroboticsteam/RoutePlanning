#include "Controller.h"
#include <iostream>
#include <cstring>
#include <math.h>
#include "Server.cpp"

#define DATA_GPS 0xC0
#define DATA_MAG 0xC1

#define SET_SPEED 0x95
#define HEADING_CHANGE 0x97

#define TARGET_LAT 123.0
#define TARGET_LNG 321.0

#define EARTH_RADIUS 6371000

#define CONV_FACTOR_LNG 8.627
#define DEGREES_METER_LNG 0.0001
#define CONV_FACTOR_LAT 111319.9

namespace RoverPathfinding {
    RoverPathfinding::Map map;
    RoverPathfinding::Server server;

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
			//std::vector<point> path = map.shortest_path_to(lat, lng, TARGET_LAT, TARGET_LNG);
			std::vector<point> path;
			point nextPoint = path[0];
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
    }

    // angle must be in radians, dist in meters
    // formula source: stackoverflow q 53182179 (convert lat/long to XY); I simply did the reverse math
    point convertToLatLng(float curr_lat, float curr_lng, float curr_dir, float dist, float angle) {
		float delta_x = dist * cos(angle + curr_dir + M_PI/2);
		float delta_y = dist * sin(angle + curr_dir + M_PI/2);
		//std::cout << "delta_x: " << delta_x << " delta_y: " << delta_y << "\n";
		float delta_lng = delta_x / CONV_FACTOR_LNG * DEGREES_METER_LNG;
		float delta_lat = delta_y / CONV_FACTOR_LNG;
		//std::cout << "delta_lat: " << delta_lat << " delta_lng: " << delta_lng << "\n";
		point p;
		p.x = delta_lat + curr_lat;
		p.y = delta_lng + curr_lng;
		return p;
    }

	
}

