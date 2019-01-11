#include "Controller.h"
#include <iostream>
#include <cstring>
#include <math.h>

#define DATA_GPS 0xC0
#define DATA_MAG 0xC1

#define TARGET_LAT 123.0
#define TARGET_LONGITUDE 321.0

namespace RoverPathfinding {
    RoverPathfinding::Map map;
    // TODO: split packet into separate parts (timestamp, packetID, data)
    // TODO: Feed lat/long to map to find vector pointing to next destination
    // TODO: Feed x/y/z into ??? to find current orientation, then we can find how much to turn
    // using given packet data and server send a packet containing either a direction or motor power
    void parsePacket(unsigned char packetID, unsigned char data[]) {
        // GPS has latitude and longitude
        if (packetID == DATA_GPS) {
            float lat = 0.0;
            std::memcpy(&lat, data, sizeof(float));
            float longitude = 0.0;
            std::memcpy(&longitude, &data[sizeof(float)], sizeof(float));
			std::vector<point> path = map.shortest_path_to(lat, longitude, TARGET_LAT, TARGET_LONGITUDE);
			point nextPoint = path[0];
			float heading = atan2(path.y - longitude, path.x - latitude);
			setDirection(heading);
			setSpeed(1.0); //TODO: figure out how setting speed and heading actually works
	   // Magnometer has x, y, z values
        } else if (packetID == DATA_MAG) {
            float x = 0.0, y = 0.0, z = 0.0;
            std::memcpy(&x, data, sizeof(float));
            std::memcpy(&y, &data[sizeof(float)], sizeof(float));
            std::memcpy(&z, &data[2 * sizeof(float)], sizeof(float));
        }
    }
	
	void setDirection(float heading) {}
	void setSpeed(float speed) {}
}

