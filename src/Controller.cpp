#include "Controller.h"
#include <iostream>
#include <cstring>
#include <math.h>

#define DATA_GPS 0xC0
#define DATA_MAG 0xC1

#define TARGET_LAT 123.0
#define TARGET_LNG 321.0

#define HANKSVILLE_LAT 38.3730
#define HANKSVILLE_LNG 110.7140
#define EARTH_RADIUS 6371000

namespace RoverPathfinding {
    RoverPathfinding::Map map;

    // TODO: split packet into separate parts (timestamp, packetID, data)
    // TODO: Feed lat/long to map to find vector pointing to next destination
    // TODO: Feed x/y/z into ??? to find current orientation, then we can find how much to turn

    void setDirection(float heading);
    void setSpeed(float speed);
    
    // using given packet data and server send a packet containing either a direction or motor power
    void parsePacket(unsigned char packetID, unsigned char data[]) {
        // GPS has latitude and lng
        if (packetID == DATA_GPS) {
            float lat = 0.0;
            std::memcpy(&lat, data, sizeof(float));
            float lng = 0.0;
            std::memcpy(&lng, &data[sizeof(float)], sizeof(float));
			std::vector<point> path = map.shortest_path_to(lat, lng, TARGET_LAT, TARGET_LNG);
			point nextPoint = path[0];
			float heading = atan2(nextPoint.y - lng, nextPoint.x - lat);
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

    point convertToLatLng(float curr_lat, float curr_lng, float curr_dir, float dist, float angle) {
        float x = dist * cos(angle);
        float y = dist * sin(angle);
        float lat = y / EARTH_RADIUS;
        float lng = x / (EARTH_RADIUS * cos(HANKSVILLE_LAT));
        point p;
        p.x = lng;
        p.y = lat;
    }


}

