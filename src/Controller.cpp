#include "Controller.h"
#include <iostream>
#include <cstring>

#define DATA_GPS 0xC0
#define DATA_MAG 0xC1

#define TARGET_LAT 123.0
#define TARGET_LONGITUDE 321.0

namespace RoverPathfinding {
    RoverPathfinding::Map map;
    
    // using given packet data and server send a packet containing either a direction or motor power
    void parsePacket(unsigned char packetID, unsigned char data[]) {
        if (packetID == DATA_GPS) {
            float lat = 0.0;
            std::memcpy(&lat, data, sizeof(float));
            float longitude = 0.0;
            std::memcpy(&longitude, &data[sizeof(float)], sizeof(float));
        } else if (packetID == DATA_MAG) {
            float x = y = z = 0.0;
            std::memcpy(&x, data, sizeof(float));
            std::memcpy(&y, &data[sizeof(float)], sizeof(float));
            std::memcpy(&z, &data[2 * sizeof(float)], sizeof(float));
        }
    }
}

