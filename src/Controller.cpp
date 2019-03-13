#include "Controller.hpp"
#include <iostream>
#include <cstring>
#include <math.h>
#include <thread>
#include "Server.hpp"
#include "Map.hpp"

#define DATA_GPS 10
#define DATA_MAG 11

#define SET_SPEED 0x01
#define HEADING_CHANGE 0x02

#define TARGET_LAT 123.0
#define TARGET_LNG 321.0

#define FOLLOW_PATH 0
#define SPIRAL 1
#define FOUND_BALL 2

int main() {
	std::deque<RP::point> targetSites(0);
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
	while(true) {
		controller.update();
	}
}


namespace RP {


	Controller::Controller(const point& cur_pos, std::deque<point> targetSites)
      : map(cur_pos, targetSites[0], std::list<RP::line>()), server()	{
		this->targetSites = targetSites;
		state = FOLLOW_PATH;
		curr_lat = cur_pos.x;
		curr_lng = cur_pos.y;

		std::thread watchdogThread(&RP::Server::send_watchdog, &server);
	}

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

    void Controller::update() {
        // step 1: get obstacle data from camera
        while (!targetSites.empty()) {
    		// TODO: actually get obstacle data from camera
			// std:vector<obstacleVector> obstacles = METHOD_GOES_HERE
	    	// Bogus obstacle data for testing
            std::vector<obstacleVector> obstacles{ 
                obstacleVector{1,2}, obstacleVector{3,4}, obstacleVector{5,6} 
            };  

            // step 2: wait for server to give current location
			// Note: If Scarlet changes size of Timestamp, be sure to update the parsePacket paramaters below

			//TODO: make this a vector or shared_ptr
            unsigned char* firstPacket = server.go();
            parsePacket(firstPacket[4], &firstPacket[5]);
            unsigned char* secondPacket = server.go();
            parsePacket(secondPacket[4], &secondPacket[5]);
			//std::cout << "Controller got a packet" << std::endl;
            point nextPoint {0.0, 0.0};
            if (state == FOLLOW_PATH) {
                if (in_spiral_radius()) {
                    state = SPIRAL;
                    spiralPts = RP::generate_spiral(0.1, 100, curr_lng, curr_lat);
                }else if (found_ball()) {
                    dst = targetSites.front();
                    targetSites.pop_front();
                }else {
                    nextPoint = map.shortest_path_to()[0];
                }
            } else if (state == SPIRAL) {
                if (found_ball()) {
                    state = FOUND_BALL;
                } else {
                    // get and remove first element of spiralPts
                    dst = spiralPts.front();
                    spiralPts.pop_front();
                    nextPoint = map.shortest_path_to()[0];
                }
            } else { // if state is FOUND_BALL 
                if (targetSites.size() > 0) {
                    // get and remove first element of targetSites
                    dst = targetSites[0];
                }
            }

            // step 3: use current location and obstacle data to update map
            for (int i = 1; i < obstacles.size(); i++) {
                obstacleVector left = obstacles.at(i-1);
                point a{ (curr_lng + cos(left.angle)*left.distance), (curr_lat + sin(left.angle)*left.distance) };
                obstacleVector right = obstacles.at(i);
                point b{ (curr_lng + cos(right.angle)*right.distance), (curr_lat + sin(right.angle)*right.distance) };
                map.add_obstacle(a, b);
            }

            // step 5: use next point to send packets specifying new direction and speed to proceed
            float delta_heading = atan2(nextPoint.y - curr_lng, nextPoint.x - curr_lat);
            setDirection(delta_heading);
            setSpeed(1.0); //TODO: figure out how setting speed and heading actually works
        }
    }

    bool Controller::in_spiral_radius() {
        return true;
    }

    bool Controller::found_ball() {
        return true;
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
			std::cout << "lat: " << lat << " long: " << lng << std::endl;
	    // Magnometer has x, y, z values
        } else if (packetID == DATA_MAG) {
            float x = 0.0, y = 0.0, z = 0.0;
            std::memcpy(&x, data, sizeof(float));
           // std::memcpy(&y, &data[sizeof(float)], sizeof(float));
            //std::memcpy(&z, &data[2 * sizeof(float)], sizeof(float));
			std::cout << "direction: " << x << std::endl;
		}
		
    }

    void Controller::addObstacle(float dist1, float dir1, float dist2, float dir2) {
        RP::point latlng1 = convertToLatLng(dist1, dir1);
        RP::point latlng2 = convertToLatLng(dist2, dir2);
        map.add_obstacle(latlng1, latlng2);
    }

	
	void Controller::foundTennisBall(float dist, float dir) {
	}

    // angle must be in radians, dist in meters
    RP::point Controller::convertToLatLng(float dist, float angle) {
		return RP::convertToLatLng(curr_lat, curr_lng, curr_dir, dist, angle); 
    }

}

