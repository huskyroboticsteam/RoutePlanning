#include "WorldCommunicator.hpp"
#include <chrono>


WorldCommunicator::WorldCommunicator()  {
	in = socket(AF_INET, SOCK_DGRAM, 0);
	// Bind in socket (copied from Server.cpp)
	sockaddr_in serverHint;
	serverHint.sin_addr.s_addr = INADDR_ANY;
	serverHint.sin_family = AF_INET;
	serverHint.sin_port = htons(54000);
	if (bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR)
	{
		std::cout << "Can't bind socket! " << strerror(errno) << std::endl;
	}
	
	out = socket(AF_INET, SOCK_DGRAM, 0);
	// Set up the address we should send to:
	send_to.sin_family = AF_INET;
	send_to.sin_port = htons(54000);
	inet_aton("127.0.0.1", &(send_to.sin_addr));
	memset(&(send_to.sin_zero), '\0', 8);
	listenThread = std::thread(&WorldCommunicator::listen, this);
}

void WorldCommunicator::update(const RP::point& position, const float& rotation, float& move, float& turn) {
	std::vector<unsigned char> nextPacket;
	mtx.lock();
	if(!packetQ.empty()) {
		nextPacket = packetQ.front();
		packetQ.pop();
		std::cout << "Update thread got a packet" <<  timer << std::endl;
	}
	else {
		mtx.unlock();
		return;
	}
	mtx.unlock();
	
	// Process Packet
	// Assuming format: ID (char denoting action) and Data (float denoting amount)
	// Ignore time stamp 
	char id = nextPacket.at(4);
	float data;
	memcpy(&data, &nextPacket[5], sizeof(float));
	// // Assuming id 0 = move and id 1 = turn
	// This doesn't work
	if (id == 0) {
		move = data;
		std::cout << "Update thread got a move packet" <<  timer << std::endl;
	}
	else {
		turn = data;
		std::cout << "Update thread got a turn packet" <<  timer << std::endl;
	}
	
	if(timer % framesPerGPS == 0) {
		std::vector<unsigned char> data(2*sizeof(float));
		memcpy(&data[0], &position.x, sizeof(float));
		memcpy(&data[sizeof(float)], &position.y, sizeof(float));
		send_action(data, gpsId);
	}
	if(timer % framesPerMag == 0) {
		std::vector<unsigned char> data(sizeof(float));
		memcpy(&data[0], &rotation, sizeof(float));
		send_action(data, magId);
	}
	
	timer++;
}


// Listens for packet 
void WorldCommunicator::listen() {
	while(true) {
		std::vector<unsigned char> buf(256);
		
		sockaddr_in client;
		memset(&client, 0, sizeof(sockaddr_in));
		
		unsigned int clientLength = sizeof(client);
		
		int bytesIn = recvfrom(in, &buf.front(), 256, 0, (sockaddr*)&client, &clientLength);
		char clientIp[256];
		memset(clientIp, 0, 256);
		inet_ntop(AF_INET, &client.sin_addr, clientIp, 256);
		std::cout << "Listen thread got packet from " << clientIp << std::endl;
		if (bytesIn == SOCKET_ERROR) 
		{
			std::cout << "Error receiving from client " << strerror(errno) << std::endl;
		}
		
		mtx.lock();
		packetQ.push(buf);
		mtx.unlock();
		std::this_thread::sleep_for (std::chrono::milliseconds(1));
	}
}

bool WorldCommunicator::send_action(std::vector<unsigned char> data, const unsigned char id) {
	std::vector<unsigned char> packet;
	for(int i = 0; i < 4; i++) {
		packet.push_back(0); //fake time stamp
	}
	packet.push_back(id); // represents component to be controlled
	for (unsigned i = 0; i < data.size(); i++) {
		packet.push_back(data[i]); // represents speed/position to be sent
	}

	// packet.data() first four bytes are the time stamp, fifth is the id, and the rest is the data
	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*)&send_to, sizeof(send_to));
	if (sendOk == SOCKET_ERROR) {
		std::cout << "That didn't work! " << strerror(errno);
		return false;
	}
	else {
		return true;
	}
}