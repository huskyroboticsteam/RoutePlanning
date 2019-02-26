#include "WorldCommunicator.hpp"

WorldCommunicator::WorldCommunicator() : listenThread(&WorldCommunicator::listen, this) {
	
}

void WorldCommunicator::update() {
	std::vector<char> nextPacket;
	bool gotAPacket = false;
	mtx.lock();
	if(!packetQ.empty()) {
		nextPacket = packetQ.front();
		packetQ.pop();
		gotAPacket = true;
	}
	else {
		return;
	}
	mtx.unlock();
	
	// Process Packet
	// Assuming format: ID (char denoting action) and Data (float denoting amount)
	char id = nextPacket.at(0);
	float* data = (float*)&(nextPacket.at(1));

	// // Assuming id 0 = move and id 1 = turn
	// This doesn't work
	// if (id == 0) {
	// 	move(*data);
	// }
	// else {
	// 	turn(*data);
	// }
}

// Listens for packet 
void WorldCommunicator::listen() {
	while(true) {
		std::vector<char> buf;
		//TODO: listen for a packet;
		mtx.lock();
		packetQ.push(buf);
		mtx.unlock();
	}
}
