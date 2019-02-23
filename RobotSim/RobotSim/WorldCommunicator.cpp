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
	mtx.unlock();
	//TODO: Process the packet
}

void WorldCommunicator::listen() {
	while(true) {
		std::vector<char> buf;
		//TODO: listen for a packet;
		mtx.lock();
		packetQ.push(buf);
		mtx.unlock();
	}
}
