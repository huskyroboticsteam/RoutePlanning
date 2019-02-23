#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <mutex>
#include <queue>
#include <thread>
#define SOCKET int
#define SOCKET_ERROR -1

class WorldCommunicator {
	public:
		// Sends and receives packets
		// To be called in the main update loop
		void update();
		WorldCommunicator();
	private:
		int timer;
		bool send_action(std::vector<unsigned char> data, unsigned char id);
		void listen();
		SOCKET out;
		std::mutex mtx;
		std::queue<std::array<char, 256>> packetQ;
		std::thread listenThread;
};