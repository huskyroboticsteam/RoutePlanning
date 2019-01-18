#include "Server.cpp"

int main() {
	
	
	int out = socket(AF_INET, SOCK_DGRAM, 0);
		std::vector<unsigned char> packet; // start packet with time stamp
	packet.push_back(0x01); // represents component to be controlled

	// packet.data() first four bytes are the time stamp, fifth is the id, and sixth is the data
	struct sockaddr_in my_addr;
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(54000);
	inet_aton("10.0.0.7", &(my_addr.sin_addr));
	memset(&(my_addr.sin_zero), '\0', 8); 
	int sendOk = sendto(out, (const char*)packet.data(), packet.size() + 1, 0, (sockaddr*) &my_addr, sizeof(my_addr));
	
	if (sendOk == SOCKET_ERROR)
	{
#ifdef _WIN32
		std::cout << "That didn't work! " << WSAGetLastError();
		return false;
#else
		std::cout << "That didn't work! " << strerror(errno);
		return false;
#endif
	}
	else {
		return true;
	}
	return 0;
}