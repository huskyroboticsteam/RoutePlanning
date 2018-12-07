
TestMap:
	g++ -std=c++11 -ggdb src/TestMap.cpp src/Simulator.cpp src/utils.cpp -o ./build/TestMap

clean:
	rm -rf ./build/*
