CC = g++
SDIR = .
ODIR = obj
SOURCES = Controller.cpp Map.cpp Server.cpp utils.cpp RobotEKF.cpp
_OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(ODIR)/%,$(_OBJECTS))
CFLAGS = -std=c++11 -g
OUT = build/routeplanning.exe

all: $(OUT)

$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS) 


$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS)  Tennisball/CMakeFiles/still.dir/src/detector.cpp.o -L/usr/local/lib -I/usr/include `pkg-config --libs opencv` -lopencv_dnn -o $(OUT) -lpthread

.PHONY: clean
clean:
	rm $(OUT)
