CC = g++
SDIR = .
ODIR = obj
SOURCES = main.cpp agent.cpp grid.cpp obstacle.cpp utils.cpp Simulator.cpp Map.cpp timer.cpp autoController.cpp
_OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(ODIR)/%,$(_OBJECTS))
SFML = C:/SFML-2.5.1/
INC = -I$(SFML)include
LINK = -L$(SFML)lib -lsfml-graphics -lsfml-audio -lsfml-window -lsfml-system
CFLAGS = -std=c++11 -g
OUT = build/simulator.exe

all: $(OUT)

$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CC) -c $(INC) -o $@ $< $(CFLAGS) 


$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LINK) -o $(OUT)

.PHONY: clean
clean:
	rm $(OUT)