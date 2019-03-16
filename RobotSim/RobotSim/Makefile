CC := g++
SDIR := .
ODIR := obj
SOURCES := main.cpp agent.cpp grid.cpp obstacle.cpp simController.cpp Simulator.cpp ui.cpp
_OBJECTS := $(SOURCES:.cpp=.o)
OBJECTS := $(patsubst %,$(ODIR)/%,$(_OBJECTS))

AUTO_ODIR := autonomous/obj
AUTO_SOURCES := $(wildcard autonomous/*.cpp)
_AUTO_OBJECTS := $(notdir $(AUTO_SOURCES:.cpp=.o))
AUTO_OBJECTS := $(patsubst %,$(AUTO_ODIR)/%,$(_AUTO_OBJECTS))

SFML := C:/SFML-2.5.1/
INC := -I$(SFML)include
LINK := -L$(SFML)lib -lsfml-graphics -lsfml-audio -lsfml-window -lsfml-system
CFLAGS := -std=c++11 -g
OUT := build/simulator.exe

all: $(OUT)

$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CC) -c $(INC) -o $@ $< $(CFLAGS) 

$(AUTO_OBJECTS): $(AUTO_SOURCES)
	$(MAKE) -C autonomous $(MAKECMDGOALS)

$(OUT): $(OBJECTS) $(AUTO_OBJECTS)
	$(CC) $(CFLAGS) $(AUTO_OBJECTS) $(OBJECTS) $(LINK) -o $(OUT)

.PHONY: clean
clean:
	rm $(OUT)
	rm -rf obj/*
