CC = g++
SDIR = .
ODIR = obj
SOURCES = main.cpp agent.cpp map.cpp
_OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(ODIR)/%,$(_OBJECTS))
SFML = ../SFML-2.5.1/
INC = -I$(SFML)include
LINK = -L$(SFML)lib
CFLAGS = -std=c++11
OUT = build/simulator

all: $(OUT)

$(ODIR)/%.o: $(SDIR)/%.cpp 
	$(CC) -c $(INC) -o $@ $< $(CFLAGS) 

$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LINK) -lsfml-graphics -lsfml-audio -lsfml-window -lsfml-system -ggdb -o $(OUT)

clean:
	rm -rf $(ODIR)/*
	rm -rf build/*