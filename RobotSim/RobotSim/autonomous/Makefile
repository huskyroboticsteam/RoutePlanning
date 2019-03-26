CC := g++
SDIR := .
ODIR := obj
SOURCES := utils.cpp mapper.cpp timer.cpp memorizer.cpp splitMapper.cpp quadMapper.cpp pather.cpp
_OBJECTS := $(SOURCES:.cpp=.o)
OBJECTS := $(patsubst %,$(ODIR)/%,$(_OBJECTS))
CFLAGS := -std=c++11 -g

all: $(OBJECTS)

$(ODIR)/%.o: $(SDIR)/%.cpp $(SDIR)/%.hpp
	$(CC) -c -o $@ $< $(CFLAGS) 
