CC := g++
SDIR := .
ODIR := obj
SOURCES := utils.cpp Mapper.cpp timer.cpp memorizer.cpp splitMapper.cpp
_OBJECTS := $(SOURCES:.cpp=.o)
OBJECTS := $(patsubst %,$(ODIR)/%,$(_OBJECTS))
CFLAGS := -std=c++11 -g

all: $(OBJECTS)

$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS) 
