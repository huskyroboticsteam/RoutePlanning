CC = g++
SDIR = .
ODIR = obj
SOURCES = Controller.cpp Map.cpp Server.cpp utils.cpp
_OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(ODIR)/%,$(_OBJECTS))
CFLAGS = -std=c++11 -g
OUT = build/routeplanning.exe

all: $(OUT)

$(ODIR)/%.o: $(SDIR)/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS) 


$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(OUT)

.PHONY: clean
clean:
	rm $(OUT)