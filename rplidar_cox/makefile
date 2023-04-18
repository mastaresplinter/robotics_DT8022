CC=g++
CFLAGS=-c -Wall
LDFLAGS=-pthread
SOURCES=src/cox.cpp src/rplidar.c src/server_rp.c src/runner.cpp
OBJECTS=$(addprefix build/,$(addsuffix .o,$(notdir $(basename $(SOURCES)))))
EXECUTABLE=build/runner

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

build/%.o: src/%.cpp
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: src/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)