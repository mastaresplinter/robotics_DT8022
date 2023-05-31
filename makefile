CC=g++
CFLAGS=-c -Wall -lrt `pkg-config --cflags opencv` -I/usr/local/include
LDFLAGS=-pthread -lrobotic_gcc -lwiringPi `pkg-config --libs opencv` -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -L/opt/vc/lib
SOURCES=src/cox.cpp src/SpeedProfile.cpp src/rplidar.c src/server_rp.cpp src/task1.cpp src/task2.cpp src/task3.cpp src/writetask.cpp src/runner.cpp
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