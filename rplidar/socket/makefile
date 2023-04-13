# Compiler settings
CC = g++
CFLAGS = -std=c++11 -Wall

# Directories and files
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
TARGET = run

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)

# Object files
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# Include directories
INCLUDES = -I $(INC_DIR)

# Default target
all: $(BUILD_DIR)/$(TARGET)

# Link target
$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

# Compile objects
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Clean up
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
