# Compiler and flags
CXX = g++
CXXFLAGS = -O3 -Wall -std=c++17 -fPIC

# pybind11 and Python config
PYBIND11_INCLUDES := $(shell python3 -m pybind11 --includes)
PYTHON_LDFLAGS := $(shell python3-config --ldflags)
PYTHON_EXT_SUFFIX := $(shell python3-config --extension-suffix)

# Output shared object
TARGET = collision_cpp$(PYTHON_EXT_SUFFIX)

# Only the cpp source file; headers are included automatically
SRCS = collision_bindings.cpp

# Build rule
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(PYBIND11_INCLUDES) -I../src -shared $(SRCS) $(PYTHON_LDFLAGS) -o $@

clean:
	rm -f *.so *.o $(TARGET)
