# Compiler and flags
CXX = g++
CXXFLAGS = -O3 -Wall -std=c++17 -fPIC

# pybind11 and Python config
PYBIND11_INCLUDES := $(shell python3 -m pybind11 --includes)
PYTHON_LDFLAGS := $(shell python3-config --ldflags)
PYTHON_EXT_SUFFIX := $(shell python3-config --extension-suffix)

# Eigen include path
EIGEN_INCLUDE = /usr/include/eigen3

# Output shared object
TARGET = algoritm_modules_cpp$(PYTHON_EXT_SUFFIX)

# Source file
SRCS = algorithm_bindings.cpp

# Include paths for all module dependencies
INCLUDES = \
	-I$(EIGEN_INCLUDE) \
	-I../src \
	-I../gradientDescent \
	-I../metaheuristics \
	$(PYBIND11_INCLUDES)

# Build rule
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared $(SRCS) $(PYTHON_LDFLAGS) -o $@

clean:
	rm -f *.so *.o $(TARGET)
