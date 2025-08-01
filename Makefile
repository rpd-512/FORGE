CXX = g++
SRC = main.cpp
OUT = FORGE_linux

PKG_CFLAGS := $(shell pkg-config --cflags yaml-cpp)
PKG_LIBS   := $(shell pkg-config --libs yaml-cpp)

EIGEN_INCLUDE = -I/usr/include/eigen3
CXXFLAGS = -std=c++17

all:
	$(CXX) $(CXXFLAGS) $(SRC) $(EIGEN_INCLUDE) $(PKG_CFLAGS) -o $(OUT) $(PKG_LIBS)

clean:
	rm -f $(OUT)
