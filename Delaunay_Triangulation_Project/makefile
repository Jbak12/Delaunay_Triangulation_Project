CXX = g++
CXXFLAGS = -std=c++17
TARGETS = main.x
SRCS = main.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGETS)

%.x: %.o
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<


clean:
	rm -f $(OBJS) $(TARGETS) triangles.txt
	
run_python:
	python3 ReadFile.py

.PHONY: all run clean
