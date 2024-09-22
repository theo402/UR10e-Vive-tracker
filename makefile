CXX:=g++
CXXFLAGS:=-std=c++11 -Wall -I./openvr-2.2.3/headers 
LDFLAGS := -L./openvr-2.2.3/lib/linux64 -Wl,-rpath,$(PWD)/openvr-2.2.3/lib/linux64
LIBs:= -lopenvr_api -lpthread -lrtde

.PHONY: all clean

all: my_app

main.o: my_project/main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

my_app: main.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -g -o $@ $^ $(LIBs)

clean:
	rm -f *.o my_app