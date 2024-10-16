VALGRIND = valgrind --leak-check=full --track-origins=yes
CXXFLAGS = -O2 -Wall -I. -g -std=c++2a

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
	WARNING = echo '\033[0;31mValgrind is not supported on MacOS. Make sure to run your tests in zyBooks to check for memory safety and leaks. See the Project 3 guide for more info.\033[0m';
	VALGRIND =
	CXXFLAGS += -I/opt/homebrew/Cellar/googletest/1.14.0/include -L/opt/homebrew/Cellar/googletest/1.14.0/lib
endif

osm_main:
	g++ $(CXXFLAGS) application.cpp dist.cpp osm.cpp tinyxml2.cpp main.cpp -o osm_main

run_osm_main: osm_main
	@$(WARNING)
	$(VALGRIND) ./osm_main
