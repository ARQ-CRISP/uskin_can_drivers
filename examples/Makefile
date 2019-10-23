CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-g $(root-config --cflags) -std=c++11
PY_CFLAGS := $(shell python2.7-config --cflags)
PY_LDFLAGS := $(shell python2.7-config --ldflags)
LDFLAGS=-g $(root-config --ldflags)

LDLIBS=$(root-config --libs) -I/usr/include/python2.7 -lpython2.7

SRCS=uskinCanDriver.cpp main.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

uskinCanDriver: $(OBJS)
	$(CXX) $(LDFLAGS) $(PY_LDFLAGS) -o main $(OBJS) $(LDLIBS) 

uskinCanDriver.o: uskinCanDriver.cpp uskinCanDriver.h
	$(CXX) $(CPPFLAGS) -c uskinCanDriver.cpp

main.o: main.cpp 
	$(CXX) $(CPPFLAGS) $(PY_CFLAGS) -c main.cpp


clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) uskinCanDriver
