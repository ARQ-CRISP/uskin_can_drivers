CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-g $(root-config --cflags) -std=c++11
LDFLAGS=-g $(root-config --ldflags)

LDLIBS=$(root-config --libs)

INCLUDEDIR=../include
INCLUDESRC=../src

SRCS= can_communication.cpp uskinCanDriver.cpp main.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

uskinCanDriver: $(OBJS)
	$(CXX) $(LDFLAGS) -o main $(OBJS) $(LDLIBS) 

can_communication.o: $(INCLUDESRC)/can_communication.cpp $(INCLUDEDIR)/can_communication.h
	$(CXX) $(CPPFLAGS) -c $(INCLUDESRC)/can_communication.cpp

uskinCanDriver.o: $(INCLUDESRC)/uskinCanDriver.cpp $(INCLUDEDIR)/uskinCanDriver.h 
	$(CXX) $(CPPFLAGS) -c $(INCLUDESRC)/uskinCanDriver.cpp

main.o: main.cpp 
	$(CXX) $(CPPFLAGS) -c main.cpp


clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) can_communication

logclean:
	$(RM) CanDriver_log.output uSkinCanDriver_log.output
