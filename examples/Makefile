CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-g $(root-config --cflags) -std=c++11
LDFLAGS=-g $(root-config --ldflags)

LDLIBS=$(root-config --libs)

INCLUDELIBS=../include

SRCS= can_communication.cpp uskinCanDriver.cpp main.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

uskinCanDriver: $(OBJS)
	$(CXX) $(LDFLAGS) -o main $(OBJS) $(LDLIBS) 

can_communication.o: $(INCLUDELIBS)/can_communication.cpp $(INCLUDELIBS)/can_communication.h
	$(CXX) $(CPPFLAGS) -c $(INCLUDELIBS)/can_communication.cpp

uskinCanDriver.o: $(INCLUDELIBS)/uskinCanDriver.cpp $(INCLUDELIBS)/uskinCanDriver.h 
	$(CXX) $(CPPFLAGS) -c $(INCLUDELIBS)/uskinCanDriver.cpp

main.o: main.cpp 
	$(CXX) $(CPPFLAGS) -c main.cpp


clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) can_communication

logclean:
	$(RM) CanDriver_log.output uSkinCanDriver_log.output
