BOOST_LOCATION = /opt/local/include
CXX = g++
INCLUDES = -I ${BOOST_LOCATION} #-include ../install/*.d
CPPFLAGS = -MMD -g $(INCLUDES)

BIN = ../install/main

SRCS = Network.cpp \
       Task.cpp \
       Main.cpp \
       UDCNetwork.cpp \
       State.cpp

OBJS=$(patsubst %.cpp,../install/%.o,$(SRCS))

all : install

install : $(OBJS)
	$(CXX) -g $(OBJS) -o $(BIN)

../install/%.o: %.cpp 
	    $(CXX) $(CPPFLAGS) -c $< -o ../install/$(subst .cpp,.o,$<)
clean :
	rm -rf ../install/*.o ../install/main
