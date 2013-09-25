BOOST_HEADER_LOCATION = /opt/local/include
BOOST_LIB_LOCATION = /opt/local/lib

CXX = g++
INCLUDES = -I ${BOOST_HEADER_LOCATION} #-include ../install/*.d
LINKS = -L${BOOST_LIB_LOCATION}
CPPFLAGS = -MMD -O3 $(INCLUDES)

BIN = ../install/main

SRCS = Network.cpp \
       Task.cpp \
       Main.cpp \
       UDCNetwork.cpp \
       State.cpp

OBJS=$(patsubst %.cpp,../install/%.o,$(SRCS))

all : install

install : $(OBJS)
	$(CXX) -g $(OBJS) -o $(BIN)  -lboost_program_options $(LINKS)

../install/%.o: %.cpp 
	    $(CXX) $(CPPFLAGS) -c $< -o ../install/$(subst .cpp,.o,$<)
clean :
	rm -rf ../install/*.o ../install/main
