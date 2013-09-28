BOOST_HEADER_LOCATION = /opt/local/include
BOOST_LIB_LOCATION = /opt/local/lib
VERSION=RELEASE

ifeq "$(VERSION)" "RELEASE"
OPT_LEVEL = -O3
else
OPT_LEVEL = -g
endif

CXX = g++
INCLUDES = -I ${BOOST_HEADER_LOCATION} -I../install/include #-include ../install/*.d
LINKS = -L${BOOST_LIB_LOCATION}
CPPFLAGS = -MMD $(OPT_LEVEL) $(INCLUDES)
LFLAGS = $(OPT_LEVEL) -lboost_program_options $(LINKS)

LIB_NAME = ../install/igames.a
BIN = ../install/main
TEST_BIN = ../install/unit-tests

LIB_SRCS = Network.cpp \
          Task.cpp \
          UDCNetwork.cpp \
          State.cpp \
          Utils.cpp

LIB_HEADERS = DynamicAlgorithm.hpp \
							DynamicEvaluator.hpp \
							DynamicEvaluator.cpp \
							State.hpp \
							Task.hpp \
							Utils.hpp \
							Network.hpp \
							UDCNetwork.hpp \
							CommonTypes.hpp

BIN_SRCS = Main.cpp

TEST_SRCS = test/test.cpp

LIB_OBJS=$(patsubst %.cpp,../install/%.o,$(LIB_SRCS))
BIN_OBJS=$(patsubst %.cpp,../install/%.o,$(BIN_SRCS))
TEST_OBJS=$(patsubst %.cpp,../install/%.o,$(TEST_SRCS))

all : headers install test run_test

run_test :
	../install/$(TEST_BIN)

headers :
	if [ ! -d ../install/include ]; then mkdir ../install/include; fi
	cp $(LIB_HEADERS) ../install/include/

library : $(LIB_OBJS)
	if [ -f $(LIB_NAME) ]; then rm $(LIB_NAME); fi
	ar -cvq $(LIB_NAME) $(LIB_OBJS)

install : library $(BIN_OBJS)
	$(CXX) $(LFLAGS) $(BIN_OBJS) $(LIB_NAME) -o $(BIN) $(LFLAGS)  

test : library $(TEST_OBJS) $(LIB_NAME)
	$(CXX) $(LFLAGS) $(TEST_OBJS) $(LIB_NAME) -o $(TEST_BIN) $(LFLAGS)  

../install/%.o: %.cpp 
	if [ ! -d ../install/test ]; then mkdir ../install/test; fi
	$(CXX) $(CPPFLAGS) -c $< -o ../install/$(subst .cpp,.o,$<)

clean :
	rm -rf ../install/*
