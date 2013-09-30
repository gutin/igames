BOOST_HEADER_LOCATION = /opt/local/include
SHELLY_LIB_LOCATION = /opt/local/lib
CORE_LIBS = /usr/lib
INSTALL_DIR = ../install

SYSTEM = x86-64_darwin9_gcc4.0
CPLEX_BASE_PATH = /Users/gutin/Applications/IBM/ILOG/CPLEX_Studio_Academic123
CPLEX_PATH= $(CPLEX_BASE_PATH)/cplex
CONCERT_PATH = $(CPLEX_BASE_PATH)/concert
CPLEX_LIBS = $(CPLEX_PATH)/lib/$(SYSTEM)/static_pic
CPLEX_HEADERS =  $(CPLEX_PATH)/include
CONCERT_LIBS = $(CONCERT_PATH)/lib/$(SYSTEM)/static_pic
CONCERT_HEADERS = $(CONCERT_PATH)/include

VERSION=RELEASE

ifeq "$(VERSION)" "RELEASE"
OPT_LEVEL = -O3
else
OPT_LEVEL = -g
endif

CXX = g++
INCLUDES = -I ${BOOST_HEADER_LOCATION} -I $(CPLEX_HEADERS) -I $(CONCERT_HEADERS) -I$(INSTALL_DIR)/include #-include $(INSTALL_DIR)/*.d
LINKS = -L${BOOST_LIB_LOCATION}  -L$(CORE_LIBS)
CPPFLAGS = -MMD $(OPT_LEVEL) $(INCLUDES) -DIL_STD
LFLAGS = $(OPT_LEVEL) -lboost_program_options $(LINKS)
HEURISTIC_LFLAGS = -framework CoreFoundation -framework IOKit -L$(CPLEX_LIBS) -lilocplex -lcplex -L$(CONCERT_LIBS) -lconcert -lm -lpthread

LIB_NAME = $(INSTALL_DIR)/igames.a
HEURISTIC_LIB_NAME = $(INSTALL_DIR)/higames.a
BIN = $(INSTALL_DIR)/main
TEST_BIN = $(INSTALL_DIR)/unit-tests
HEURISTICS_BIN = $(INSTALL_DIR)/heuristic

LIB_SRCS = Network.cpp \
          Task.cpp \
          UDCNetwork.cpp \
          State.cpp \
          Utils.cpp

LIB_HEADERS = DynamicAlgorithm.hpp \
							State.hpp \
							Task.hpp \
							Utils.hpp \
							Network.hpp \
							UDCNetwork.hpp \
							CommonTypes.hpp

HEURISTIC_LIB_SRCS = StaticAlgorithms.cpp
HEURISTIC_LIB_HEADERS = StaticAlgorithms.hpp \
												DynamicEvaluator.hpp \
							          DynamicEvaluator.cpp

BIN_SRCS = Main.cpp
HBIN_SRCS = HeuristicMain.cpp
TEST_SRCS = test/test.cpp

LIB_OBJS=$(patsubst %.cpp,$(INSTALL_DIR)/%.o,$(LIB_SRCS))
BIN_OBJS=$(patsubst %.cpp,$(INSTALL_DIR)/%.o,$(BIN_SRCS))
TEST_OBJS=$(patsubst %.cpp,$(INSTALL_DIR)/%.o,$(TEST_SRCS))
HEURISTIC_OBJS=$(patsubst %.cpp,$(INSTALL_DIR)/%.o,$(HEURISTIC_LIB_SRCS))
HBIN_OBJS=$(patsubst %.cpp,$(INSTALL_DIR)/%.o,$(HBIN_SRCS))

all : headers install hinstall test run_test

run_test :
	$(INSTALL_DIR)/$(TEST_BIN)

headers :
	if [ ! -d $(INSTALL_DIR)/include ]; then mkdir $(INSTALL_DIR)/include; fi
	if [ ! -d $(INSTALL_DIR)/test ]; then mkdir $(INSTALL_DIR)/test; fi
	cp $(LIB_HEADERS) $(HEURISTIC_LIB_HEADERS) $(INSTALL_DIR)/include/

library : $(LIB_OBJS)
	if [ -f $(LIB_NAME) ]; then rm $(LIB_NAME); fi
	ar -cvq $(LIB_NAME) $(LIB_OBJS)

install : library $(BIN_OBJS)
	$(CXX) $(LFLAGS) $(BIN_OBJS) $(LIB_NAME) -o $(BIN) $(LFLAGS)  

heuristics_lib : library $(HEURISTIC_OBJS)
	if [ -f $(HEURISTIC_LIB_NAME) ]; then rm $(HEURISTIC_LIB_NAME); fi
	ar -cvq $(HEURISTIC_LIB_NAME) $(HEURISTIC_OBJS) $(LIB_NAME)

test : library heuristics_lib $(TEST_OBJS) $(LIB_NAME) $(HEURISTIC_LIB_NAME)
	$(CXX) $(TEST_OBJS) $(LIB_NAME) $(HEURISTIC_LIB_NAME) $(HEURISTIC_LFLAGS) -o $(TEST_BIN) 

hinstall : heuristics_lib $(HBIN_OBJS) $(HEURISTIC_LIB_NAME) $(LIB_NAME)
	 $(CXX) $(HBIN_OBJS) $(LFLAGS) $(HEURISTIC_LIB_NAME) $(LIB_NAME)  $(HEURISTIC_LFLAGS) -o $(HEURISTICS_BIN) 

$(INSTALL_DIR)/%.o: %.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $(INSTALL_DIR)/$(subst .cpp,.o,$<)

clean :
	rm -rf $(INSTALL_DIR)/*
