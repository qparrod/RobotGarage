program_NAME := angryRobotHost
program_CXX_SRCS := $(wildcard src/*.cpp)
program_CXX_OBJS := ${program_CXX_SRCS:.cpp=.o}
program_OBJS := $(program_CXX_OBJS)

test_srcs := $(wildcard test/*.cpp)
test_obj := ${test_srcs:.cpp:.o}
test_name := angryRobotUt

CD=cd
MAKE=make

CXX = g++
CPPFLAGS += -std=c++11 -O2 -Wall -DUNITTEST

all: $(program_NAME) target

host: $(program_NAME)

target:
	@echo "\nbuilding target"
	@echo "-----------------------"
	$(MAKE) -C src

test: $(test_name)

$(test_name): $(test_obj)
	@- $(LINK.cc) $(test_obj) -o $(test_name)



$(program_NAME): $(program_OBJS)
	@echo "building host\n"
	@- $(LINK.cc) $(program_OBJS) -o $(program_NAME)

clean:
	@- $(RM) $(program_NAME)
	@- $(RM) $(program_OBJS)
	make -C src clean

distclean: clean
