program_NAME := mfc-server
program_C_SRCS := mfc-server.c ../lib/scn_adapter.c ../lib/extras.c
program_CXX_SRCS := $(wildcard *.cpp)
program_C_OBJS := ${program_C_SRCS:.c=.o}
program_CXX_OBJS := ${program_CXX_SRCS:.cpp=.o}
program_OBJS := $(program_C_OBJS) $(program_CXX_OBJS)
program_INCLUDE_DIRS := ../lib
program_LIBRARY_DIRS := ../lib
program_LIBRARIES :=usb-1.0

CPPFLAGS += $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir)) -Wall
LDFLAGS += $(foreach librarydir,$(program_LIBRARY_DIRS),-L$(librarydir))
LDLIBS += $(foreach library,$(program_LIBRARIES),-l$(library))

.PHONY: all clean distclean

install: $(program_NAME)
	cp $(program_NAME) /opt/mfc/

all: $(program_NAME)

$(program_NAME): $(program_OBJS)
	$(LINK.cc) $(program_OBJS) $(LDLIBS) -o $(program_NAME)
	cp $(program_NAME) /opt/mfc/

clean:
	@- $(RM) $(program_NAME)
	@- $(RM) $(program_OBJS)

distclean: clean
