# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Should be CXXFLAGS, but webots uses CFLAGS for both of C and C++
CFLAGS = -Wall -Wextra -std=c++14 -pedantic -O2 -pthread
LFLAGS = -pthread

CFLAGS += -I ../../extlibs/bonefish/include \
	-I ../../extlibs/msgpack-c/include \
	-I ../../extlibs/autobahn-cpp/include

CXX_SOURCES += supervisor.cpp game.cpp image_buffer.cpp

LIBRARIES = -lbonefish -lboost_system -lboost_thread -lboost_filesystem

# Do not modify the following: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Following lines need os detection
ifneq ($(OSTYPE),windows) # webots for windows contains boost
CFLAGS += -I ../../extlibs/boost-1.65.1/include
LFLAGS += -L ../../extlibs/boost-1.65.1/lib/$(OSTYPE)
endif

LFLAGS += -L ../../extlibs/bonefish/lib/$(OSTYPE)
