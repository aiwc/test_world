# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Should be CXXFLAGS, but webots uses CFLAGS for both of C and C++
CFLAGS = -Wall -Wextra -std=c++14 -pedantic -O2
LFLAGS = 

# Do not modify the following: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Following lines need os detection
ifneq ($(OSTYPE),windows)
# need >=boost-1.64. use bundled boost for systems that has lower version of boost
INCLUDE += -I ../../extlibs/boost-1.65.1/include
LIBRARIES += -L ../../extlibs/boost-1.65.1/lib/$(OSTYPE)
BOOST_POSTFIX = 
else
# webots for windows contains boost
# winsock
LIBRARIES += -lws2_32 -lwsock32
BOOST_POSTFIX = -mt
endif

LIBRARIES += -lboost_random$(BOOST_POSTFIX) -lboost_system$(BOOST_POSTFIX)
