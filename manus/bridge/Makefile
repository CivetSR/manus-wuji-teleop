# Manus SDK skeleton bridge
# Usage:
#   make MANUS_SDK=/path/to/ManusSDK
#   ./skeleton_bridge.out

MANUS_SDK ?= $(HOME)/ManusSDK
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
LDFLAGS ?= -Wl,-rpath,$(MANUS_SDK)/lib

INCLUDES := -I$(MANUS_SDK)/include
LIBS := -L$(MANUS_SDK)/lib -lManusSDK_Integrated -lpthread -lusb-1.0 -ludev -lz

TARGET := skeleton_bridge.out
PAIR_TARGET := pair_gloves.out
HAPTIC_TARGET := haptic_cycle.out
SRC := skeleton_bridge.cpp
PAIR_SRC := pair_gloves.cpp
HAPTIC_SRC := haptic_cycle.cpp

.PHONY: all clean check-sdk pair haptic

all: check-sdk $(TARGET)

pair: check-sdk $(PAIR_TARGET)

haptic: check-sdk $(HAPTIC_TARGET)

check-sdk:
	@test -f "$(MANUS_SDK)/include/ManusSDK.h" || (echo "Set MANUS_SDK to your Manus SDK path." && exit 1)
	@test -f "$(MANUS_SDK)/lib/libManusSDK_Integrated.so" || (echo "libManusSDK_Integrated.so not found in $(MANUS_SDK)/lib" && exit 1)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(LDFLAGS) $(LIBS)

$(PAIR_TARGET): $(PAIR_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(LDFLAGS) $(LIBS)

$(HAPTIC_TARGET): $(HAPTIC_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET) $(PAIR_TARGET) $(HAPTIC_TARGET)
