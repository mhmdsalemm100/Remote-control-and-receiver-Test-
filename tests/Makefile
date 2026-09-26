# Host (PC) simulation tests for the Arduino sketches.
#
#   make            build and run all tests
#   make telemetry  only the telemetry communication tests
#   make flysky     only the FlySky health-test scenarios
#   make clean      remove the build folder

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra
CXXFLAGS += -Iarduino_mock

TELEMETRY_SKETCHES = $(wildcard ../01_Telemetry_Radio_Communication/*/*.ino)
FLYSKY_SKETCHES    = $(wildcard ../02_FlySky_Receiver_Health_Test/*/*.ino)

.PHONY: all telemetry flysky clean

all: telemetry flysky

build:
	mkdir -p build

build/test_telemetry: test_telemetry.cpp arduino_mock/Arduino.h $(TELEMETRY_SKETCHES) | build
	$(CXX) $(CXXFLAGS) $< -o $@

build/test_flysky: test_flysky.cpp arduino_mock/Arduino.h $(FLYSKY_SKETCHES) | build
	$(CXX) $(CXXFLAGS) $< -o $@

telemetry: build/test_telemetry
	./build/test_telemetry

flysky: build/test_flysky
	./build/test_flysky

clean:
	rm -rf build
