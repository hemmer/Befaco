RACK_DIR ?= ../..

SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/noise-plethora/*/*.cpp)
FLAGS += -Idep/sst-basic-blocks/include -Idep/sst-filters/include -std=c++17

DISTRIBUTABLES += $(wildcard LICENSE*) res

include $(RACK_DIR)/plugin.mk
