RACK_DIR ?= ../..

SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/noise-plethora/*/*.cpp)
FLAGS += -Ilibs/sst-basic-blocks/include -Ilibs/sst-filters/include -std=c++17

DISTRIBUTABLES += $(wildcard LICENSE*) res

include $(RACK_DIR)/plugin.mk
