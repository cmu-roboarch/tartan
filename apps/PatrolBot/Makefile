INC_DIRS := ./ ../include/ /usr/local/include/opencv4 /usr/include/opencv4
SRC_DIRS := ./
OBJ_DIR := ./obj
BIN_DIR := .

EXE_NAME := patrolbot

include ../makefile.rules

CXXFLAGS +=  -Wno-delete-non-virtual-dtor
LDFLAGS += -lopencv_core -lopencv_imgcodecs -lopencv_dnn -lopencv_imgproc -lopencv_highgui -lopencv_stitching
