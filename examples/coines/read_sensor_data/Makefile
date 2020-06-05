ifeq ($(OS),Windows_NT)
HOME=C:
endif

COINES_INSTALL_PATH ?= $(HOME)/coines

EXAMPLE_FILE = bmi090l_read_sensor_data.c

C_SRCS += ../../../bmi090la.c
C_SRCS += ../../../bmi090lg.c

INCLUDEPATHS += ..
INCLUDEPATHS += ../../..

TARGET ?= PC

include $(COINES_INSTALL_PATH)/coines.mk