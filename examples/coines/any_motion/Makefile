ifeq ($(OS),Windows_NT)
HOME=C:
endif

COINES_INSTALL_PATH ?= $(HOME)/coines

EXAMPLE_FILE = bmi090l_any_motion_interrupt_mcu.c

C_SRCS += ../../../bmi090la.c
C_SRCS += ../../../bmi090lg.c

INCLUDEPATHS += ..
INCLUDEPATHS += ../../..

TARGET ?= MCU_APP20

include $(COINES_INSTALL_PATH)/coines.mk