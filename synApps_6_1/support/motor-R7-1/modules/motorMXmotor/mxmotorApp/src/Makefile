# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
USR_CFLAGS += -DDEBUG
USR_CXXFLAGS += -DDEBUG

LIBRARY_IOC_DEFAULT = MXmotor
LIBRARY_IOC_vxWorks = -nil-

ifdef MX
DBD += devMXmotor.dbd

SRCS_DEFAULT += MXRegister.cc

# The following is required for the Soft Channel (i.e., MotorSoft) device driver.
SRCS_DEFAULT += devMXmotor.cc drvMXmotor.cc
endif

MXmotor_LIBS += motor
ifdef MX
MXmotor_LIBS += Mx
endif
MXmotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

