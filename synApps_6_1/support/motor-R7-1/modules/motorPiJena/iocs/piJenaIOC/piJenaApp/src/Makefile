TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = piJena
# piJena.dbd will be created and installed
DBD += piJena.dbd

# piJena.dbd will be made up from these files:
piJena_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
piJena_DBD += asyn.dbd
piJena_DBD += drvAsynSerialPort.dbd
#endif
piJena_DBD += motorSupport.dbd
piJena_DBD += devPIJena.dbd

# Add all the support libraries needed by this IOC
piJena_LIBS += PIJena
piJena_LIBS += motor
#ifdef ASYN
piJena_LIBS += asyn
#endif
#ifdef SNCSEQ
piJena_LIBS += seq pv
#endif

# piJena_registerRecordDeviceDriver.cpp derives from piJena.dbd
piJena_SRCS += piJena_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
piJena_SRCS_DEFAULT += piJenaMain.cpp
piJena_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#piJena_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
piJena_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

