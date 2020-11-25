TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = microMo
# microMo.dbd will be created and installed
DBD += microMo.dbd

# microMo.dbd will be made up from these files:
microMo_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
microMo_DBD += asyn.dbd
microMo_DBD += drvAsynSerialPort.dbd
#endif
microMo_DBD += motorSupport.dbd
microMo_DBD += devMicroMo.dbd

# Add all the support libraries needed by this IOC
microMo_LIBS += MicroMo
microMo_LIBS += motor
#ifdef ASYN
microMo_LIBS += asyn
#endif
#ifdef SNCSEQ
microMo_LIBS += seq pv
#endif

# microMo_registerRecordDeviceDriver.cpp derives from microMo.dbd
microMo_SRCS += microMo_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
microMo_SRCS_DEFAULT += microMoMain.cpp
microMo_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#microMo_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
microMo_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

