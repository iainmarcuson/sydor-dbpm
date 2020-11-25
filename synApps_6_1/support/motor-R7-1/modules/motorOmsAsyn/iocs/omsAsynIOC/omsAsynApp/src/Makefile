TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = omsAsyn
# omsAsyn.dbd will be created and installed
DBD += omsAsyn.dbd

# omsAsyn.dbd will be made up from these files:
omsAsyn_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
omsAsyn_DBD += asyn.dbd
omsAsyn_DBD += drvAsynSerialPort.dbd
omsAsyn_DBD += drvAsynIPPort.dbd
#endif
omsAsyn_DBD += motorSupport.dbd
omsAsyn_DBD += omsAsynSupport.dbd

# Add all the support libraries needed by this IOC
omsAsyn_LIBS += omsAsyn
omsAsyn_LIBS += motor
#ifdef ASYN
omsAsyn_LIBS += asyn
#endif
#ifdef SNCSEQ
omsAsyn_LIBS += seq pv
#endif

# omsAsyn_registerRecordDeviceDriver.cpp derives from omsAsyn.dbd
omsAsyn_SRCS += omsAsyn_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
omsAsyn_SRCS_DEFAULT += omsAsynMain.cpp
omsAsyn_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#omsAsyn_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
omsAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

